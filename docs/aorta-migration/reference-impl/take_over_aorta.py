# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

# ruff: noqa: E501, N812

"""Aorta-only takeover muxer (migration PR-B draft).

Direct rewrite of ``robo_orchard_teleop_ros2/take_over/node.py`` onto the Aorta
Python SDK. Stateful 3-mode multiplexer (AUTONOMOUS / OVERRIDE / STOP) that
routes the arm command stream and supports human takeover with history replay.

Key migration decisions (see docs/aorta-migration-plan.md §4.3, §5):
  - The muxer is a pure ROUTER: it forwards the *exact* command payload
    (``publish_bytes``), never decoding the JointState. This mirrors ROS2
    ``publish(msg)`` pass-through (the master's original AortaHeader rides
    along) and is allocation-free at 200 Hz. The output publisher is declared
    typed (ArmJointState) only so peers can discover the topic schema.
  - The dynamic ``message_type`` ROS param is GONE — the arm stream is always
    ArmJointState.
  - Input subscribers, services, and the mode timer are bound to ONE
    ExecutionGroup(SERIALIZED), reproducing rclpy single-threaded spin so
    ``_current_mode`` / ``_history`` are never raced.
  - Timestamps: ``node.now()`` (int ns) replaces rclpy Time/Duration.

Behavior preserved exactly, including the original quirk that STOP mode does
NOT actually publish a zero-command (the code never did, despite the message).

Needs libaorta_core + the generated ``*_schema_meta`` modules (see README).
"""

from __future__ import annotations
import argparse
import collections
import logging
import signal
import threading
from enum import Enum

import aorta

# Generated Aorta schema-meta modules (from aorta repo arm .fbs).
import arm_joint_state_schema_meta as JS  # ArmJointState (output topic schema)
import arm_trigger_schema_meta as TRIG  # ArmTriggerRequest / ArmTriggerResponse
import control_mode_schema_meta as CM  # ControlMode
import takeover_event_schema_meta as EV  # TakeOverEvent
from aorta.services.arm.ArmTriggerResponse import (
    ArmTriggerResponseAddAortaHeader,
    ArmTriggerResponseAddMessage,
    ArmTriggerResponseAddStatus,
    ArmTriggerResponseEnd,
    ArmTriggerResponseStart,
)

log = logging.getLogger("takeover_muxer_node")

SVC_SUCCESS = 0
SVC_INTERNAL_ERROR = 4

# TakeOverEvent.event_type constants (were message string-constants).
EV_TAKEOVER_TRIGGERED = "takeover_triggered"
EV_REPLAY_COMMAND_SENT = "replay_command_sent"
EV_RELEASE_TRIGGERED = "release_triggered"
EV_STOP_TRIGGERED = "stop_triggered"


class ControlMode(Enum):
    AUTONOMOUS = "autonomous"
    OVERRIDE = "override"
    STOP = "stop"


class TakeOverMuxerNode:
    def __init__(self, node: aorta.Node, args: argparse.Namespace) -> None:
        self.node = node
        self._algo_topic = args.algo_topic
        self._override_topic = args.override_topic
        self._output_topic = args.output_topic
        self._replay_time_s = args.replay_time_s
        self._override_behavior = args.override_mode_behavior
        if self._override_behavior not in ("forward", "silent"):
            raise ValueError(
                f"Invalid override_mode_behavior: {self._override_behavior}. "
                "Must be `forward` or `silent`."
            )

        # --- internal state ---
        self._current_mode = ControlMode.AUTONOMOUS
        buffer_size = int(200 * (self._replay_time_s + 2.0))
        self._history = collections.deque(
            maxlen=buffer_size
        )  # (recv_ns, payload)

        # Serialize every callback (inputs + services + timer) onto one thread.
        grp = node.create_execution_group(aorta.ExecutionPolicy.SERIALIZED)

        # Output = raw pass-through, but declared typed for schema discovery.
        self._command_pub = node.create_publisher_typed(
            JS, self._output_topic, qos=aorta.QoS.realtime_control()
        )
        self._mode_pub = node.create_publisher_typed(
            CM, args.control_mode_topic, qos=aorta.QoS.state_update()
        )
        self._event_pub = node.create_publisher_typed(
            EV, args.events_topic, qos=aorta.QoS.state_update()
        )

        # algo subscriber: always present (feeds the replay history).
        node.create_subscriber(
            self._algo_topic, self._on_algo, options=grp.subscriber_options()
        )
        # override subscriber: only when we actually forward override commands.
        if self._override_behavior == "forward":
            log.info(
                "Behavior `forward`: subscribing override topic %s",
                self._override_topic,
            )
            node.create_subscriber(
                self._override_topic,
                self._on_override,
                options=grp.subscriber_options(),
            )
        else:
            log.info("Behavior `silent`: skipping override subscriber.")

        # Services.
        for name, cb in (
            (args.takeover_service, self._takeover_service_callback),
            (args.release_service, self._release_service_callback),
            (args.stop_service, self._stop_service_callback),
        ):
            node.create_service_typed_view(
                TRIG.ArmTriggerRequest.GetRootAs,
                name,
                cb,
                request_schema_meta=TRIG,
                response_schema_meta=TRIG,
                options=grp.service_options(),
            )

        # Mode-publish timer.
        node.create_timer(
            1.0 / args.mode_publish_rate_hz,
            self._publish_current_mode,
            options=grp.timer_options(),
        )
        log.info(
            "TakeoverMuxerNode initialized. Default AUTONOMOUS. Override behavior: %s",
            self._override_behavior,
        )

    # ── input routing (raw pass-through) ─────────────────────────────────────
    def _on_algo(self, payload: bytes, ctx) -> None:
        if self._current_mode == ControlMode.STOP:
            return
        if self._current_mode == ControlMode.AUTONOMOUS:
            self._history.append((self.node.now(), payload))
            self._command_pub.publish_bytes(payload)

    def _on_override(self, payload: bytes, ctx) -> None:
        if self._current_mode == ControlMode.STOP:
            return
        if self._current_mode == ControlMode.OVERRIDE:
            # subscriber only exists when behavior == "forward"
            self._command_pub.publish_bytes(payload)

    # ── services ─────────────────────────────────────────────────────────────
    def _takeover_service_callback(self, request, responder) -> None:
        if self._current_mode == ControlMode.OVERRIDE:
            self._reply(responder, True, "Already in OVERRIDE mode.")
            return

        self._publish_event(EV_TAKEOVER_TRIGGERED, "Takeover service called.")
        log.info("Takeover triggered. Switching to OVERRIDE mode.")
        self._current_mode = ControlMode.OVERRIDE

        if self._replay_time_s <= 0.0:
            log.info("Takeover successful, but without any replay.")
            self._publish_current_mode()
            self._reply(
                responder, True, "Takeover successful, but without any replay."
            )
            return

        if not self._history:
            log.warning(
                "Takeover successful, but cannot replay: history buffer is empty."
            )
            self._publish_current_mode()
            self._reply(
                responder,
                True,
                "Takeover successful, but cannot replay: history buffer is empty.",
            )
            return

        target_ns = self.node.now() - int(self._replay_time_s * 1e9)
        found_payload = None
        for recv_ns, payload in reversed(self._history):
            if recv_ns <= target_ns:
                found_payload = payload
                break

        if found_payload is not None:
            log.info("Resetting state to ~%.2fs ago.", self._replay_time_s)
            self._command_pub.publish_bytes(found_payload)
            self._publish_event(
                EV_REPLAY_COMMAND_SENT,
                f"Replayed command from approx {self._replay_time_s:.2f}s ago.",
            )
            message = "Takeover successful. Replay command published."
        else:
            message = "Takeover successful. No message is replay."

        self._publish_current_mode()
        self._reply(responder, True, message)

    def _release_service_callback(self, request, responder) -> None:
        if self._current_mode == ControlMode.AUTONOMOUS:
            self._reply(responder, True, "Already in AUTONOMOUS mode.")
            return
        self._publish_event(
            EV_RELEASE_TRIGGERED, "Release control service called."
        )
        log.info("Control released. Switching back to AUTONOMOUS mode.")
        self._current_mode = ControlMode.AUTONOMOUS
        self._history.clear()
        self._publish_current_mode()
        self._reply(responder, True, "Switched to AUTONOMOUS mode.")

    def _stop_service_callback(self, request, responder) -> None:
        if self._current_mode == ControlMode.STOP:
            self._reply(responder, True, "Already in STOP mode.")
            return
        log.info("Stop triggered. Switching to STOP mode.")
        self._current_mode = ControlMode.STOP
        self._publish_event(EV_STOP_TRIGGERED, "Stop service called.")
        self._publish_current_mode()
        # NOTE: preserves original behavior — no zero-command is actually sent.
        self._reply(
            responder, True, "Switched to STOP mode and sent a zero-command."
        )

    # ── typed publishers / reply ─────────────────────────────────────────────
    def _publish_current_mode(self) -> None:
        stamp = self.node.now()
        value = self._current_mode.value

        def fill(b, header_off):
            data_off = b.CreateString(value)
            CM.ControlModeStart(b)
            CM.ControlModeAddAortaHeader(b, header_off)
            CM.ControlModeAddStampNs(b, stamp)
            CM.ControlModeAddData(b, data_off)
            return CM.ControlModeEnd(b)

        self._mode_pub.publish_typed(fill)

    def _publish_event(self, event_type: str, details: str) -> None:
        stamp = self.node.now()

        def fill(b, header_off):
            et_off = b.CreateString(event_type)
            det_off = b.CreateString(details)
            EV.TakeOverEventStart(b)
            EV.TakeOverEventAddAortaHeader(b, header_off)
            EV.TakeOverEventAddStampNs(b, stamp)
            EV.TakeOverEventAddEventType(b, et_off)
            EV.TakeOverEventAddDetails(b, det_off)
            return EV.TakeOverEventEnd(b)

        self._event_pub.publish_typed(fill)

    @staticmethod
    def _reply(responder, ok: bool, message: str) -> None:
        status = SVC_SUCCESS if ok else SVC_INTERNAL_ERROR

        def fill(b, header_off):
            msg_off = b.CreateString(message)
            ArmTriggerResponseStart(b)
            ArmTriggerResponseAddAortaHeader(b, header_off)
            ArmTriggerResponseAddStatus(b, status)
            ArmTriggerResponseAddMessage(b, msg_off)
            return ArmTriggerResponseEnd(b)

        responder.reply(fill)


def _parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aorta takeover muxer")
    p.add_argument("--node-name", default="takeover_muxer_node")
    p.add_argument("--algo-topic", default="aorta/left_algo_cmd")
    p.add_argument("--override-topic", default="aorta/master/joint_left")
    p.add_argument("--output-topic", default="aorta/robot/left/joint_cmd")
    p.add_argument(
        "--control-mode-topic",
        default="aorta/robot/left/takeover_muxer/control_mode",
    )
    p.add_argument(
        "--events-topic", default="aorta/robot/left/takeover_muxer/events"
    )
    p.add_argument(
        "--takeover-service",
        default="aorta/robot/left/takeover_muxer/trigger_takeover",
    )
    p.add_argument(
        "--release-service",
        default="aorta/robot/left/takeover_muxer/release_control",
    )
    p.add_argument(
        "--stop-service", default="aorta/robot/left/takeover_muxer/stop"
    )
    p.add_argument("--replay-time-s", type=float, default=2.0)
    p.add_argument("--mode-publish-rate-hz", type=float, default=1.0)
    p.add_argument(
        "--override-mode-behavior",
        default="forward",
        choices=["forward", "silent"],
    )
    return p.parse_args(argv)


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO)
    args = _parse_args(argv)
    node = aorta.Node(args.node_name)
    muxer = TakeOverMuxerNode(node, args)  # noqa: F841

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        stop.wait()
    finally:
        node.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
