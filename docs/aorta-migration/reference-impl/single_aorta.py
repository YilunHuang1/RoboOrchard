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

"""Aorta-only Piper single-arm controller (migration PR-B draft).

Direct rewrite of ``robo_orchard_piper_ros2/single.py`` onto the Aorta Python
SDK. Same universal single-arm node reused for BOTH master and slave roles
(differentiated by topic-name args + params, replacing the old ROS2 launch
remaps). CAN control is unchanged — it lives in ``arm_bridge`` (dataclasses).

Key migration decisions (see docs/aorta-migration-plan.md §4.3, §5):
  - ``joint_cmd`` uses a PULL subscriber drained inside the 200 Hz timer, not a
    callback. This preserves depth-1 latest-wins AND keeps every CAN access on
    the single executor thread (no Zenoh-worker-thread race on the piper handle).
  - The 200 Hz timer + both services are bound to ONE ExecutionGroup(SERIALIZED),
    reproducing rclpy single-threaded spin serialization for CAN safety.
  - Topic/service names are args (no launch remap layer in Aorta). Set AORTA_GROUP
    per process so master (desktop) and slave (S100) share a group.

NOTE: this draft needs (a) libaorta_core on the host (AORTA_CORE_FFI_LIB) and
(b) the generated schema-meta modules from the aorta repo's arm .fbs
(``*_schema_meta_py_lib`` targets). It is not runnable in a repo without the
Aorta runtime; it is the reviewable code for PR-B.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import threading
import time

import aorta

from arm_bridge import (
    JointStateData,
    create_piper,
    enable_arm_ctrl,
    get_arm_ee_pose,
    get_arm_state,
    get_arm_status,
    joint_control,
    set_ctrl_method,
    switch_piper_ctrl_mode,
)

# Generated Aorta schema-meta modules (from aorta repo arm .fbs). Each bundles
# the FlatBuffers accessors/builders + SCHEMA_BFBS/hash (like demo_chat_schema_meta).
import arm_joint_state_schema_meta as JS   # ArmJointState  (joint_state + joint_cmd)
import arm_ee_pose_schema_meta as EE       # ArmEePose (+ CreateVec3 / CreateQuat)
import piper_status_schema_meta as ST      # PiperStatus
import arm_trigger_schema_meta as TRIG     # ArmTriggerRequest / ArmTriggerResponse

# NOTE: EE.CreateVec3 / EE.CreateQuat are the generated struct creators for the
# Vec3 / Quat structs in arm_ee_pose.fbs. If the aorta `aorta_fbs_library`
# schema-meta bundle does NOT re-export them, import them from the generated
# flatbuffers modules instead (e.g. `from arm.Vec3 import CreateVec3`). Verified
# against flatc output in docs/aorta-migration/reference-impl/validate_fill.py.

# aorta.services.common.ServiceStatus enum values (see status.fbs)
SVC_SUCCESS = 0
SVC_UNAVAILABLE = 3
SVC_INTERNAL_ERROR = 4

log = logging.getLogger("piper_single_ctrl")


class PiperSingleControlNode:
    def __init__(self, node: aorta.Node, args: argparse.Namespace) -> None:
        self.node = node
        self.can_port = args.can_port
        self.gripper_exist = args.gripper_exist
        self.gripper_val_mutiple = max(0, min(int(args.gripper_val_mutiple), 10))
        self.auto_enable_arm_ctrl = args.auto_enable_arm_ctrl
        self.enable_mit_ctrl = args.enable_mit_ctrl

        log.info(
            "can_port=%s auto_enable_arm_ctrl=%s gripper_exist=%s "
            "gripper_val_mutiple=%s enable_mit_ctrl=%s",
            self.can_port, self.auto_enable_arm_ctrl, self.gripper_exist,
            self.gripper_val_mutiple, self.enable_mit_ctrl,
        )

        self.piper = create_piper(self.can_port)

        self._enable_flag = False
        if self.auto_enable_arm_ctrl:
            if self.enable_arm_ctrl():
                log.info("Auto enable successed!")
            else:
                log.warning("Auto enable failed! Maybe in teach mode?")

        # Serialize timer + service callbacks onto one executor thread (= rclpy
        # single-thread spin) so nothing races the CAN handle.
        grp = node.create_execution_group(aorta.ExecutionPolicy.SERIALIZED)

        # Publishers (typed). realtime_control for the control-loop streams,
        # state_update for the lower-rate status.
        self.joint_pub = node.create_publisher_typed(
            JS, args.joint_state_topic, qos=aorta.QoS.realtime_control()
        )
        self.status_pub = node.create_publisher_typed(
            ST, args.status_topic, qos=aorta.QoS.state_update()
        )
        self.ee_pose_pub = node.create_publisher_typed(
            EE, args.ee_pose_topic, qos=aorta.QoS.realtime_control()
        )

        # Services (typed, Trigger-shaped), serialized with the timer.
        node.create_service_typed_view(
            TRIG.ArmTriggerRequest.GetRootAs, args.enable_service,
            self._enable_ctrl_service_callback,
            request_schema_meta=TRIG, response_schema_meta=TRIG,
            options=grp.service_options(),
        )
        node.create_service_typed_view(
            TRIG.ArmTriggerRequest.GetRootAs, args.reset_service,
            self._reset_ctrl_service_callback,
            request_schema_meta=TRIG, response_schema_meta=TRIG,
            options=grp.service_options(),
        )

        # joint_cmd: pull subscriber (latest-wins, depth 1). Drained in the timer.
        self.cmd_sub = node.create_subscriber_pull(
            args.joint_cmd_topic, 1, qos=aorta.QoS.realtime_control()
        )

        # 200 Hz control+publish loop on the serialized executor thread.
        self.timer = node.create_timer(
            1 / 200.0, self.publish_callback, options=grp.timer_options()
        )

    # ── control-mode helpers (unchanged logic) ───────────────────────────────
    def is_controlable(self) -> bool:
        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        return self._enable_flag and ctrl_mode == 0x01

    def enable_arm_ctrl(self, force_reset: bool = False) -> bool:
        arm_status = self.piper.GetArmStatus().arm_status
        ctrl_mode = arm_status.ctrl_mode
        is_post_teach_recovery = ctrl_mode == 0x02
        if ctrl_mode == 0x02:
            if arm_status.teach_status == 1:
                return False
            log.warning("ctrl_mode is %s, switch directly to ctrl mode...", ctrl_mode)
            switch_piper_ctrl_mode(self.piper, 0x01)
        else:
            enable_arm_ctrl(self.piper)
        set_ctrl_method(piper=self.piper, is_mit=self.enable_mit_ctrl)
        if (
            not is_post_teach_recovery
            and self.piper.GetArmStatus().arm_status.ctrl_mode != 0x01
        ):
            return False
        self._enable_flag = True
        return True

    # ── 200 Hz loop: drain joint_cmd, then publish state ─────────────────────
    def publish_callback(self) -> None:
        # 1) apply the latest joint command (drop stale; keep only newest)
        latest = None
        while True:
            sample = self.cmd_sub.try_recv()
            if sample is None:
                break
            latest = sample
        if latest is not None and self.is_controlable():
            cmd = self._decode_joint_cmd(latest.payload)
            joint_control(
                self.piper, joint_data=cmd,
                has_gripper=self.gripper_exist,
                gripper_val_mutiple=self.gripper_val_mutiple,
            )

        # 2) publish status / joint_state / ee_pose
        stamp = self.node.now()
        status = get_arm_status(self.piper)
        self.status_pub.publish_typed(self._fill_status(status))

        js = get_arm_state(self.piper)
        self.joint_pub.publish_typed(self._fill_joint_state(js, stamp))

        pose = get_arm_ee_pose(self.piper)
        self.ee_pose_pub.publish_typed(self._fill_ee_pose(pose, stamp))

    @staticmethod
    def _decode_joint_cmd(payload: bytes) -> JointStateData:
        m = JS.ArmJointState.GetRootAs(payload, 0)
        name = [m.Name(i).decode() for i in range(m.NameLength())]
        position = [m.Position(i) for i in range(m.PositionLength())]
        # slave joint_control() only reads name/position; velocity/effort ignored
        return JointStateData(name=name or None, position=position)

    # ── FlatBuffers fill callbacks (header injected by publish_typed) ─────────
    @staticmethod
    def _fill_joint_state(js: JointStateData, stamp: int):
        def fill(b, header_off):
            name_offs = [b.CreateString(n) for n in js.name]
            JS.ArmJointStateStartNameVector(b, len(name_offs))
            for o in reversed(name_offs):
                b.PrependUOffsetTRelative(o)
            name_vec = b.EndVector()

            JS.ArmJointStateStartPositionVector(b, len(js.position))
            for v in reversed(js.position):
                b.PrependFloat64(v)
            pos_vec = b.EndVector()

            JS.ArmJointStateStartVelocityVector(b, len(js.velocity))
            for v in reversed(js.velocity):
                b.PrependFloat64(v)
            vel_vec = b.EndVector()

            JS.ArmJointStateStartEffortVector(b, len(js.effort))
            for v in reversed(js.effort):
                b.PrependFloat64(v)
            eff_vec = b.EndVector()

            JS.ArmJointStateStart(b)
            JS.ArmJointStateAddAortaHeader(b, header_off)
            JS.ArmJointStateAddStampNs(b, stamp)
            JS.ArmJointStateAddName(b, name_vec)
            JS.ArmJointStateAddPosition(b, pos_vec)
            JS.ArmJointStateAddVelocity(b, vel_vec)
            JS.ArmJointStateAddEffort(b, eff_vec)
            return JS.ArmJointStateEnd(b)

        return fill

    @staticmethod
    def _fill_ee_pose(pose, stamp: int):
        def fill(b, header_off):
            EE.ArmEePoseStart(b)
            EE.ArmEePoseAddAortaHeader(b, header_off)
            EE.ArmEePoseAddStampNs(b, stamp)
            EE.ArmEePoseAddPosition(b, EE.CreateVec3(b, pose.px, pose.py, pose.pz))
            EE.ArmEePoseAddOrientation(
                b, EE.CreateQuat(b, pose.ox, pose.oy, pose.oz, pose.ow)
            )
            return EE.ArmEePoseEnd(b)

        return fill

    @staticmethod
    def _fill_status(s):
        def fill(b, header_off):
            ST.PiperStatusStart(b)
            ST.PiperStatusAddAortaHeader(b, header_off)
            ST.PiperStatusAddCtrlMode(b, s.ctrl_mode)
            ST.PiperStatusAddArmStatus(b, s.arm_status)
            ST.PiperStatusAddModeFeedback(b, s.mode_feedback)
            ST.PiperStatusAddTeachStatus(b, s.teach_status)
            ST.PiperStatusAddMotionStatus(b, s.motion_status)
            ST.PiperStatusAddTrajectoryNum(b, s.trajectory_num)
            ST.PiperStatusAddErrCode(b, s.err_code)
            ST.PiperStatusAddJoint1AngleLimit(b, s.joint_1_angle_limit)
            ST.PiperStatusAddJoint2AngleLimit(b, s.joint_2_angle_limit)
            ST.PiperStatusAddJoint3AngleLimit(b, s.joint_3_angle_limit)
            ST.PiperStatusAddJoint4AngleLimit(b, s.joint_4_angle_limit)
            ST.PiperStatusAddJoint5AngleLimit(b, s.joint_5_angle_limit)
            ST.PiperStatusAddJoint6AngleLimit(b, s.joint_6_angle_limit)
            ST.PiperStatusAddCommunicationStatusJoint1(b, s.communication_status_joint_1)
            ST.PiperStatusAddCommunicationStatusJoint2(b, s.communication_status_joint_2)
            ST.PiperStatusAddCommunicationStatusJoint3(b, s.communication_status_joint_3)
            ST.PiperStatusAddCommunicationStatusJoint4(b, s.communication_status_joint_4)
            ST.PiperStatusAddCommunicationStatusJoint5(b, s.communication_status_joint_5)
            ST.PiperStatusAddCommunicationStatusJoint6(b, s.communication_status_joint_6)
            return ST.PiperStatusEnd(b)

        return fill

    def _reply_trigger(self, responder, ok: bool, message: str) -> None:
        status = SVC_SUCCESS if ok else SVC_INTERNAL_ERROR

        def fill(b, header_off):
            msg_off = b.CreateString(message)
            TRIG.ArmTriggerResponseStart(b)
            TRIG.ArmTriggerResponseAddAortaHeader(b, header_off)
            TRIG.ArmTriggerResponseAddStatus(b, status)
            TRIG.ArmTriggerResponseAddMessage(b, msg_off)
            return TRIG.ArmTriggerResponseEnd(b)

        responder.reply(fill)

    # ── services (unchanged logic, Aorta reply) ──────────────────────────────
    def _enable_ctrl_service_callback(self, request, responder) -> None:
        log.info("Received request to enable arm.")
        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        if ctrl_mode != 0x02 and self._enable_flag:
            self._reply_trigger(responder, True, "Arm is already enabled.")
            return
        try:
            if self.enable_arm_ctrl(force_reset=True):
                self._reply_trigger(responder, True, "Arm enabled successfully.")
            else:
                self._reply_trigger(
                    responder, False,
                    "Failed to enable arm. It might be in an unrecoverable state.",
                )
        except Exception as e:  # noqa: BLE001
            log.error("Error while enabling arm: %s", e)
            self._reply_trigger(responder, False, f"An unexpected error occurred: {e}")

    def _reset_ctrl_service_callback(self, request, responder) -> None:
        log.info("Received request to reset arm.")
        if not self.is_controlable():
            msg = "Arm is not controllable, reset failed."
            log.error(msg)
            self._reply_trigger(responder, False, msg)
            return

        control_freq = 200.0

        def _gen_reset_traj():
            cur = get_arm_state(self.piper)
            target = [0.0] * 7
            num_steps = int(3.0 * control_freq)
            traj = []
            for step in range(num_steps):
                interp = [
                    cur.position[i]
                    + (target[i] - cur.position[i]) * (step + 1) / num_steps
                    for i in range(7)
                ]
                traj.append(
                    JointStateData(
                        name=cur.name, position=interp,
                        velocity=cur.velocity, effort=cur.effort,
                    )
                )
            return traj

        try:
            for waypoint in _gen_reset_traj():
                joint_control(
                    self.piper, joint_data=waypoint,
                    has_gripper=self.gripper_exist,
                    gripper_val_mutiple=self.gripper_val_mutiple,
                )
                time.sleep(1.0 / control_freq)
            self._reply_trigger(responder, True, "Arm reset successfully.")
        except Exception as e:  # noqa: BLE001
            log.error("Error while resetting arm: %s", e)
            self._reply_trigger(responder, False, f"An unexpected error occurred: {e}")


def _parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aorta Piper single-arm controller")
    p.add_argument("--node-name", default="piper_single_ctrl")
    # control params (were ROS params)
    p.add_argument("--can-port", default=os.environ.get("ROBO_ORCHARD_CAN_PORT", "can0"))
    p.add_argument("--gripper-exist", type=_boolish, default=True)
    p.add_argument("--gripper-val-mutiple", type=int, default=1)
    p.add_argument("--auto-enable-arm-ctrl", type=_boolish,
                   default=_boolish(os.environ.get("ROBO_ORCHARD_AUTO_ENABLE_ARM_CTRL", "false")))
    p.add_argument("--enable-mit-ctrl", type=_boolish, default=False)
    # topic/service names (were launch remaps). slave defaults => /puppet/*.
    p.add_argument("--joint-state-topic", default="aorta/puppet/joint_left")
    p.add_argument("--status-topic", default="aorta/puppet/status_left")
    p.add_argument("--ee-pose-topic", default="aorta/puppet/end_pose_left")
    p.add_argument("--joint-cmd-topic", default="aorta/robot/left/joint_cmd")
    p.add_argument("--enable-service", default="aorta/robot/left/enable_ctrl")
    p.add_argument("--reset-service", default="aorta/robot/left/reset_ctrl")
    return p.parse_args(argv)


def _boolish(v) -> bool:
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO)
    args = _parse_args(argv)

    # AORTA_GROUP (env) must match across master (desktop) and slave (S100).
    node = aorta.Node(args.node_name)
    controller = PiperSingleControlNode(node, args)  # noqa: F841

    # Timers/services run on the auto-executor daemon thread; block main here.
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
