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

"""Aorta-only keyboard takeover trigger (migration PR-B draft).

Direct rewrite of ``robo_orchard_teleop_ros2/take_over/trigger/keyboard.py``.
Maps typed words (e.g. ``trigger`` / ``release``) to sets of Aorta Trigger
services. The ``sshkeyboard`` input source is unchanged.

Difference from ROS2: Aorta's typed client ``call(fill)`` is SYNCHRONOUS (no
``call_async`` / future). Each matched service is called on a short-lived worker
thread so the keyboard listener is never blocked.

Needs libaorta_core + the generated ``arm_trigger_schema_meta`` module.
"""

from __future__ import annotations
import argparse
import json
import logging
import signal
import threading

import aorta
import arm_trigger_schema_meta as TRIG  # ArmTriggerRequest / ArmTriggerResponse
from aorta.services.arm.ArmTriggerResponse import ArmTriggerResponse

log = logging.getLogger("multi_keyboard_trigger_node")

try:
    from sshkeyboard import listen_keyboard, stop_listening
except ImportError:  # keep importable for offline review
    listen_keyboard = stop_listening = None

SVC_SUCCESS = 0  # aorta.services.common.ServiceStatus.SUCCESS


class Colors:
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"
    CLEAR_LINE = "\033[K"


def _request_fill(b, header_off):
    """ArmTriggerRequest is header-only (mirrors std_srvs/Trigger empty request)."""
    TRIG.ArmTriggerRequestStart(b)
    TRIG.ArmTriggerRequestAddAortaHeader(b, header_off)
    return TRIG.ArmTriggerRequestEnd(b)


class KeyboardTriggerNode:
    def __init__(self, node: aorta.Node, config_path: str) -> None:
        self.node = node

        with open(config_path, "r") as fh:
            self._key_service_map = json.load(fh)
        if not isinstance(self._key_service_map, dict):
            raise ValueError("config must be a dict of {key: [service, ...]}")

        self._clients = {}
        self._input_buffer = ""
        self._lock = threading.Lock()

        log.info("Configuring multi-keyboard trigger...")
        for key, service_list in self._key_service_map.items():
            log.info(
                "- Type `%s` and press Enter ==> Calls %s", key, service_list
            )
            for service_name in service_list:
                if service_name not in self._clients:
                    self._clients[service_name] = node.create_client_typed(
                        service_name,
                        ArmTriggerResponse.GetRootAs,
                        request_schema_meta=TRIG,
                    )

        keys = list(self._key_service_map.keys())
        if keys:
            colored = f"{Colors.BOLD}{Colors.CYAN}[ {', '.join(keys)} ]{Colors.RESET}"
            log.info("Hint: Valid commands are -> %s", colored)

        self._listener_thread = threading.Thread(
            target=self._keyboard_listener_loop, daemon=True
        )
        self._listener_thread.start()
        log.info("Keyboard listener started. Ready for triggers.")

    def _keyboard_listener_loop(self):
        listen_keyboard(
            on_press=self._on_key_press, delay_second_char=0.05, lower=False
        )

    def _on_key_press(self, key: str):
        with self._lock:
            if key == "enter":
                if self._input_buffer in self._key_service_map:
                    self._trigger_services_for_key(self._input_buffer)
                else:
                    log.warning(
                        "No action defined for input: '%s'", self._input_buffer
                    )
                self._input_buffer = ""
            elif key == "backspace":
                self._input_buffer = self._input_buffer[:-1]
            elif len(key) == 1:
                self._input_buffer += key
            print(
                f"Input: {Colors.BOLD}{Colors.CYAN}{self._input_buffer}"
                f"{Colors.RESET}{Colors.CLEAR_LINE}",
                end="\r",
                flush=True,
            )

    def _trigger_services_for_key(self, key: str):
        service_names = self._key_service_map.get(key, [])
        log.info(
            "Key binding `%s` matched. Calling %d service(s).",
            key,
            len(service_names),
        )
        for service_name in service_names:
            client = self._clients.get(service_name)
            if client is None or not client.raw().matching_status():
                log.warning(
                    "Service `%s` is not available. Trigger ignored.",
                    service_name,
                )
                continue
            # Aorta call() is blocking; run each on a worker so the listener stays live.
            threading.Thread(
                target=self._call_service,
                args=(client, service_name),
                daemon=True,
            ).start()

    def _call_service(self, client, service_name: str):
        try:
            resp = client.call(_request_fill).decode()
            ok = resp.Status() == SVC_SUCCESS
            msg = resp.Message().decode() if resp.Message() else ""
            if ok:
                log.info("Service `%s` successful: %s", service_name, msg)
            else:
                log.warning("Service `%s` failed: %s", service_name, msg)
        except aorta.TimeoutError:
            log.warning("Service `%s` call timed out.", service_name)
        except Exception as e:  # noqa: BLE001
            log.error("Service `%s` call failed: %s", service_name, e)

    def close(self):
        log.info("Stopping keyboard listener thread...")
        if stop_listening is not None:
            stop_listening()
        if self._listener_thread.is_alive():
            self._listener_thread.join(timeout=1.0)


def _parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aorta keyboard takeover trigger")
    p.add_argument("--node-name", default="multi_keyboard_trigger_node")
    p.add_argument(
        "--config",
        required=True,
        help="JSON file: {key: [aorta_service_name, ...]}",
    )
    return p.parse_args(argv)


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO)
    args = _parse_args(argv)
    node = aorta.Node(args.node_name)
    trigger = KeyboardTriggerNode(node, args.config)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        stop.wait()
    finally:
        trigger.close()
        node.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
