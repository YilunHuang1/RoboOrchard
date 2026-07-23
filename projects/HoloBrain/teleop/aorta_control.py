# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
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

"""Safe control commands for the desktop Aorta takeover muxer."""

from __future__ import annotations
import argparse
import math
import time

import aorta
import arm_joint_state_schema_meta as joint_schema
import arm_trigger_schema_meta as trigger_schema
from aorta.services.arm.ArmTriggerResponse import ArmTriggerResponse

SERVICES = {
    "takeover": "aorta/robot/left/takeover_muxer/trigger_takeover",
    "release": "aorta/robot/left/takeover_muxer/release_control",
    "stop": "aorta/robot/left/takeover_muxer/stop",
}


def _request_fill(builder, header_offset):
    trigger_schema.ArmTriggerRequestStart(builder)
    trigger_schema.ArmTriggerRequestAddAortaHeader(builder, header_offset)
    return trigger_schema.ArmTriggerRequestEnd(builder)


def _read_positions(node, topic: str, timeout: float) -> list[float]:
    subscriber = node.create_subscriber_pull(
        topic,
        1,
        qos=aorta.QoS.realtime_control(),
    )
    sample = subscriber.recv(timeout=timeout)
    if sample is None:
        raise RuntimeError(f"No sample received from {topic!r}")
    message = joint_schema.ArmJointState.GetRootAs(sample.payload, 0)
    return [message.Position(i) for i in range(message.PositionLength())]


def _check_alignment(node, args) -> None:
    master = _read_positions(node, args.master_topic, args.timeout)
    slave = _read_positions(node, args.slave_topic, args.timeout)
    differences = [
        abs(master_value - slave_value)
        for master_value, slave_value in zip(
            master[:6], slave[:6], strict=True
        )
    ]
    max_difference = math.degrees(max(differences))
    print(
        "Joint differences (deg):",
        [round(math.degrees(value), 1) for value in differences],
    )
    if max_difference > args.max_joint_diff_deg:
        raise RuntimeError(
            f"Takeover blocked: maximum joint difference is "
            f"{max_difference:.1f} deg "
            f"(limit {args.max_joint_diff_deg:.1f} deg)"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=SERVICES)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-joint-diff-deg", type=float, default=5.0)
    parser.add_argument("--master-topic", default="aorta/master/joint_left")
    parser.add_argument("--slave-topic", default="aorta/puppet/joint_left")
    args = parser.parse_args()

    node = aorta.Node(f"desktop_{args.action}_client")
    try:
        if args.action == "takeover":
            _check_alignment(node, args)

        client = node.create_client_typed(
            SERVICES[args.action],
            ArmTriggerResponse.GetRootAs,
            request_schema_meta=trigger_schema,
        )
        deadline = time.monotonic() + args.timeout
        while (
            not client.raw().matching_status() and time.monotonic() < deadline
        ):
            time.sleep(0.1)
        if not client.raw().matching_status():
            service = SERVICES[args.action]
            raise RuntimeError(f"Service is unavailable: {service}")

        response = client.call(_request_fill).decode()
        message = response.Message().decode() if response.Message() else ""
        print(f"{args.action}: status={response.Status()} message={message}")
        return 0 if response.Status() == 0 else 2
    finally:
        node.close()


if __name__ == "__main__":
    raise SystemExit(main())
