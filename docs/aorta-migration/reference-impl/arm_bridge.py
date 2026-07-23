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

"""Transport-neutral Piper CAN layer (Aorta migration, PR-B draft).

Byte-for-byte the same CAN / ``piper_sdk`` logic as
``robo_orchard_piper_ros2/ros_bridge.py``, but returns plain dataclasses instead
of ROS2 message objects. This removes the ``rclpy`` / ``sensor_msgs`` /
``geometry_msgs`` dependency from the arm driver so it can be published over
Aorta FlatBuffers (or anything else). The Aorta node fills FlatBuffers from
these dataclasses; the CAN conversion constants are unchanged.

Only difference vs ros_bridge.py:
  - return types: JointStateData / EePoseData / PiperStatusData (dataclasses)
  - joint_control() takes a JointStateData (still reads .name / .position)
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import List

from piper_sdk import C_PiperInterface
from scipy.spatial.transform import Rotation as R  # noqa: N817

__all__ = [
    "JOINT_NAMES",
    "JointStateData",
    "EePoseData",
    "PiperStatusData",
    "PiperLossError",
    "create_piper",
    "get_arm_status",
    "get_arm_ctrl_state",
    "get_arm_state",
    "get_arm_ee_pose",
    "joint_control",
    "get_enable_flag",
    "enable_arm_ctrl",
    "switch_piper_ctrl_mode",
    "set_ctrl_method",
]

global_logger = logging.getLogger(__name__)

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


# ── Plain data containers (replace sensor_msgs / geometry_msgs / PiperStatusMsg) ──
@dataclass
class JointStateData:
    """Mirrors sensor_msgs/JointState (the fields the arm actually uses)."""

    name: List[str] = field(default_factory=lambda: list(JOINT_NAMES))
    position: List[float] = field(default_factory=lambda: [0.0] * 7)
    velocity: List[float] = field(default_factory=list)
    effort: List[float] = field(default_factory=list)


@dataclass
class EePoseData:
    """Mirrors geometry_msgs/Pose (position xyz + orientation quaternion xyzw)."""

    px: float = 0.0
    py: float = 0.0
    pz: float = 0.0
    ox: float = 0.0
    oy: float = 0.0
    oz: float = 0.0
    ow: float = 1.0


@dataclass
class PiperStatusData:
    """Mirrors robo_orchard_piper_msg_ros2/PiperStatusMsg."""

    ctrl_mode: int = 0
    arm_status: int = 0
    mode_feedback: int = 0
    teach_status: int = 0
    motion_status: int = 0
    trajectory_num: int = 0
    err_code: int = 0
    joint_1_angle_limit: bool = False
    joint_2_angle_limit: bool = False
    joint_3_angle_limit: bool = False
    joint_4_angle_limit: bool = False
    joint_5_angle_limit: bool = False
    joint_6_angle_limit: bool = False
    communication_status_joint_1: bool = False
    communication_status_joint_2: bool = False
    communication_status_joint_3: bool = False
    communication_status_joint_4: bool = False
    communication_status_joint_5: bool = False
    communication_status_joint_6: bool = False


class PiperLossError(Exception):
    pass


def create_piper(can_port: str) -> C_PiperInterface:
    piper = C_PiperInterface(can_name=can_port)
    piper.ConnectPort()

    # NOTE: refresh piper message, without this stage,
    # you may get error message
    _ = piper.GetArmStatus()
    _ = get_arm_ctrl_state(piper)
    _ = get_arm_ee_pose(piper)
    _ = get_enable_flag(piper)
    _ = get_arm_status(piper)

    return piper


def get_arm_status(piper: C_PiperInterface) -> PiperStatusData:
    status_msg = piper.GetArmStatus()
    s = status_msg.arm_status
    err = s.err_status
    return PiperStatusData(
        ctrl_mode=s.ctrl_mode,
        arm_status=s.arm_status,
        mode_feedback=s.mode_feed,
        teach_status=s.teach_status,
        motion_status=s.motion_status,
        trajectory_num=s.trajectory_num,
        err_code=s.err_code,
        joint_1_angle_limit=err.joint_1_angle_limit,
        joint_2_angle_limit=err.joint_2_angle_limit,
        joint_3_angle_limit=err.joint_3_angle_limit,
        joint_4_angle_limit=err.joint_4_angle_limit,
        joint_5_angle_limit=err.joint_5_angle_limit,
        joint_6_angle_limit=err.joint_6_angle_limit,
        communication_status_joint_1=err.communication_status_joint_1,
        communication_status_joint_2=err.communication_status_joint_2,
        communication_status_joint_3=err.communication_status_joint_3,
        communication_status_joint_4=err.communication_status_joint_4,
        communication_status_joint_5=err.communication_status_joint_5,
        communication_status_joint_6=err.communication_status_joint_6,
    )


def get_arm_ctrl_state(piper: C_PiperInterface) -> JointStateData:
    joint_state_factor = 1.0 / 1000 * 0.017444
    gripper_state_factor = 1.0 / 1000000

    joint_msg = piper.GetArmJointCtrl()
    gripper_msg = piper.GetArmGripperCtrl()

    return JointStateData(
        name=list(JOINT_NAMES),
        position=[
            joint_msg.joint_ctrl.joint_1 * joint_state_factor,
            joint_msg.joint_ctrl.joint_2 * joint_state_factor,
            joint_msg.joint_ctrl.joint_3 * joint_state_factor,
            joint_msg.joint_ctrl.joint_4 * joint_state_factor,
            joint_msg.joint_ctrl.joint_5 * joint_state_factor,
            joint_msg.joint_ctrl.joint_6 * joint_state_factor,
            gripper_msg.gripper_ctrl.grippers_angle * gripper_state_factor,
        ],
        velocity=[0.0] * 7,
        effort=[0.0] * 7,
    )


def get_arm_state(piper: C_PiperInterface) -> JointStateData:
    joint_msg = piper.GetArmJointMsgs()
    spd_info_msg = piper.GetArmHighSpdInfoMsgs()
    gripper_msg = piper.GetArmGripperMsgs()

    # Raw data is in degrees * 1000; /1000 * (pi/180) -> radians.
    joint_0 = (joint_msg.joint_state.joint_1 / 1000) * 0.017444
    joint_1 = (joint_msg.joint_state.joint_2 / 1000) * 0.017444
    joint_2 = (joint_msg.joint_state.joint_3 / 1000) * 0.017444
    joint_3 = (joint_msg.joint_state.joint_4 / 1000) * 0.017444
    joint_4 = (joint_msg.joint_state.joint_5 / 1000) * 0.017444
    joint_5 = (joint_msg.joint_state.joint_6 / 1000) * 0.017444
    joint_6 = gripper_msg.gripper_state.grippers_angle / 1000000

    vel = [getattr(spd_info_msg, f"motor_{i}").motor_speed / 1000 for i in range(1, 7)]
    effort = [getattr(spd_info_msg, f"motor_{i}").effort / 1000 for i in range(1, 7)]
    effort.append(gripper_msg.gripper_state.grippers_effort / 1000)

    return JointStateData(
        name=list(JOINT_NAMES),
        position=[joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
        velocity=vel,       # 6 elements (no gripper velocity), same as ros_bridge
        effort=effort,      # 7 elements
    )


def get_arm_ee_pose(piper: C_PiperInterface) -> EePoseData:
    pose_msg = piper.GetArmEndPoseMsgs()
    roll = math.radians(pose_msg.end_pose.RX_axis / 1000)
    pitch = math.radians(pose_msg.end_pose.RY_axis / 1000)
    yaw = math.radians(pose_msg.end_pose.RZ_axis / 1000)
    q = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()  # [x, y, z, w]
    return EePoseData(
        px=pose_msg.end_pose.X_axis / 1000000,
        py=pose_msg.end_pose.Y_axis / 1000000,
        pz=pose_msg.end_pose.Z_axis / 1000000,
        ox=float(q[0]),
        oy=float(q[1]),
        oz=float(q[2]),
        ow=float(q[3]),
    )


def joint_control(
    piper: C_PiperInterface,
    joint_data: JointStateData,
    has_gripper: bool = True,
    gripper_val_mutiple: float = 1.0,
):
    factor = 57324.840764  # 1000 * 180 / 3.14

    joint_positions = {}
    gripper = 0
    for idx, joint_name in enumerate(joint_data.name):
        joint_positions[joint_name] = round(joint_data.position[idx] * factor)

    if len(joint_data.position) >= 7:
        gripper = round(joint_data.position[6] * 1000 * 1000)
        gripper = gripper * gripper_val_mutiple

    piper.JointCtrl(
        joint_positions.get("joint1", 0),
        joint_positions.get("joint2", 0),
        joint_positions.get("joint3", 0),
        joint_positions.get("joint4", 0),
        joint_positions.get("joint5", 0),
        joint_positions.get("joint6", 0),
    )
    if has_gripper:
        piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)


def get_enable_flag(piper: C_PiperInterface) -> bool:
    msg = piper.GetArmLowSpdInfoMsgs()
    return (
        msg.motor_1.foc_status.driver_enable_status
        and msg.motor_2.foc_status.driver_enable_status
        and msg.motor_3.foc_status.driver_enable_status
        and msg.motor_4.foc_status.driver_enable_status
        and msg.motor_5.foc_status.driver_enable_status
        and msg.motor_6.foc_status.driver_enable_status
    )


def enable_arm_ctrl(piper: C_PiperInterface, timeout: float = 5):
    timeout = 5
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        if get_enable_flag(piper):
            return
        if elapsed_time > timeout:
            break
        time.sleep(1)
    raise TimeoutError


def switch_piper_ctrl_mode(piper: C_PiperInterface, target_mode: int, timeout: float = 5):
    start_time = time.time()
    while True:
        if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:
            return
        elapsed_time = time.time() - start_time
        piper.MotionCtrl_2(target_mode, 0x01, 100, 0x00)
        if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:
            return
        if elapsed_time > timeout:
            break
        time.sleep(1)
    raise TimeoutError


def set_ctrl_method(piper: C_PiperInterface, is_mit: bool = False):
    if is_mit:
        piper.MotionCtrl_2(0x01, 0x01, 100, is_mit_mode=0xAD)
        for idx in range(6):
            piper.JointMitCtrl(idx + 1, 0, 45, 10, 0.8, 0)
    else:
        piper.MotionCtrl_2(0x01, 0x01, 100, is_mit_mode=0x00)
