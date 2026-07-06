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

"""Robot dog (S100) side launch for distributed teleop.

This launches only the slave arm controller on the robot dog.
It subscribes to /robot/left/joint_cmd published by the PC side
via ROS2 DDS over network.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the slave arm controller on the robot dog."""

    # --- Declare Launch Arguments ---
    left_slave_can_port_arg = DeclareLaunchArgument(
        "left_slave_can_port",
        default_value="can_left",
        description="CAN port for the left slave arm on the robot dog.",
    )

    enable_mit_control_mode_arg = DeclareLaunchArgument(
        "enable_mit_control_mode",
        default_value="true",
        description="Whether enable mit control mode or not.",
    )

    # --- Node Definitions ---

    # Slave Arm Controller Node (receives joint_cmd, controls slave arm)
    left_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_left_controller",
        namespace="/robot/left",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("left_slave_can_port"),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_mit_control_mode"
                ),
            }
        ],
        remappings=[
            ("/robot/left/status", "/puppet/status_left"),
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
        ],
    )

    # --- Create the Launch Description ---
    return LaunchDescription(
        [
            left_slave_can_port_arg,
            enable_mit_control_mode_arg,
            left_controller_node,
        ]
    )
