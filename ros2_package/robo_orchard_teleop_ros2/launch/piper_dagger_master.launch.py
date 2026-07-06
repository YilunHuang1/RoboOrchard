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

"""PC-side (master arm only) launch for distributed teleop.

This launches only the master arm controller and the takeover muxer.
The slave arm controller runs on the robot dog (S100).
Communication between PC and robot dog is via ROS2 DDS over network.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the master arm controller and takeover muxer on PC side."""

    # --- Declare Launch Arguments ---
    left_algo_topic_arg = DeclareLaunchArgument(
        "left_algo_topic",
        default_value="/left_algo_cmd",
        description="Algorithm command topic for the left arm.",
    )
    left_master_can_port_arg = DeclareLaunchArgument(
        "left_master_can_port",
        default_value="can_left_mst",
        description="CAN port for the left master arm.",
    )

    enable_master_mit_control_mode_arg = DeclareLaunchArgument(
        "enable_master_mit_control_mode",
        default_value="true",
        description="Whether enable mit control mode or not.",
    )

    replay_time_s_arg = DeclareLaunchArgument(
        "replay_time_s",
        default_value="0.0",
        description="Replay time (in seconds)",
    )

    # --- Node Definitions ---

    # 1. Takeover Muxer Node
    left_takeover_muxer_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="take_over",
        name="robot_left_takeover_muxer",
        namespace="/robot/left/takeover_muxer",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "message_type": "sensor_msgs/msg/JointState",
                "algo_topic": LaunchConfiguration("left_algo_topic"),
                "override_topic": "/master/joint_left",
                "output_topic": "/robot/left/joint_cmd",
                "override_mode_behavior": "forward",
                "replay_time_s": LaunchConfiguration("replay_time_s"),
            }
        ],
    )

    # 2. Master Arm Controller Node (reads master arm, publishes joint state)
    left_master_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_left_master_controller",
        namespace="/robot/left_master",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("left_master_can_port"),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_master_mit_control_mode"
                ),
            }
        ],
        remappings=[
            ("/robot/left_master/joint_cmd", "/robot/left/joint_cmd"),
            ("/robot/left_master/status", "/master/status_left"),
            ("/robot/left_master/ee_pose", "/master/end_pose_left"),
            ("/robot/left_master/joint_state", "/master/joint_left"),
        ],
    )

    # --- Create the Launch Description ---
    return LaunchDescription(
        [
            left_algo_topic_arg,
            left_master_can_port_arg,
            enable_master_mit_control_mode_arg,
            replay_time_s_arg,
            left_takeover_muxer_node,
            left_master_controller_node,
        ]
    )
