"""Launch MoveIt demo alongside MoveIt Servo for teleoperation."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def _load_yaml(package: str, relative_path: str):
    file_path = os.path.join(get_package_share_directory(package), relative_path)
    with open(file_path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description() -> LaunchDescription:
    moveit_config = MoveItConfigsBuilder(
        "so101_new_calib", package_name="so_arm_moveit_config"
    ).to_moveit_configs()

    servo_parameters = {
        "moveit_servo": _load_yaml("so_arm_moveit_config", "config/moveit_servo.yaml")
    }

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so_arm_moveit_config"),
                "launch",
                "demo.launch.py",
            )
        )
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_parameters,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    reference_frame = LaunchConfiguration("reference_frame")
    ee_frame = LaunchConfiguration("end_effector_frame")
    auto_start = LaunchConfiguration("auto_start_servo")
    linear_gain = LaunchConfiguration("linear_gain")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    angular_gain = LaunchConfiguration("angular_gain")
    max_angular_speed = LaunchConfiguration("max_angular_speed")

    pose_bridge = Node(
        package="so_arm_motion_interface",
        executable="pose_to_servo_node",
        parameters=[
            {"input_topic": pose_topic},
            {"servo_output_topic": "/servo_node/delta_twist_cmds"},
            {"reference_frame": reference_frame},
            {"end_effector_frame": ee_frame},
            {"linear_gain": linear_gain},
            {"max_linear_speed": max_linear_speed},
            {"angular_gain": angular_gain},
            {"max_angular_speed": max_angular_speed},
        ],
        output="screen",
    )

    start_servo = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
        condition=IfCondition(auto_start),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/so_arm/cmd_vel"),
            DeclareLaunchArgument("pose_topic", default_value="/so_arm/pose_cmd"),
            DeclareLaunchArgument("reference_frame", default_value="base"),
            DeclareLaunchArgument("end_effector_frame", default_value="gripper"),
            DeclareLaunchArgument("auto_start_servo", default_value="true"),
            DeclareLaunchArgument("linear_gain", default_value="10.0"),
            DeclareLaunchArgument("max_linear_speed", default_value="1.2"),
            DeclareLaunchArgument("angular_gain", default_value="6.0"),
            DeclareLaunchArgument("max_angular_speed", default_value="3.0"),
            demo_launch,
            servo_node,
            pose_bridge,
            start_servo,
        ]
    )
