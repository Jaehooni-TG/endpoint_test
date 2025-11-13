"""Launch MoveIt + Servo + Web Pose bridge + Custom IK controller."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def _load_yaml(package: str, relative_path: str):
    file_path = os.path.join(get_package_share_directory(package), relative_path)
    with open(file_path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description() -> LaunchDescription:
    moveit_config = MoveItConfigsBuilder(
        'so101_new_calib', package_name='so_arm_moveit_config'
    ).to_moveit_configs()

    servo_parameters = {
        'moveit_servo': _load_yaml('so_arm_moveit_config', 'config/moveit_servo.yaml')
    }

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('so_arm_moveit_config'),
                'launch',
                'demo.launch.py',
            )
        )
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        parameters=[
            servo_parameters,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output='screen',
    )

    # Launch args
    websocket_url = LaunchConfiguration('websocket_url')
    pose_topic = LaunchConfiguration('pose_topic')
    reference_frame = LaunchConfiguration('reference_frame')
    ee_frame = LaunchConfiguration('end_effector_frame')
    auto_start = LaunchConfiguration('auto_start_servo')

    # Web pose bridge (websocket → PoseStamped)
    pose_bridge = Node(
        package='so_arm_motion_interface',
        executable='websocket_pose_bridge_node',
        parameters=[
            {'websocket_url': websocket_url},
            {'publish_topic': pose_topic},
            {'reference_frame': reference_frame},
            {'end_effector_frame': ee_frame},
        ],
        output='screen',
    )

    # Custom IK controller (Pose → JointJog)
    custom_ik = Node(
        package='so_arm_custom_control',
        executable='custom_ik_controller_node',
        parameters=[
            {'input_pose_topic': pose_topic},
            {'reference_frame': reference_frame},
            # Tunables can be overridden by CLI
            {'x_offset': 0.1629},
            {'z_offset': 0.1131},
            {'pan_gain_deg_per_m': 250.0},
            {'velocity_gain': 0.6},
            {'max_velocity_deg_s': 80.0},
        ],
        output='screen',
    )

    start_servo = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'
                ],
                output='screen',
            )
        ],
        condition=IfCondition(auto_start),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('websocket_url', default_value=''),
            DeclareLaunchArgument('pose_topic', default_value='/so_arm/pose_cmd'),
            DeclareLaunchArgument('reference_frame', default_value='base'),
            DeclareLaunchArgument('end_effector_frame', default_value='gripper'),
            DeclareLaunchArgument('auto_start_servo', default_value='true'),
            demo_launch,
            servo_node,
            pose_bridge,
            custom_ik,
            start_servo,
        ]
    )
