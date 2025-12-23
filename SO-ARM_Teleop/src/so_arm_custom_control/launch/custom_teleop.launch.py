"""LeCabot-style teleop: MoveIt demo + Web Pose bridge + Direct joint controller.

This launch file bypasses MoveIt Servo and drives the arm_controller
JointTrajectoryController directly from streamed web poses.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

  # We still use the MoveIt demo launch to bring up the robot/ros2_control,
  # but do not start MoveIt Servo.
  demo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(
              get_package_share_directory("so_arm_moveit_config"),
              "launch",
              "demo.launch.py",
          )
      )
  )

  # Launch args
  websocket_url = LaunchConfiguration("websocket_url")
  joint_ws_url = LaunchConfiguration("joint_ws_url")
  pose_topic = LaunchConfiguration("pose_topic")
  reference_frame = LaunchConfiguration("reference_frame")
  ee_frame = LaunchConfiguration("end_effector_frame")

  # Web pose bridge (websocket → PoseStamped)
  pose_bridge = Node(
      package="so_arm_motion_interface",
      executable="websocket_pose_bridge_node",
      parameters=[
          {"websocket_url": websocket_url},
          {"publish_topic": pose_topic},
          {"reference_frame": reference_frame},
          {"end_effector_frame": ee_frame},
          # Disable position limits so the direct controller can see the full
          # incoming workspace without clamping.
          {"enforce_position_limits": False},
      ],
      output="screen",
  )

  # Joint-state WebSocket bridge (JointState → Web)
  joint_state_bridge = Node(
      package="so_arm_motion_interface",
      executable="websocket_joint_state_bridge_node",
      parameters=[
          {"websocket_url": joint_ws_url},
          # 기본값: /joint_states, 주요 6개 조인트만 전송
          {"publish_period": 0.1},
      ],
      output="screen",
  )

  # Direct joint controller (Pose → JointTrajectory)
  direct_ctrl = Node(
      package="so_arm_custom_control",
      executable="custom_direct_controller_node",
      parameters=[
          {"input_pose_topic": pose_topic},
          {"trajectory_topic": "/arm_controller/joint_trajectory"},
          {"reference_frame": reference_frame},
          {"end_effector_frame": ee_frame},
          # --- 하드웨어 1차 테스트용 안전 파라미터 ---
          {"direction_step_m": 0.03, "direction_deadzone_m": 0.005},
          {"max_joint_step_deg": 5.0, "joint_deadband_deg": 0.02},
          {"time_horizon": 0.18},
          # 앞/뒤, 위/아래 gain (HW용으로 약간 완화)
          {"forward_pitch_gain_deg_per_m": 600.0,
           "forward_elbow_gain_deg_per_m": -360.0},
          {"up_pitch_gain_deg_per_m": 300.0,
           "up_elbow_gain_deg_per_m": -240.0},
          # 손목이 아래로 숙이지 않도록 bias/리밋
          {"wrist_pitch_bias_deg": -10.0,
           "wrist_pitch_limit_deg": 95.0},
      ],
      output="screen",
  )

  return LaunchDescription(
      [
          DeclareLaunchArgument(
              "websocket_url",
              default_value=(
                  "ws://cobot.center:8286/pang/ws/pub"
                  "?channel=instant&name=so101&track=left_arm&mode=bundle"
              ),
          ),
          DeclareLaunchArgument(
              "joint_ws_url",
              default_value=(
                  "ws://cobot.center:8286/pang/ws/pub"
                  "?channel=instant&name=so101&track=robot_joint_states&mode=bundle"
              ),
          ),
          DeclareLaunchArgument("pose_topic", default_value="/so_arm/pose_cmd"),
          DeclareLaunchArgument("reference_frame", default_value="base"),
          DeclareLaunchArgument("end_effector_frame", default_value="gripper"),
          demo_launch,
          pose_bridge,
          joint_state_bridge,
          direct_ctrl,
      ]
  )
