#!/usr/bin/env bash
set -euo pipefail

# Start MoveIt demo + MoveIt Servo + pose_to_servo only (no websocket)
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   colcon build --packages-select so_arm_moveit_config so_arm_motion_interface
#   source install/setup.bash
#   ./scripts/run_servo_only.sh
#
# Env overrides:
#   REF_FRAME=base
#   EE_FRAME=gripper
#   POSE_TOPIC=/so_arm/pose_cmd
#   AUTO_START=true

REF_FRAME=${REF_FRAME:-base}
EE_FRAME=${EE_FRAME:-gripper}
POSE_TOPIC=${POSE_TOPIC:-/so_arm/pose_cmd}
AUTO_START=${AUTO_START:-true}

ros2 launch so_arm_motion_interface servo_teleop.launch.py \
  reference_frame:=$REF_FRAME \
  end_effector_frame:=$EE_FRAME \
  pose_topic:=$POSE_TOPIC \
  auto_start_servo:=$AUTO_START
