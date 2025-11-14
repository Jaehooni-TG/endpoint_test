#!/usr/bin/env bash
set -euo pipefail

# Quick checks to verify Pose -> Servo -> JointTrajectory pipeline without custom_control.
# Prereq: Launch the stack first (e.g., ./scripts/run_interface_teleop.sh or run_servo_only.sh)
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   source SO-ARM101_MoveIt_IsaacSim/install/setup.bash
#   ./SO-ARM101_MoveIt_IsaacSim/scripts/quick_pipeline_check.sh

echo "[check] Listing key topics..."
ros2 topic list | rg -e '^/so_arm/pose_cmd$' -e '^/servo_node/delta_twist_cmds$' -e '^/arm_controller/joint_trajectory$' -e '^/servo_node/status$' || true

echo "[check] Topic info: /arm_controller/joint_trajectory"
ros2 topic info /arm_controller/joint_trajectory || true

echo "[check] Publish a test pose..."
python3 "$(dirname "$0")/publish_test_pose.py" --x 0.30 --y 0.00 --z 0.20 || true

echo "[check] Waiting briefly for Servo twist output..."
if timeout 5 ros2 topic echo -n 1 /servo_node/delta_twist_cmds >/dev/null 2>&1; then
  echo "[ok] Received TwistStamped on /servo_node/delta_twist_cmds"
else
  echo "[warn] No TwistStamped seen on /servo_node/delta_twist_cmds (check pose_to_servo_node and TF)"
fi

echo "[check] Waiting briefly for JointTrajectory output..."
if timeout 5 ros2 topic echo -n 1 /arm_controller/joint_trajectory >/dev/null 2>&1; then
  echo "[ok] Received JointTrajectory on /arm_controller/joint_trajectory"
else
  echo "[warn] No JointTrajectory seen on /arm_controller/joint_trajectory (check Servo status and controllers)"
fi

echo "[hint] Servo status (1 == RUNNING):"
timeout 3 ros2 topic echo -n 1 /servo_node/status --qos-reliability best_effort || true

