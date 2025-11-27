#!/usr/bin/env bash
set -euo pipefail

# IK-based teleop runner (MoveIt demo + Web Pose bridge + Pose→JointTrajectory)
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   colcon build --packages-select so_arm_moveit_config so_arm_motion_interface
#   source install/setup.bash
#   ./scripts/run_ik_trajectory_teleop.sh
#
# Env overrides:
#   WS_URL=ws://host:port/path?query   # if empty, uses default left_arm track
#   REF_FRAME=base                     # reference frame for IK
#   EE_FRAME=gripper                   # end-effector link name
#   POSE_TOPIC=/so_arm/pose_cmd        # pose topic from websocket bridge

REF_FRAME=${REF_FRAME:-base}
EE_FRAME=${EE_FRAME:-gripper}
POSE_TOPIC=${POSE_TOPIC:-/so_arm/pose_cmd}
WS_URL=${WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle}

echo "[ik-traj] Launching MoveIt demo (controllers + planning scene)"
ros2 launch so_arm_moveit_config demo.launch.py >/tmp/so_arm_moveit_demo.log 2>&1 &
DEMO_PID=$!
echo "[ik-traj] demo.launch.py PID=$DEMO_PID (logs: /tmp/so_arm_moveit_demo.log)"

cleanup() {
  echo "[ik-traj] Shutting down..."
  if kill -0 "$DEMO_PID" 2>/dev/null; then kill "$DEMO_PID" || true; fi
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then kill "$BRIDGE_PID" || true; fi
  if [[ -n "${IK_PID:-}" ]] && kill -0 "$IK_PID" 2>/dev/null; then kill "$IK_PID" || true; fi
}
trap cleanup EXIT INT TERM

echo "[ik-traj] Starting WebSocket pose bridge → $POSE_TOPIC"
WS_ARGS=()
if [[ -n "$WS_URL" ]]; then
  WS_ARGS+=("-p" "websocket_url:=$WS_URL")
fi

ros2 run so_arm_motion_interface websocket_pose_bridge_node \
  --ros-args \
    -p publish_topic:=$POSE_TOPIC \
    -p reference_frame:=$REF_FRAME \
    -p end_effector_frame:=$EE_FRAME \
    "${WS_ARGS[@]}" \
  >/tmp/so_arm_ws_bridge_ik_traj.log 2>&1 &
BRIDGE_PID=$!
echo "[ik-traj] websocket_pose_bridge_node PID=$BRIDGE_PID (logs: /tmp/so_arm_ws_bridge_ik_traj.log)"

echo "[ik-traj] Starting Pose→JointTrajectory IK interface"
ros2 run so_arm_motion_interface pose_to_joint_trajectory_node \
  --ros-args \
    -p input_topic:=$POSE_TOPIC \
    -p joint_trajectory_topic:=/arm_controller/joint_trajectory \
    -p group_name:=arm \
    -p ik_link_name:=$EE_FRAME \
  >/tmp/so_arm_pose_to_joint_trajectory.log 2>&1 &
IK_PID=$!
echo "[ik-traj] pose_to_joint_trajectory_node PID=$IK_PID (logs: /tmp/so_arm_pose_to_joint_trajectory.log)"

echo "[ik-traj] Ready. Move the web marker; IK will follow."

wait "$DEMO_PID"

