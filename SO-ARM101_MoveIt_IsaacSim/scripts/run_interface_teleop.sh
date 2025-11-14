#!/usr/bin/env bash
set -euo pipefail

# Interface-based teleop runner (MoveIt + Servo + Pose bridge)
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   colcon build --packages-select so_arm_moveit_config so_arm_motion_interface
#   source install/setup.bash
#   ./scripts/run_interface_teleop.sh
#
# Env overrides:
#   WS_URL=ws://host:port/path?query   # if empty, skips websocket bridge
#   REF_FRAME=base                     # reference frame for planning/servo
#   EE_FRAME=gripper                   # end-effector frame
#   POSE_TOPIC=/so_arm/pose_cmd        # topic for PoseStamped
#   AUTO_START=true                    # auto call /servo_node/start_servo

REF_FRAME=${REF_FRAME:-base}
EE_FRAME=${EE_FRAME:-gripper}
POSE_TOPIC=${POSE_TOPIC:-/so_arm/pose_cmd}
AUTO_START=${AUTO_START:-true}
# Default WS track should provide pose messages compatible with the bridge parser.
# If your server uses a different channel/track, override WS_URL via env.
WS_URL=${WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle}

echo "[run] Launching MoveIt + Servo + pose_to_servo (frames: $REF_FRAME → $EE_FRAME)"
ros2 launch so_arm_motion_interface servo_teleop.launch.py \
  reference_frame:=$REF_FRAME \
  end_effector_frame:=$EE_FRAME \
  pose_topic:=$POSE_TOPIC \
  auto_start_servo:=$AUTO_START \
  >/tmp/so_arm_servo.log 2>&1 &
SERVO_PID=$!
echo "[run] servo_teleop.launch.py PID=$SERVO_PID (logs: /tmp/so_arm_servo.log)"

cleanup() {
  echo "[run] Shutting down..."
  if kill -0 "$SERVO_PID" 2>/dev/null; then kill "$SERVO_PID" || true; fi
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then kill "$BRIDGE_PID" || true; fi
}
trap cleanup EXIT INT TERM

echo "[run] Starting WebSocket pose bridge → $POSE_TOPIC"
WS_ARGS=()
if [[ -n "$WS_URL" ]]; then
  WS_ARGS+=("-p" "websocket_url:=$WS_URL")
fi

ros2 run so_arm_motion_interface websocket_pose_bridge_node \
  --ros-args \
    -p publish_topic:=$POSE_TOPIC \
    -p reference_frame:=$REF_FRAME \
    -p end_effector_frame:=$EE_FRAME \
    -p position_smoothing_alpha:=0.85 \
    -p orientation_smoothing_alpha:=0.85 \
    -p max_position_step:=0.15 \
    -p ws_ping_interval:=0.0 \
    -p ws_ping_timeout:=0.0 \
    "${WS_ARGS[@]}" \
  >/tmp/so_arm_ws_bridge.log 2>&1 &
BRIDGE_PID=$!
echo "[run] websocket_pose_bridge_node PID=$BRIDGE_PID (logs: /tmp/so_arm_ws_bridge.log)"


echo "[run] Ready. Useful commands:"
echo "  ros2 topic echo /servo_node/status --qos-reliability best_effort"
echo "  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'"

wait "$SERVO_PID"
