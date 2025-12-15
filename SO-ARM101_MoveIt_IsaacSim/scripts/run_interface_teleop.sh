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
# Note:
#   This script 자체는 RViz / Isaac Sim 을 직접 띄우지 않는다.
#   RViz 는 so_arm_moveit_config/launch/demo.launch.py 가 담당하고,
#   그 demo.launch.py 는 다시 servo_teleop.launch.py 안에서 IncludeLaunchDescription 으로 불린다.
#   따라서 RViz 를 빼고 headless 로만 사용하고 싶다면
#   run_interface_teleop.sh 가 아니라 servo_teleop.launch.py / demo.launch.py 쪽을 수정하면 된다.
#
# Env overrides:
#   WS_URL=ws://host:port/path?query           # if empty, skips pose websocket bridge
#   REF_FRAME=base                             # reference frame for planning/servo
#   EE_FRAME=gripper                           # end-effector frame
#   POSE_TOPIC=/so_arm/pose_cmd                # topic for PoseStamped
#   AUTO_START=true                            # auto call /servo_node/start_servo
#   READY_POSE=0.0,0.1,0.7,0.5,0.0             # default ready pose (Rotation,Pitch,Elbow,Wrist_Pitch,Wrist_Roll)
#   IMAGE_TOPIC=/so_arm/overhead_camera/image_raw
#   IMAGE_WS_URL=ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=head_camera&mode=single
#   IMAGE_CODEC=h264                           # jpeg | h264
#   IMAGE_PERIOD=0.0416667                     # seconds (≈24 FPS)
#   IMAGE_MAX_WIDTH=1280
#   IMAGE_MAX_HEIGHT=720
#   IMAGE_BITRATE=4000000                      # bits/s
#   IMAGE_H264_CODEC_STRING=avc1.42E03C        # advertised codec string

REF_FRAME=${REF_FRAME:-base}
EE_FRAME=${EE_FRAME:-gripper}
POSE_TOPIC=${POSE_TOPIC:-/so_arm/pose_cmd}
AUTO_START=${AUTO_START:-false}
# Tuning knobs (env-overridable)
# Linear path (conservative, smoother like RViz planning)
# - Lower gain = 덜 공격적인 추종
# - 낮은 max speed = 속도 상한 제한 (더 부드러운 가속/감속)
LIN_GAIN=${LIN_GAIN:-10.0}
MAX_LIN_SPEED=${MAX_LIN_SPEED:-2.0}
# Angular path (orientation도 부드럽게)
ANG_GAIN=${ANG_GAIN:-4.0}
MAX_ANG_SPEED=${MAX_ANG_SPEED:-2.0}
# Bridge smoothing/step (더 많은 필터링 + per-step 제한)
# alpha가 1.0에 가까울수록 \"새 값\"을 많이 따라가고,
# 0.0에 가까울수록 이전 pose를 더 많이 유지한다.
POS_ALPHA=${POS_ALPHA:-0.6}
ORI_ALPHA=${ORI_ALPHA:-0.5}
# 한 번 업데이트에서 최대 이동/회전량 (m, rad)
MAX_POS_STEP=${MAX_POS_STEP:-0.15}
MAX_ORI_STEP=${MAX_ORI_STEP:-0.35}
# Command frame toggle for pose_to_servo (true: EE local frame, false: base frame)
CMD_IN_EE=${CMD_IN_EE:-false}
# Default WS tracks:
#  - Pose: left_arm (incoming EE pose commands)
#  - Joint states: robot_joint_states (outgoing /joint_states snapshot)
# Override via WS_URL / JOINT_WS_URL if needed.
WS_URL=${WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle}
JOINT_WS_URL=${JOINT_WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=robot_joint_states&mode=bundle}
# Image WebSocket bridge defaults (head camera)
IMAGE_TOPIC=${IMAGE_TOPIC:-/so_arm/overhead_camera/image_raw}
IMAGE_WS_URL=${IMAGE_WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=head_camera&mode=single}
IMAGE_CODEC=${IMAGE_CODEC:-h264}
IMAGE_PERIOD=${IMAGE_PERIOD:-0.0416667}
IMAGE_MAX_WIDTH=${IMAGE_MAX_WIDTH:-1280}
IMAGE_MAX_HEIGHT=${IMAGE_MAX_HEIGHT:-720}
IMAGE_BITRATE=${IMAGE_BITRATE:-4000000}
IMAGE_H264_CODEC_STRING=${IMAGE_H264_CODEC_STRING:-avc1.42E03C}
IMAGE_BRIDGE_ENABLE=${IMAGE_BRIDGE_ENABLE:-true}
START_SERVO_AFTER_READY=${START_SERVO_AFTER_READY:-true}

JOINT_RECOVERY_PIDS=""
IMAGE_BRIDGE_PID=""

echo "[run] Launching MoveIt + Servo + pose_to_servo (frames: $REF_FRAME → $EE_FRAME)"
ros2 launch so_arm_motion_interface servo_teleop.launch.py \
  reference_frame:=$REF_FRAME \
  end_effector_frame:=$EE_FRAME \
  pose_topic:=$POSE_TOPIC \
  linear_gain:=$LIN_GAIN \
  max_linear_speed:=$MAX_LIN_SPEED \
  angular_gain:=$ANG_GAIN \
  max_angular_speed:=$MAX_ANG_SPEED \
  command_in_ee:=$CMD_IN_EE \
  auto_start_servo:=$AUTO_START \
  >/tmp/so_arm_servo.log 2>&1 &
SERVO_PID=$!
echo "[run] servo_teleop.launch.py PID=$SERVO_PID (logs: /tmp/so_arm_servo.log)"

echo "[run] Enabling orientation drift on Servo (translations prioritized)"
ros2 run so_arm_motion_interface enable_servo_orientation_drift_node \
  >/tmp/so_arm_servo_drift_setup.log 2>&1 &

start_servo_safe() {
  echo "[run] Waiting for /servo_node/start_servo service..."
  for _ in {1..10}; do
    if ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}" >/tmp/so_arm_servo_start.log 2>&1; then
      echo "[run] Servo started (see /tmp/so_arm_servo_start.log)"
      return
    fi
    sleep 0.5
  done
  echo "[run] Failed to start servo after ready pose (check /tmp/so_arm_servo_start.log)"
}

echo "[run] Waiting 2s for bringup..."
sleep 2

echo "[run] Running initial self-collision check with MoveIt..."
if ! ros2 run so_arm_motion_interface initial_state_collision_guard_node \
  >/tmp/so_arm_initial_collision_check.log 2>&1; then
  echo "[run] Initial pose is in self-collision."
  echo "[run] See /tmp/so_arm_initial_collision_check.log for details (contact pairs, etc.)."
  echo "[run] Move the robot to a collision-free pose and rerun this script."
  exit 1
fi

if [[ "$START_SERVO_AFTER_READY" == "true" ]]; then
  start_servo_safe
fi

cleanup() {
  echo "[run] Shutting down..."
  if kill -0 "$SERVO_PID" 2>/dev/null; then kill "$SERVO_PID" || true; fi
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then kill "$BRIDGE_PID" || true; fi
  if [[ -n "${JOINT_BRIDGE_PID:-}" ]] && kill -0 "$JOINT_BRIDGE_PID" 2>/dev/null; then kill "$JOINT_BRIDGE_PID" || true; fi
  if [[ -n "${IMAGE_BRIDGE_PID:-}" ]] && kill -0 "$IMAGE_BRIDGE_PID" 2>/dev/null; then kill "$IMAGE_BRIDGE_PID" || true; fi
  if [[ -n "${JOINT_RECOVERY_PIDS:-}" ]]; then
    for pid in $JOINT_RECOVERY_PIDS; do
      if kill -0 "$pid" 2>/dev/null; then kill "$pid" || true; fi
    done
  fi
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
    -p position_smoothing_alpha:=$POS_ALPHA \
    -p orientation_smoothing_alpha:=$ORI_ALPHA \
    -p max_position_step:=$MAX_POS_STEP \
    -p max_orientation_step:=$MAX_ORI_STEP \
    -p ws_ping_interval:=0.0 \
    -p ws_ping_timeout:=0.0 \
    "${WS_ARGS[@]}" \
  >/tmp/so_arm_ws_bridge.log 2>&1 &
BRIDGE_PID=$!
echo "[run] websocket_pose_bridge_node PID=$BRIDGE_PID (logs: /tmp/so_arm_ws_bridge.log)"

echo "[run] Starting WebSocket joint-state bridge → robot_joint_states"
JOINT_WS_ARGS=()
if [[ -n "$JOINT_WS_URL" ]]; then
  JOINT_WS_ARGS+=("-p" "websocket_url:=$JOINT_WS_URL")
fi

ros2 run so_arm_motion_interface websocket_joint_state_bridge_node \
  --ros-args \
    -p publish_period:=0.1 \
    "${JOINT_WS_ARGS[@]}" \
  >/tmp/so_arm_ws_joint.log 2>&1 &
JOINT_BRIDGE_PID=$!
echo "[run] websocket_joint_state_bridge_node PID=$JOINT_BRIDGE_PID (logs: /tmp/so_arm_ws_joint.log)"

if [[ "$IMAGE_BRIDGE_ENABLE" == "true" ]]; then
  echo "[run] Starting WebSocket image bridge → $IMAGE_TOPIC"
  IMAGE_WS_ARGS=()
  if [[ -n "$IMAGE_WS_URL" ]]; then
    IMAGE_WS_ARGS+=("-p" "websocket_url:=$IMAGE_WS_URL")
  fi

  ros2 run so_arm_motion_interface websocket_image_bridge_node \
    --ros-args \
      -p image_topic:=$IMAGE_TOPIC \
      -p codec:=$IMAGE_CODEC \
      -p publish_period:=$IMAGE_PERIOD \
      -p max_width:=$IMAGE_MAX_WIDTH \
      -p max_height:=$IMAGE_MAX_HEIGHT \
      -p bitrate:=$IMAGE_BITRATE \
      -p h264_codec_string:=$IMAGE_H264_CODEC_STRING \
      -p ws_ping_interval:=0.0 \
      -p ws_ping_timeout:=0.0 \
      "${IMAGE_WS_ARGS[@]}" \
    >/tmp/so_arm_ws_image.log 2>&1 &
  IMAGE_BRIDGE_PID=$!
  echo "[run] websocket_image_bridge_node PID=$IMAGE_BRIDGE_PID (logs: /tmp/so_arm_ws_image.log)"
fi

# Joint-limit recovery helpers for multiple joints
# Soft limits are chosen slightly inside the URDF limits to avoid hard hits.
echo "[run] Starting joint-limit recovery helper (Wrist_Pitch soft-limit escape)"
ros2 run so_arm_motion_interface joint_limit_recovery_node \
  --ros-args \
    -p joint_name:=Wrist_Pitch \
    -p upper_soft_limit:=1.62 \
    -p lower_soft_limit:=-1.62 \
    -p trigger_status_codes:="[5]" \
  >/tmp/so_arm_joint_recovery_Wrist_Pitch.log 2>&1 &
pid=$!
JOINT_RECOVERY_PIDS+=" $pid"
echo "[run] joint_limit_recovery_node[Wrist_Pitch] PID=$pid (logs: /tmp/so_arm_joint_recovery_Wrist_Pitch.log)"

echo "[run] Starting joint-limit recovery helper (Elbow soft-limit escape)"
ros2 run so_arm_motion_interface joint_limit_recovery_node \
  --ros-args \
    -p joint_name:=Elbow \
    -p upper_soft_limit:=1.53 \
    -p lower_soft_limit:=-1.70 \
    -p trigger_status_codes:="[5]" \
  >/tmp/so_arm_joint_recovery_Elbow.log 2>&1 &
pid=$!
JOINT_RECOVERY_PIDS+=" $pid"
echo "[run] joint_limit_recovery_node[Elbow] PID=$pid (logs: /tmp/so_arm_joint_recovery_Elbow.log)"

echo "[run] Starting joint-limit recovery helper (Pitch soft-limit escape)"
ros2 run so_arm_motion_interface joint_limit_recovery_node \
  --ros-args \
    -p joint_name:=Pitch \
    -p upper_soft_limit:=1.70 \
    -p lower_soft_limit:=-1.70 \
    -p trigger_status_codes:="[5]" \
  >/tmp/so_arm_joint_recovery_Pitch.log 2>&1 &
pid=$!
JOINT_RECOVERY_PIDS+=" $pid"
echo "[run] joint_limit_recovery_node[Pitch] PID=$pid (logs: /tmp/so_arm_joint_recovery_Pitch.log)"

echo "[run] Starting joint-limit recovery helper (Rotation soft-limit escape)"
ros2 run so_arm_motion_interface joint_limit_recovery_node \
  --ros-args \
    -p joint_name:=Rotation \
    -p upper_soft_limit:=1.88 \
    -p lower_soft_limit:=-1.88 \
    -p trigger_status_codes:="[5]" \
  >/tmp/so_arm_joint_recovery_Rotation.log 2>&1 &
pid=$!
JOINT_RECOVERY_PIDS+=" $pid"
echo "[run] joint_limit_recovery_node[Rotation] PID=$pid (logs: /tmp/so_arm_joint_recovery_Rotation.log)"

echo "[run] Starting joint-limit recovery helper (Wrist_Roll soft-limit escape)"
ros2 run so_arm_motion_interface joint_limit_recovery_node \
  --ros-args \
    -p joint_name:=Wrist_Roll \
    -p upper_soft_limit:=2.75 \
    -p lower_soft_limit:=-2.75 \
    -p trigger_status_codes:="[5]" \
  >/tmp/so_arm_joint_recovery_Wrist_Roll.log 2>&1 &
pid=$!
JOINT_RECOVERY_PIDS+=" $pid"
echo "[run] joint_limit_recovery_node[Wrist_Roll] PID=$pid (logs: /tmp/so_arm_joint_recovery_Wrist_Roll.log)"

echo "[run] Starting joint-limit recovery helper (Jaw soft-limit escape)"
ros2 run so_arm_motion_interface joint_limit_recovery_node \
  --ros-args \
    -p joint_name:=Jaw \
    -p upper_soft_limit:=1.70 \
    -p lower_soft_limit:=-0.13 \
    -p trigger_status_codes:="[5]" \
  >/tmp/so_arm_joint_recovery_Jaw.log 2>&1 &
pid=$!
JOINT_RECOVERY_PIDS+=" $pid"
echo "[run] joint_limit_recovery_node[Jaw] PID=$pid (logs: /tmp/so_arm_joint_recovery_Jaw.log)"


echo "[run] Ready. Useful commands:"
echo "  ros2 topic echo /servo_node/status --qos-reliability best_effort"
echo "  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'"

wait "$SERVO_PID"
