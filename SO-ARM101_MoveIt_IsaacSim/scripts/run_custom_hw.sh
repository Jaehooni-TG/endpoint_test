#!/usr/bin/env bash
set -euo pipefail

# One-shot launcher for hardware teleop using the custom direct controller.
# Assumes:
#   - HW ros2_control 드라이버 + arm_controller 가 이미 올라와 있고,
#   - joint_states 가 퍼블리시되고 있으며,
#   - pose_cmd 를 웹/VR에서 받을 수 있는 브리지가 필요하다면 아래 웹소켓 URL을 맞춰준다.
#
# 필요한 경우 아래 변수만 수정해서 사용하세요.

WS_POSE_URL="${WS_POSE_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle}"
WS_JOINT_URL="${WS_JOINT_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=robot_joint_states&mode=bundle}"
POSE_TOPIC="${POSE_TOPIC:-/so_arm/pose_cmd}"
REF_FRAME="${REF_FRAME:-base}"
EE_FRAME="${EE_FRAME:-gripper}"
TRAJ_TOPIC="${TRAJ_TOPIC:-/arm_controller/joint_trajectory}"
JOINT_STATE_TOPIC="${JOINT_STATE_TOPIC:-/joint_states}"
# 웹으로 보낼 관절 상태 소스(HW 드라이버가 퍼블리시하는 토픽, 기본 /joint_states)
WS_JOINT_SOURCE="${WS_JOINT_SOURCE:-/joint_states}"

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
# setup.bash references COLCON_TRACE / AMENT_TRACE_SETUP_FILES; ensure they're defined for set -u
set +u  # allow setup files to reference unset tracing vars
export COLCON_TRACE="${COLCON_TRACE:-}"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-python3}"
source "$root/install/setup.bash"
set -u

pids=()
cleanup() {
  for p in "${pids[@]:-}"; do
    kill "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "[run_custom_hw] pose bridge 시작: $WS_POSE_URL -> $POSE_TOPIC"
ros2 run so_arm_motion_interface websocket_pose_bridge_node \
  --ros-args \
  -p websocket_url:="$WS_POSE_URL" \
  -p publish_topic:="$POSE_TOPIC" \
  -p reference_frame:="$REF_FRAME" \
  -p end_effector_frame:="$EE_FRAME" \
  -p enforce_position_limits:=false \
  >/tmp/so_arm_ws_pose.log 2>&1 &
pids+=($!)

echo "[run_custom_hw] joint state bridge 시작: $WS_JOINT_URL"
ros2 run so_arm_motion_interface websocket_joint_state_bridge_node \
  --ros-args \
  -p websocket_url:="$WS_JOINT_URL" \
  -p publish_period:=0.1 \
  -p joint_state_topic:="$WS_JOINT_SOURCE" \
  >/tmp/so_arm_ws_joint.log 2>&1 &
pids+=($!)

echo "[run_custom_hw] direct controller 시작"
ros2 run so_arm_custom_control lecabot_direct_controller_node \
  --ros-args \
  -p input_pose_topic:="$POSE_TOPIC" \
  -p trajectory_topic:="$TRAJ_TOPIC" \
  -p joint_state_topic:="$JOINT_STATE_TOPIC" \
  -p reference_frame:="$REF_FRAME" \
  -p end_effector_frame:="$EE_FRAME" \
  -p direction_step_m:=0.03 -p direction_deadzone_m:=0.005 \
  -p max_joint_step_deg:=5.0 -p joint_deadband_deg:=0.02 \
  -p time_horizon:=0.18 \
  -p forward_pitch_gain_deg_per_m:=600.0 -p forward_elbow_gain_deg_per_m:=-360.0 \
  -p up_pitch_gain_deg_per_m:=300.0 -p up_elbow_gain_deg_per_m:=-240.0 \
  -p wrist_pitch_bias_deg:=-10.0 -p wrist_pitch_limit_deg:=95.0 \
  >/tmp/so_arm_custom.log 2>&1 &
pids+=($!)

echo "[run_custom_hw] 모든 노드가 백그라운드로 실행 중입니다."
echo "  - pose log   : /tmp/so_arm_ws_pose.log"
echo "  - joint log  : /tmp/so_arm_ws_joint.log"
echo "  - ctrl log   : /tmp/so_arm_custom.log"
echo "중단하려면 Ctrl+C"
wait
