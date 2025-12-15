#!/usr/bin/env bash
set -euo pipefail

# Real hardware teleop using custom direct controller (no MoveIt Servo).
# 1) Feetech HW bridge를 띄워 모터/엔코더와 통신
# 2) Web pose/joint 브리지
# 3) Custom controller (Pose -> JointTrajectory/JointState)
#
# 환경변수로 덮어쓸 수 있는 기본값:
#   SO101_PORT=/dev/ttyACM0
#   SO101_ROBOT_ID=go2_so101_follower_arm
#   SO101_CALIB_DIR=<repo>/calibration/so101_follower
#   POSE_TOPIC=/so_arm/pose_cmd
#   REF_FRAME=base
#   EE_FRAME=gripper
#   TRAJ_TOPIC=/so_arm/hw_joint_command          # HW 드라이버 명령 토픽
#   JOINT_STATE_TOPIC=/so_arm/hw_joint_states    # HW 드라이버 상태 토픽
#   WS_POSE_URL=ws://...left_arm...
#   WS_JOINT_URL=ws://...robot_joint_states...

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SO101_PORT=${SO101_PORT:-/dev/ttyACM0}
SO101_ROBOT_ID=${SO101_ROBOT_ID:-go2_so101_follower_arm}
SO101_CALIB_DIR=${SO101_CALIB_DIR:-${ROOT_DIR}/calibration/so101_follower}

WS_POSE_URL="${WS_POSE_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle}"
WS_JOINT_URL="${WS_JOINT_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=robot_joint_states&mode=bundle}"
POSE_TOPIC="${POSE_TOPIC:-/so_arm/pose_cmd}"
REF_FRAME="${REF_FRAME:-base}"
EE_FRAME="${EE_FRAME:-gripper}"
TRAJ_TOPIC="${TRAJ_TOPIC:-/so_arm/hw_joint_command}"
JOINT_STATE_TOPIC="${JOINT_STATE_TOPIC:-/so_arm/hw_joint_states}"
URDF_PATH="${URDF_PATH:-${ROOT_DIR}/src/so_arm_description/urdf/so101_new_calib.urdf}"

# Allow sourcing even with set -u
set +u
export COLCON_TRACE="${COLCON_TRACE:-}"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-python3}"
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

HW_BRIDGE_PID=""
RSP_PID=""
PIDS=()

cleanup() {
  echo "[run_custom_teleop_real] Shutting down..."
  for p in "${PIDS[@]:-}"; do
    kill "$p" 2>/dev/null || true
  done
  if [[ -n "${HW_BRIDGE_PID}" ]] && kill -0 "${HW_BRIDGE_PID}" 2>/dev/null; then
    kill "${HW_BRIDGE_PID}" || true
  fi
  if [[ -n "${RSP_PID}" ]] && kill -0 "${RSP_PID}" 2>/dev/null; then
    kill "${RSP_PID}" || true
  fi
}
trap cleanup EXIT INT TERM

echo "[run_custom_teleop_real] Starting HW bridge (so101_lerobot_bridge_node)"
ros2 run so_arm_motion_interface so101_lerobot_bridge_node \
  --ros-args \
    -p port:="${SO101_PORT}" \
    -p robot_id:="${SO101_ROBOT_ID}" \
    -p calibration_dir:="${SO101_CALIB_DIR}" \
  > /tmp/so101_hw_bridge.log 2>&1 &
HW_BRIDGE_PID=$!
echo "[run_custom_teleop_real] hw bridge PID=$HW_BRIDGE_PID (log: /tmp/so101_hw_bridge.log)"

echo "[run_custom_teleop_real] pose bridge 시작: $WS_POSE_URL -> $POSE_TOPIC"
ros2 run so_arm_motion_interface websocket_pose_bridge_node \
  --ros-args \
  -p websocket_url:="$WS_POSE_URL" \
  -p publish_topic:="$POSE_TOPIC" \
  -p reference_frame:="$REF_FRAME" \
  -p end_effector_frame:="$EE_FRAME" \
  -p enforce_position_limits:=false \
  >/tmp/so_arm_ws_pose.log 2>&1 &
PIDS+=($!)

echo "[run_custom_teleop_real] joint state bridge 시작: $WS_JOINT_URL (source $JOINT_STATE_TOPIC)"
ros2 run so_arm_motion_interface websocket_joint_state_bridge_node \
  --ros-args \
  -p websocket_url:="$WS_JOINT_URL" \
  -p publish_period:=0.1 \
  -p joint_topic:="$JOINT_STATE_TOPIC" \
  >/tmp/so_arm_ws_joint.log 2>&1 &
PIDS+=($!)

# robot_state_publisher 를 띄워 TF 제공 (joint_states -> TF)
echo "[run_custom_teleop_real] robot_state_publisher 시작 (urdf=$URDF_PATH, joint_states:=$JOINT_STATE_TOPIC)"
ros2 run robot_state_publisher robot_state_publisher "$URDF_PATH" \
  --ros-args --remap /joint_states:="$JOINT_STATE_TOPIC" \
  >/tmp/so_arm_rsp.log 2>&1 &
RSP_PID=$!

echo "[run_custom_teleop_real] direct controller 시작"
ros2 run so_arm_custom_control lecabot_direct_controller_node \
  --ros-args \
  -p input_pose_topic:="$POSE_TOPIC" \
  -p trajectory_topic:="$TRAJ_TOPIC" \
  -p joint_state_topic:="$JOINT_STATE_TOPIC" \
  -p reference_frame:="$REF_FRAME" \
  -p end_effector_frame:="$EE_FRAME" \
  -p pan_gain_deg_per_m:=120.0 \
  -p direction_step_m:=0.02 -p direction_deadzone_m:=0.01 \
  -p max_joint_step_deg:=4.0 -p joint_deadband_deg:=0.02 \
  -p time_horizon:=0.20 \
  -p forward_pitch_gain_deg_per_m:=450.0 -p forward_elbow_gain_deg_per_m:=-270.0 \
  -p up_pitch_gain_deg_per_m:=120.0 -p up_elbow_gain_deg_per_m:=-260.0 \
  -p wrist_pitch_bias_deg:=-10.0 -p wrist_pitch_limit_deg:=95.0 \
  -p wrist_follow_scale:=0.6 \
  >/tmp/so_arm_custom.log 2>&1 &
PIDS+=($!)

echo "[run_custom_teleop_real] 모든 노드가 백그라운드로 실행 중입니다."
echo "  - hw bridge log : /tmp/so101_hw_bridge.log"
echo "  - pose log      : /tmp/so_arm_ws_pose.log"
echo "  - joint log     : /tmp/so_arm_ws_joint.log"
echo "  - ctrl log      : /tmp/so_arm_custom.log"
echo "중단하려면 Ctrl+C"
wait
