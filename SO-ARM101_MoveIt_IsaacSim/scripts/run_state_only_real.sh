#!/usr/bin/env bash
set -euo pipefail

# Real-hardware state-only runner (no Servo / no controllers commanding joints)
#
# 목적:
#   - 실제 SO-ARM101 하드웨어 상태를 ROS2 (/joint_states, TF)로만 읽어와서
#   - WebSocket joint_state 브리지를 통해 웹에 스트리밍
#   - MoveIt Servo, ros2_control controller, Isaac Sim 등은 전혀 사용하지 않음
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   colcon build --packages-select so_arm_moveit_config so_arm_motion_interface
#   source install/setup.bash
#   ./scripts/run_state_only_real.sh
#
# Env overrides:
#   SO101_PORT=/dev/ttyACM0
#   SO101_ROBOT_ID=go2_so101_follower_arm
#   SO101_CALIB_DIR=<abs path> (기본: repo/calibration/so101_follower)
#   JOINT_WS_URL=ws://... (기본: robot_joint_states track)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SO101_PORT=${SO101_PORT:-/dev/ttyACM0}
SO101_ROBOT_ID=${SO101_ROBOT_ID:-go2_so101_follower_arm}
SO101_CALIB_DIR=${SO101_CALIB_DIR:-${ROOT_DIR}/calibration/so101_follower}

# 기본 좌표계 / 토픽 설정
REF_FRAME=${REF_FRAME:-base}
EE_FRAME=${EE_FRAME:-gripper}
# state-only 모드에서는 pose 명령을 실제 Servo로 보내지 않기 위해
# 전용 토픽 이름을 사용한다.
POSE_TOPIC=${POSE_TOPIC:-/so_arm/pose_cmd_state_only}

# WebSocket URL들 (웹의 기존 트랙과 동일하게)
WS_URL=${WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle}
JOINT_WS_URL=${JOINT_WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=robot_joint_states&mode=bundle}

HW_BRIDGE_PID=""
RSP_PID=""
JOINT_BRIDGE_PID=""
POSE_BRIDGE_PID=""

start_hw_bridge() {
  echo "[state_only] Starting SO101 hardware bridge (state only)..."
  echo "[state_only]  port=${SO101_PORT}, robot_id=${SO101_ROBOT_ID}, calib_dir=${SO101_CALIB_DIR}"

  # joint_state_topic 을 /joint_states 로 바로 발행하고,
  # joint_command_topic 은 별도 토픽(/so_arm/hw_joint_command)을 사용한다.
  # 이 모드에서는 어느 노드도 joint_command_topic 에 publish 하지 않으므로
  # 하드웨어 드라이버는 읽기 전용으로만 동작한다.
  # 또한 enable_torque_on_connect=false 로 설정하여 모터 토크를 끄고,
  # 사용자가 손으로 관절을 움직일 수 있게 한다.
  ros2 run so_arm_motion_interface so101_lerobot_bridge_node \
    --ros-args \
      -p port:="${SO101_PORT}" \
      -p robot_id:="${SO101_ROBOT_ID}" \
      -p calibration_dir:="${SO101_CALIB_DIR}" \
      -p enable_torque_on_connect:=false \
      -p joint_state_topic:=/joint_states \
      -p joint_command_topic:=/so_arm/hw_joint_command \
    > /tmp/so101_hw_bridge_state_only.log 2>&1 &
  HW_BRIDGE_PID=$!
  echo "[state_only] so101_lerobot_bridge_node PID=${HW_BRIDGE_PID} (logs: /tmp/so101_hw_bridge_state_only.log)"
}

start_rsp() {
  echo "[state_only] Starting robot_state_publisher (rsp.launch.py)..."
  # rsp.launch.py 는 robot_state_publisher + 필수 TF 만 올린다
  # (컨트롤러 / Servo / RViz 없음).
  ros2 launch so_arm_moveit_config rsp.launch.py \
    > /tmp/so101_rsp_state_only.log 2>&1 &
  RSP_PID=$!
  echo "[state_only] rsp.launch.py PID=${RSP_PID} (logs: /tmp/so101_rsp_state_only.log)"
}

start_joint_ws_bridge() {
  echo "[state_only] Starting WebSocket joint-state bridge → robot_joint_states"
  JOINT_WS_ARGS=()
  if [[ -n "${JOINT_WS_URL}" ]]; then
    JOINT_WS_ARGS+=("-p" "websocket_url:=${JOINT_WS_URL}")
  fi

  ros2 run so_arm_motion_interface websocket_joint_state_bridge_node \
    --ros-args \
      -p publish_period:=0.1 \
      "${JOINT_WS_ARGS[@]}" \
    > /tmp/so_arm_ws_joint_state_only.log 2>&1 &
  JOINT_BRIDGE_PID=$!
  echo "[state_only] websocket_joint_state_bridge_node PID=${JOINT_BRIDGE_PID} (logs: /tmp/so_arm_ws_joint_state_only.log)"
}

start_pose_ws_bridge() {
  echo "[state_only] Starting WebSocket pose bridge (state only) → $POSE_TOPIC"
  WS_ARGS=()
  if [[ -n "${WS_URL}" ]]; then
    WS_ARGS+=("-p" "websocket_url:=${WS_URL}")
  fi

  ros2 run so_arm_motion_interface websocket_pose_bridge_node \
    --ros-args \
      -p publish_topic:=${POSE_TOPIC} \
      -p reference_frame:=${REF_FRAME} \
      -p end_effector_frame:=${EE_FRAME} \
      -p ws_ping_interval:=0.0 \
      -p ws_ping_timeout:=0.0 \
      "${WS_ARGS[@]}" \
    > /tmp/so_arm_ws_pose_state_only.log 2>&1 &
  POSE_BRIDGE_PID=$!
  echo "[state_only] websocket_pose_bridge_node PID=${POSE_BRIDGE_PID} (logs: /tmp/so_arm_ws_pose_state_only.log)"
}

cleanup() {
  echo "[state_only] Shutting down state-only stack..."
  if [[ -n "${POSE_BRIDGE_PID}" ]] && kill -0 "${POSE_BRIDGE_PID}" 2>/dev/null; then
    kill "${POSE_BRIDGE_PID}" || true
  fi
  if [[ -n "${JOINT_BRIDGE_PID}" ]] && kill -0 "${JOINT_BRIDGE_PID}" 2>/dev/null; then
    kill "${JOINT_BRIDGE_PID}" || true
  fi
  if [[ -n "${RSP_PID}" ]] && kill -0 "${RSP_PID}" 2>/dev/null; then
    kill "${RSP_PID}" || true
  fi
  if [[ -n "${HW_BRIDGE_PID}" ]] && kill -0 "${HW_BRIDGE_PID}" 2>/dev/null; then
    kill "${HW_BRIDGE_PID}" || true
  fi
}
trap cleanup EXIT INT TERM

start_hw_bridge
sleep 1
start_rsp
sleep 1
start_joint_ws_bridge
sleep 1
start_pose_ws_bridge

echo "[state_only] Ready. Move the physical robot (또는 다른 노드로 제어)하면"
echo "[state_only]   /joint_states → WebSocket(robot_joint_states) 로 관절 상태가,"
echo "[state_only]   TF(base→gripper) → WebSocket(left_arm track) 로 EE pose 가 스트리밍됩니다."
echo "[state_only] Press Ctrl+C to stop."

# 하드웨어 브리지가 종료될 때까지 대기
wait "${HW_BRIDGE_PID}"
