#!/usr/bin/env bash
set -euo pipefail

# Real-hardware teleop runner (MoveIt + Servo + Web + SO101 lerobot bridge)
#
# 기존 run_interface_teleop.sh 는 Isaac Sim + RViz 시뮬레이션용으로 그대로 두고,
# 이 스크립트는 실제 SO-ARM101 하드웨어와 연동할 때 사용한다.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   colcon build --packages-select so_arm_moveit_config so_arm_motion_interface
#   source install/setup.bash
#   ./scripts/run_interface_teleop_real.sh
#
# Env overrides:
#   SO101_PORT=/dev/ttyACM0                     # Feetech 컨트롤러가 잡힌 시리얼 포트
#   SO101_ROBOT_ID=go2_so101_follower_arm       # calibration 파일 이름 (xxx.json 의 xxx)
#   SO101_CALIB_DIR=<absolute/path/to/calibdir> # calibration 디렉터리 (기본: repo/calibration/so101_follower)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SO101_PORT=${SO101_PORT:-/dev/ttyACM0}
SO101_ROBOT_ID=${SO101_ROBOT_ID:-go2_so101_follower_arm}
SO101_CALIB_DIR=${SO101_CALIB_DIR:-${ROOT_DIR}/calibration/so101_follower}

# Try to locate local lerobot repo and add its src to PYTHONPATH
DEFAULT_LEROBOT_SRC="$(cd "${ROOT_DIR}/.." && pwd)/lerobot/src"
LEROBOT_SRC=${LEROBOT_SRC:-${DEFAULT_LEROBOT_SRC}}
if [ -d "${LEROBOT_SRC}" ]; then
  export PYTHONPATH="${LEROBOT_SRC}:${PYTHONPATH:-}"
  echo "[run_real] Using lerobot from ${LEROBOT_SRC} (PYTHONPATH updated)"
else
  echo "[run_real] WARNING: lerobot src not found at ${LEROBOT_SRC}."
  echo "[run_real]          Please set LEROBOT_SRC to your lerobot/src directory if import fails."
fi

HW_BRIDGE_PID=""

start_hw_bridge() {
  echo "[run_real] Starting SO101 lerobot hardware bridge..."
  echo "[run_real]  port=${SO101_PORT}, robot_id=${SO101_ROBOT_ID}, calib_dir=${SO101_CALIB_DIR}"

  ros2 run so_arm_motion_interface so101_lerobot_bridge_node \
    --ros-args \
      -p port:="${SO101_PORT}" \
      -p robot_id:="${SO101_ROBOT_ID}" \
      -p calibration_dir:="${SO101_CALIB_DIR}" \
    > /tmp/so101_hw_bridge.log 2>&1 &
  HW_BRIDGE_PID=$!
  echo "[run_real] so101_lerobot_bridge_node PID=${HW_BRIDGE_PID} (logs: /tmp/so101_hw_bridge.log)"
}

cleanup() {
  echo "[run_real] Shutting down real-hardware teleop..."
  if [[ -n "${HW_BRIDGE_PID}" ]] && kill -0 "${HW_BRIDGE_PID}" 2>/dev/null; then
    kill "${HW_BRIDGE_PID}" || true
  fi
  # 추가 안전망: 브리지 강제 종료로 토크가 남았을 때를 대비해 모든 모터 토크를 끈다.
  python3 - <<'PY'
from pathlib import Path
import sys

from so_arm_motion_interface.so_arm_motion_interface.so101_feetech_driver import So101FeetechArm

port = "${SO101_PORT}"
calib_path = Path("${SO101_CALIB_DIR}") / f"{'${SO101_ROBOT_ID}'}.json"
try:
    arm = So101FeetechArm(port, calib_path)
    arm.connect(enable_torque=False)
    arm.disable_all_torque()
    arm.disconnect(disable_torque=False)
    print("[run_real] Forced torque-off sent to all motors.")
except Exception as exc:
    print(f"[run_real] Failed to force torque-off: {exc}", file=sys.stderr)
PY
}
trap cleanup EXIT INT TERM

start_hw_bridge

echo "[run_real] Launching standard teleop stack (run_interface_teleop.sh)..."
"${ROOT_DIR}/scripts/run_interface_teleop.sh"
