#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SO101_PORT=${SO101_PORT:-/dev/ttyACM0}
SO101_ROBOT_ID=${SO101_ROBOT_ID:-go2_so101_follower_arm}
SO101_CALIB_DIR=${SO101_CALIB_DIR:-${ROOT_DIR}/calibration/so101_follower}

WS_POSE_URL="${WS_POSE_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle}"
WS_JOINT_URL="${WS_JOINT_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=robot_joint_states&mode=bundle}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-480}"
CAMERA_FPS="${CAMERA_FPS:-30.0}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/image_raw}"
# MJPG 인식 오류가 있을 수 있어 기본은 YUYV로 둔다.
CAMERA_PIXEL_FORMAT="${CAMERA_PIXEL_FORMAT:-YUYV}"    # YUYV | MJPG 등 (v4l2_camera pixel_format)
# 브리지와 호환성 위해 기본 출력은 rgb8.
CAMERA_ENCODING="${CAMERA_ENCODING:-rgb8}"            # 예: rgb8, yuv422_yuy2
IMAGE_WS_URL="${IMAGE_WS_URL:-ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=head_camera&mode=single}"
IMAGE_CODEC="${IMAGE_CODEC:-h264}"  # h264 | jpeg
IMAGE_PERIOD="${IMAGE_PERIOD:-0.0416667}"  # ≈24 FPS
IMAGE_BITRATE="${IMAGE_BITRATE:-4000000}"
IMAGE_H264_CODEC_STRING="${IMAGE_H264_CODEC_STRING:-avc1.42E03C}"
IMAGE_ENABLE="${IMAGE_ENABLE:-true}"
POSE_TOPIC="${POSE_TOPIC:-/so_arm/pose_cmd}"
REF_FRAME="${REF_FRAME:-base}"
EE_FRAME="${EE_FRAME:-gripper}"
TRAJ_TOPIC="${TRAJ_TOPIC:-/arm_controller/joint_trajectory}"
JOINT_STATE_TOPIC="${JOINT_STATE_TOPIC:-/so_arm/hw_joint_states}"
URDF_PATH="${URDF_PATH:-${ROOT_DIR}/src/so_arm_description/urdf/so101_new_calib.urdf}"

# ROS setup 스크립트가 미정의 변수를 읽을 수 있으니 잠시 nounset 해제
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

HW_BRIDGE_PID=""
RSP_PID=""
CAM_PID=""
IMG_BRIDGE_PID=""
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
  if [[ -n "${CAM_PID}" ]] && kill -0 "${CAM_PID}" 2>/dev/null; then
    kill "${CAM_PID}" || true
  fi
  if [[ -n "${IMG_BRIDGE_PID}" ]] && kill -0 "${IMG_BRIDGE_PID}" 2>/dev/null; then
    kill "${IMG_BRIDGE_PID}" || true
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
  -p enable_jaw_command:=true \
  -p jaw_joint_name:=Jaw \
  -p jaw_command_topic:=/so_arm/jaw_command \
  >/tmp/so_arm_ws_joint.log 2>&1 &
PIDS+=($!)

echo "[run_custom_teleop_real] jaw command bridge 시작 (/so_arm/jaw_command -> /so_arm/hw_joint_command)"
ros2 run so_arm_motion_interface jaw_command_bridge_node \
  --ros-args \
  -p jaw_command_topic:=/so_arm/jaw_command \
  -p jaw_state_topic:=/so_arm/hw_joint_states \
  -p hw_joint_command_topic:=/so_arm/hw_joint_command \
  -p joint_name:=Jaw \
  >/tmp/so_arm_jaw_bridge.log 2>&1 &
PIDS+=($!)

# robot_state_publisher 를 띄워 TF 제공 (joint_states -> TF)
echo "[run_custom_teleop_real] robot_state_publisher 시작 (urdf=$URDF_PATH, joint_states:=$JOINT_STATE_TOPIC)"
ros2 run robot_state_publisher robot_state_publisher "$URDF_PATH" \
  --ros-args --remap /joint_states:="$JOINT_STATE_TOPIC" \
  >/tmp/so_arm_rsp.log 2>&1 &
RSP_PID=$!

# 카메라 퍼블리시 (v4l2) + WebSocket 전송
if [[ "$IMAGE_ENABLE" == "true" ]]; then
  echo "[run_custom_teleop_real] camera node 시작: $CAMERA_DEVICE -> $CAMERA_TOPIC (${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS})"
  ros2 run v4l2_camera v4l2_camera_node \
    --ros-args \
    -p video_device:="$CAMERA_DEVICE" \
    -p image_size:="[$CAMERA_WIDTH,$CAMERA_HEIGHT]" \
    -p frame_rate:="$CAMERA_FPS" \
    -p pixel_format:="$CAMERA_PIXEL_FORMAT" \
    -p output_encoding:="$CAMERA_ENCODING" \
    --remap image_raw:="$CAMERA_TOPIC" \
    --remap camera_info:="/camera_info" \
    >/tmp/so_arm_cam.log 2>&1 &
  CAM_PID=$!

  echo "[run_custom_teleop_real] image bridge 시작: $CAMERA_TOPIC -> $IMAGE_WS_URL (codec=$IMAGE_CODEC)"
  ros2 run so_arm_motion_interface websocket_image_bridge_node \
    --ros-args \
    -p image_topic:="$CAMERA_TOPIC" \
    -p websocket_url:="$IMAGE_WS_URL" \
    -p codec:="$IMAGE_CODEC" \
    -p publish_period:="$IMAGE_PERIOD" \
    -p max_width:="$CAMERA_WIDTH" \
    -p max_height:="$CAMERA_HEIGHT" \
    -p bitrate:="$IMAGE_BITRATE" \
    -p h264_codec_string:="$IMAGE_H264_CODEC_STRING" \
    -p ws_ping_interval:=0.0 -p ws_ping_timeout:=0.0 \
    >/tmp/so_arm_ws_image.log 2>&1 &
  IMG_BRIDGE_PID=$!
fi

echo "[run_custom_teleop_real] direct controller 시작"
ros2 run so_arm_custom_control custom_direct_controller_node \
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
echo "  - rsp log       : /tmp/so_arm_rsp.log"
if [[ "$IMAGE_ENABLE" == "true" ]]; then
  echo "  - cam log       : /tmp/so_arm_cam.log"
  echo "  - image log     : /tmp/so_arm_ws_image.log"
fi
echo "중단하려면 Ctrl+C"
wait
