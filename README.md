# SO-ARM Teleop (Raspberry Pi runtime)

This repo is stripped down to run the hardware teleop stack on a Pi:
- HW bridge (Feetech)  
- Web pose/joint bridges  
- Custom direct controller (Pose → JointTrajectory/JointState)  
- Camera publish (v4l2) + WebSocket image bridge  
- robot_state_publisher (TF)

## Runtime dependencies (Pi)
- ROS 2 Humble (desktop or base)  
- `python3-catkin-pkg` (for `colcon build`)  
- GStreamer H.264 plugins: `gstreamer1.0-plugins-base`, `gstreamer1.0-plugins-good`, `gstreamer1.0-plugins-bad`, `gstreamer1.0-plugins-ugly`  
- Video: `ros-humble-v4l2-camera`  
- (추천) 사용자 `video` 그룹 추가: `sudo usermod -aG video $USER` 후 재로그인

## Build
```bash
cd ~/jaehooni/endpoint_control/SO-ARM_Teleop
rm -rf build install log   # 필요 시
source /opt/ros/humble/setup.bash
colcon build
```

## Run (hardware teleop + camera + WS)
```bash
cd ~/jaehooni/endpoint_control/SO-ARM_Teleop
./scripts/run_custom_teleop_real.sh
```
기본값:
- Pose WS: `ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle`
- Joint WS: `ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=robot_joint_states&mode=bundle`
- Image WS: `ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=head_camera&mode=single`
- Camera: `/dev/video0`, 640x480@30, YUYV → rgb8, topic `/camera/image_raw`
- TF: `so101_new_calib.urdf`

환경변수로 덮어쓰기:
```
SO101_PORT=/dev/ttyACM0 CAMERA_DEVICE=/dev/video0 IMAGE_CODEC=jpeg IMAGE_ENABLE=true ./scripts/run_custom_teleop_real.sh
```

## Insta360 (MJPG only) on Jetson
Insta360 UVC는 MJPG만 제공하는 경우가 많아서 `gscam`으로 디코딩 후 ROS 토픽으로 퍼블리시합니다.

필수 패키지:
```bash
sudo apt update
sudo apt install -y ros-humble-gscam \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

카메라 → ROS 토픽:
```bash
ros2 run gscam gscam_node --ros-args \
  -p gscam_config:="v4l2src device=/dev/video0 io-mode=2 ! image/jpeg,framerate=30/1,width=2880,height=1440 ! jpegparse ! jpegdec ! videoconvert ! video/x-raw,format=RGB" \
  -p sync_sink:=false \
  -p image_encoding:=rgb8 \
  -r camera/image_raw:=/insta360/image_raw
```

토픽 확인:
```bash
ros2 topic info /insta360/image_raw
ros2 topic echo /insta360/image_raw --once --field encoding --field width --field height
```

WebSocket 전송 (track=insta360):
```bash
ros2 run so_arm_motion_interface websocket_image_bridge_node --ros-args \
  -p image_topic:=/insta360/image_raw \
  -p websocket_url:="ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=insta360&mode=single" \
  -p publish_period:=0.0333 \
  -p max_width:=2880 -p max_height:=1440 \
  -p bitrate:=2000000 \
  -p h264_codec_string:=avc1.42E03C \
  -p ws_ping_interval:=0.0 -p ws_ping_timeout:=0.0
```

## Topics/Nodes (when running)
- Command/state: `/so_arm/pose_cmd`, `/so_arm/hw_joint_command`, `/so_arm/hw_joint_states`
- Camera: `/camera/image_raw`
- Nodes: `so101_lerobot_bridge_node`, `websocket_pose_bridge`, `websocket_joint_state_bridge`, `custom_direct_controller_node`, `v4l2_camera_node`, `websocket_image_bridge_node`, `robot_state_publisher`

### Topic 상세
- `/so_arm/pose_cmd` (PoseStamped): 웹에서 오는 EE 목표 pose. 헤더 frame_id=`base`(기본), 포지션/쿼터니언 전달.
- `/so_arm/hw_joint_command` (JointState): 커스텀 컨트롤러가 HW 브리지로 보내는 관절 명령. name은 `Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw`.
- `/so_arm/hw_joint_states` (JointState): HW 브리지 → ROS. 위 name 순서로 현재 관절 각도/속도.
- `/so_arm/jaw_command` (Float64): 웹에서 들어오는 Jaw **증분**(rad). `custom_direct_controller_node`가 최신 Jaw 상태에 누적해서 `/so_arm/hw_joint_command`로 보냄.
- `/camera/image_raw` (sensor_msgs/Image): v4l2_camera 출력. 기본 YUYV→rgb8 640x480@30.
- WebSocket 트랙:
  - `left_arm` → `/so_arm/pose_cmd`
  - `robot_joint_states` → `/so_arm/hw_joint_states` (RPi→웹), + 선택적으로 Jaw **증분** 명령(Web→RPi, `Jaw` 값만 사용)
  - `head_camera` → `/camera/image_raw` (H.264/JPEG)

## Camera 문제 시 팁
- 프레임 확인: `ros2 topic hz /camera/image_raw`
- MJPG 크래시 시 YUYV 유지 (기본값). 느리면 `IMAGE_CODEC=jpeg`로 전송.
