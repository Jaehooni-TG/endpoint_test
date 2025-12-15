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

## Topics/Nodes (when running)
- Command/state: `/so_arm/pose_cmd`, `/so_arm/hw_joint_command`, `/so_arm/hw_joint_states`
- Camera: `/camera/image_raw`
- Nodes: `so101_lerobot_bridge_node`, `websocket_pose_bridge`, `websocket_joint_state_bridge`, `lecabot_direct_controller_node`, `v4l2_camera_node`, `websocket_image_bridge_node`, `robot_state_publisher`

### Topic 상세
- `/so_arm/pose_cmd` (PoseStamped): 웹에서 오는 EE 목표 pose. 헤더 frame_id=`base`(기본), 포지션/쿼터니언 전달.
- `/so_arm/hw_joint_command` (JointState): 커스텀 컨트롤러가 HW 브리지로 보내는 관절 명령. name은 `Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw`.
- `/so_arm/hw_joint_states` (JointState): HW 브리지 → ROS. 위 name 순서로 현재 관절 각도/속도.
- `/camera/image_raw` (sensor_msgs/Image): v4l2_camera 출력. 기본 YUYV→rgb8 640x480@30.
- WebSocket 트랙:
  - `left_arm` → `/so_arm/pose_cmd`
  - `robot_joint_states` → `/so_arm/hw_joint_states`
  - `head_camera` → `/camera/image_raw` (H.264/JPEG)

## Camera 문제 시 팁
- 프레임 확인: `ros2 topic hz /camera/image_raw`
- MJPG 크래시 시 YUYV 유지 (기본값). 느리면 `IMAGE_CODEC=jpeg`로 전송.
