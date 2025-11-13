## 요구 사항

- Ubuntu 22.04 + ROS 2 Humble
- Isaac Sim(USD 씬 로드 후 Play)
- ros2_control 통신 패키지
  ```bash
  sudo apt install ros-humble-topic-based-ros2-control
  ```

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select so_arm_moveit_config so_arm_motion_interface
source install/setup.bash
```

## 실행(권장)

```bash
# Isaac Sim에서 Play 후 실행
WS_URL="ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle" \
  SO-ARM101_MoveIt_IsaacSim/scripts/run_interface_teleop.sh
```

- 프레임: reference_frame=`base`, end_effector_frame=`gripper`
- Servo 명령 프레임: `base` (TwistStamped `/servo_node/delta_twist_cmds`)
- 인터페이스는 위치만 추종(각속도 0)

## WebSocket 입력 형식(단일)

브리지는 아래 형식만 수용합니다.

```json
{"type":"type_pose",
 "position":{"x":0.0,"y":0.0,"z":0.0},
 "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}
```

기본 변환은 `transform_quaternion`으로 웹→base에 동일 적용(위치/자세).

## 유용한 확인

- 상태: `ros2 topic echo /servo_node/status --qos-reliability best_effort`
- 명령: `ros2 topic echo -n 3 /servo_node/delta_twist_cmds`
- 목표: `ros2 topic echo -n 3 /so_arm/pose_cmd`
- 재시작: `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'`

## 참고

- Servo 구성은 `so_arm_moveit_config/config/moveit_servo.yaml`에서 관리합니다.
- 불필요한 브리지/인터페이스 파라미터는 제거되어, 기본값으로 운영 가능합니다.
