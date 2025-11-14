## 준비 사항

- ROS 2 Humble + Isaac Sim 환경
- `ros-humble-topic-based-ros2-control` 등 ros2_control 패키지가 설치되어 있어야 Isaac Sim과 통신됩니다.
  ```bash
  sudo apt install ros-humble-topic-based-ros2-control
  ```
- (선택) `conda activate endpoint` 등 프로젝트별 Python 환경을 활성화한 뒤, 항상
  ```bash
  source /opt/ros/humble/setup.bash
  ```
  를 먼저 실행합니다.

## 빌드 및 설정

1. 리포지토리 루트에서 MoveIt 설정과 인터페이스 패키지를 빌드합니다.
   ```bash
   colcon build --packages-select so_arm_moveit_config so_arm_motion_interface
   source install/setup.bash
   ```
   MoveIt Servo 파라미터(`config/moveit_servo.yaml`)나 노드 기본값을 수정한 뒤에는 반드시 위 명령으로 다시 빌드하세요.
2. `pose_to_servo_node` 기본 게인과 속도 제한은 로컬 테스트 기준으로 크게 상향되어 있습니다  
   (선형 게인 10, 각속도 게인 6, 선형 속도 1.2 m/s, 각속도 3 rad/s). 필요 시 런치에서 더 낮은 값으로 재정의할 수 있습니다.

## 전체 실행 절차

1. **Isaac Sim 준비**  
   SO-ARM USD 씬을 열고 `Play`를 눌러 `/isaac_joint_states`·`/isaac_joint_command`가 갱신되도록 합니다.
2. **MoveIt Servo 런치**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/jaehooni/endpoint_control/SO-ARM101_MoveIt_IsaacSim/install/setup.bash
   ros2 launch so_arm_motion_interface servo_teleop.launch.py \
     reference_frame:=base end_effector_frame:=gripper
   ```
   - MoveIt 데모, MoveIt Servo, Twist/Pose 브리지 노드가 함께 올라옵니다.
   - 2초 뒤 자동으로 `/servo_node/start_servo`를 호출합니다. 자동 시작을 끄고 싶으면 `auto_start_servo:=false`를 전달한 뒤, 필요할 때 직접 서비스를 호출하세요.
3. **(옵션) Servo 재시작**  
   특이점에 의해 Servo가 `HALT` 되거나 자동 시작을 끈 경우:
   ```bash
   ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'
   ```
4. **외부 포즈 스트림 연결**  
   ```bash
   ros2 run so_arm_motion_interface websocket_pose_bridge_node \
     --ros-args \
       -p websocket_url:=ws://cobot.center:8286/pang/ws/pub?channel=instant\&name=so101\&track=left_arm\&mode=bundle \
       -p reference_frame:=base \
       -p end_effector_frame:=gripper
   ```
   - 노드는 TF에서 `base→gripper`를 조회해 초기 자세를 자동 동기화합니다.
   - 웹에서 들어온 포즈는 `/so_arm/pose_cmd`로 퍼블리시되고, 큐(최대 10개)에 저장된 최신 명령만 순서대로 처리합니다.
   - 기본적으로 0.2 s마다 로봇 현재 자세를 동일한 JSON 형식으로 웹에 회신합니다. `status_publish_period`로 조정하거나, `ack_enabled:=false`로 답장을 끌 수 있습니다.

## 로컬 HTTP 테스트

웹 UI 없이 빠르게 점검하려면 HTTP 브리지를 사용할 수 있습니다.

```bash
ros2 run so_arm_motion_interface local_pose_bridge_node
curl -X POST http://127.0.0.1:5005/pose \
  -H 'Content-Type: application/json' \
  -d '{"position":{"x":0.32,"y":0.02,"z":0.18},
       "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}'
```

파라미터 `default_frame_id`, `publish_topic`, `server_port`로 프레임과 토픽을 원하는 구성에 맞게 바꿀 수 있습니다.

## 런타임 팁

- `/servo_node/status --qos-reliability best_effort` 값이 `1`이면 Servo가 RUNNING 상태입니다. 특이점 경고 이후 정지하면 위 서비스를 다시 호출하세요.
- `/so_arm/pose_cmd`, `/so_arm/cmd_vel`, `/servo_node/delta_twist_cmds`, `/arm_controller/joint_trajectory`를 순서대로 모니터링하면 데이터 흐름을 추적할 수 있습니다.
- WebSocket 브리지는 메시지를 큐에 최대 10개까지 유지하고 최신 명령부터 처리합니다. 급격한 명령으로 로봇이 따라오지 못하면, 웹 입력 필터링이나 게인 조정을 통해 안정성을 확보하세요.
