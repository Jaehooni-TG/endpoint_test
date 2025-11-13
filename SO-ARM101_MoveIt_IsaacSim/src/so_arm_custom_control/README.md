# so_arm_custom_control

Custom planar IK controller for SO-ARM101 driven by web pose.

- Lateral Y controls `Rotation` (shoulder pan) only.
- Planar IK on X–Z for `Pitch` (shoulder lift) and `Elbow`.
- `Wrist_Pitch` couples to maintain EE pitch: `-(Pitch + Elbow) + pitch`.
- `Wrist_Roll` follows EE roll.

Inputs and outputs:
- Subscribes: `geometry_msgs/PoseStamped` at `/so_arm/pose_cmd` (from websocket bridge).
- Publishes: `control_msgs/JointJog` at `/servo_node/delta_joint_cmds` for MoveIt Servo.

Launch together with MoveIt + Servo + Web bridge:

```
ros2 launch so_arm_custom_control custom_ik_teleop.launch.py \
  websocket_url:=ws://127.0.0.1:8080/pose \
  reference_frame:=base end_effector_frame:=gripper
```

Tune via params (examples):
- `x_offset` (default 0.1629), `z_offset` (0.1131)
- `pan_gain_deg_per_m` (250.0)
- `velocity_gain` (0.6), `max_velocity_deg_s` (80.0)
- `pitch_scale` (1.0), `pitch_bias_deg` (0.0)

Notes:
- Joint names match the URDF: `Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`.
- The IK and joint-axis offsets mirror the user-provided reference implementation.
