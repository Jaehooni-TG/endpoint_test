# so_arm_custom_control

This package previously contained a custom planar IK controller node
(`custom_ik_controller_node`) and a corresponding launch file
(`custom_ik_teleop.launch.py`) for driving SO‑ARM101 directly from web poses.

The main teleoperation path has since been migrated to MoveIt Servo +
`pose_to_servo_node` + `websocket_pose_bridge_node`, and the custom IK controller
is no longer used or maintained in this workspace.

The controller node and its launch file have been removed to avoid confusion.
