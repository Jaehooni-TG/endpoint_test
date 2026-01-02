#!/usr/bin/env python3
"""ROS 2 bridge between MoveIt Servo (via ros2_control TopicBasedSystem)
and a physical SO-ARM101 arm driven over a Feetech serial bus.

This version no longer depends on the external ``lerobot`` Python package.
Instead it uses a tiny local driver (``so101_feetech_driver.So101FeetechArm``)
that talks to the STS3215 motors through ``scservo_sdk``.

ros2_control is assumed to be configured with ``topic_based_ros2_control/TopicBasedSystem``
using something like:

  <param name="joint_commands_topic">/so_arm/hw_joint_command</param>
  <param name="joint_states_topic">/so_arm/hw_joint_states</param>

This node then:
- Subscribes to ``/so_arm/hw_joint_command`` (JointState) and forwards the
  latest point as joint goals to the hardware driver.
- Periodically reads joint positions from the hardware and publishes them on
  ``/so_arm/hw_joint_states`` as ``sensor_msgs/JointState``.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from .so101_feetech_driver import So101FeetechArm


# Ordered list of ROS joint names we care about.
ORDERED_ROS_JOINTS: List[str] = [
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
]


class So101LerobotBridge(Node):
    """Bridge JointTrajectory/JointState topics to lerobot's SO101Follower."""

    def __init__(self) -> None:
        super().__init__("so101_lerobot_bridge")

        # Parameters for lerobot SO101Follower
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("robot_id", "so101")
        self.declare_parameter("calibration_dir", "")
        self.declare_parameter("max_relative_target", 30.0)  # unused in simple driver
        self.declare_parameter("use_degrees", True)
        # When false, the bridge opens the serial bus but leaves torque disabled
        # so the arm can be backdriven by hand (state-only / hand-guiding).
        self.declare_parameter("enable_torque_on_connect", True)
        # ROS topic parameters (must match TopicBasedSystem in ros2_control)
        self.declare_parameter("joint_command_topic", "/so_arm/hw_joint_command")
        self.declare_parameter("joint_state_topic", "/so_arm/hw_joint_states")
        self.declare_parameter("state_publish_rate", 50.0)  # Hz

        port = self.get_parameter("port").get_parameter_value().string_value
        robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        calib_dir_str = self.get_parameter("calibration_dir").get_parameter_value().string_value
        max_rel = float(self.get_parameter("max_relative_target").value)
        use_degrees = bool(self.get_parameter("use_degrees").value)
        enable_torque = bool(self.get_parameter("enable_torque_on_connect").value)

        calibration_dir: Optional[Path] = Path(calib_dir_str) if calib_dir_str else None
        if not calibration_dir:
            raise RuntimeError("Parameter 'calibration_dir' must point to a calibration JSON directory.")
        calib_file = calibration_dir / f"{robot_id}.json"

        self.get_logger().info(
            f"Connecting SO101 Feetech arm on port '{port}' "
            f"(calibration={calib_file}), torque_on_connect={enable_torque}"
        )
        self._arm = So101FeetechArm(port=port, calibration_file=calib_file)
        self._arm.connect(enable_torque=enable_torque)

        # ROS I/O
        joint_command_topic = self.get_parameter("joint_command_topic").get_parameter_value().string_value
        joint_state_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        state_rate_hz = max(1.0, float(self.get_parameter("state_publish_rate").value))

        qos_cmd = QoSProfile(depth=10)
        qos_state = QoSProfile(depth=10)

        self._cmd_sub = self.create_subscription(
            JointState,
            joint_command_topic,
            self._on_joint_command,
            qos_cmd,
        )
        self._state_pub = self.create_publisher(JointState, joint_state_topic, qos_state)

        self._state_timer = self.create_timer(1.0 / state_rate_hz, self._publish_joint_state)

        self.get_logger().info(
            f"SO101 lerobot bridge ready | cmd: {joint_command_topic} -> port:{port} | "
            f"state: {joint_state_topic}"
        )

    # ---- JointState commands → hardware action ----

    def _on_joint_command(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        # TopicBasedSystem publishes desired joint positions in radians
        # on the joint_commands_topic. Convert the joints we care about
        # into degrees and forward them to the Feetech driver.
        joint_deg: Dict[str, float] = {}
        for name, pos in zip(msg.name, msg.position):
            if name not in ORDERED_ROS_JOINTS:
                continue
            joint_deg[name] = math.degrees(float(pos))

        if not joint_deg:
            return

        try:
            self._arm.send_action(joint_deg)
        except Exception as exc:  # pragma: no cover - transport errors
            self.get_logger().warn(f"Failed to send action to Feetech arm: {exc}")

    # ---- lerobot observation → JointState ----

    def _publish_joint_state(self) -> None:
        try:
            joint_deg = self._arm.get_observation()
        except Exception as exc:  # pragma: no cover - transport errors
            self.get_logger().warn(f"Failed to read state from Feetech arm: {exc}")
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ORDERED_ROS_JOINTS

        positions: List[float] = []
        for ros_name in ORDERED_ROS_JOINTS:
            deg = float(joint_deg.get(ros_name, 0.0))
            positions.append(math.radians(deg))

        js.position = positions
        self._state_pub.publish(js)

    def destroy_node(self) -> bool:
        try:
            self._arm.disconnect()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = So101LerobotBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
