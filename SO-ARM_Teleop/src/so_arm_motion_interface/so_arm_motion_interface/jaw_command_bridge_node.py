#!/usr/bin/env python3
"""Bridge a scalar Jaw command into a JointState on the HW command topic.

This node subscribes to a Float64 topic (jaw angle in radians) and publishes a
JointState message containing only the configured jaw joint. The Feetech
bridge (`so101_lerobot_bridge_node`) will interpret this as a command for the
gripper motor, leaving other joints unchanged.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JawCommandBridgeNode(Node):
  """Convert Float64 jaw commands into JointState messages."""

  def __init__(self) -> None:
    super().__init__("so_arm_jaw_command_bridge")

    self.declare_parameter("jaw_command_topic", "/so_arm/jaw_command")
    self.declare_parameter("hw_joint_command_topic", "/so_arm/hw_joint_command")
    self.declare_parameter("joint_name", "Jaw")

    jaw_cmd_topic = (
        self.get_parameter("jaw_command_topic").get_parameter_value().string_value
    )
    hw_cmd_topic = (
        self.get_parameter("hw_joint_command_topic").get_parameter_value().string_value
    )
    self._joint_name = (
        self.get_parameter("joint_name").get_parameter_value().string_value
    )

    qos = QoSProfile(depth=10)
    self._publisher = self.create_publisher(JointState, hw_cmd_topic, qos)
    self._subscription = self.create_subscription(
        Float64, jaw_cmd_topic, self._on_jaw_command, qos
    )

    self.get_logger().info(
        f"Jaw command bridge ready | jaw_topic: {jaw_cmd_topic} → hw_joint_command_topic: {hw_cmd_topic} "
        f"| joint_name: {self._joint_name}"
    )

  def _on_jaw_command(self, msg: Float64) -> None:
    js = JointState()
    js.header.stamp = self.get_clock().now().to_msg()
    js.name = [self._joint_name]
    js.position = [float(msg.data)]
    self._publisher.publish(js)


def main(args: Optional[list[str]] = None) -> None:
  rclpy.init(args=args)
  node = JawCommandBridgeNode()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()

