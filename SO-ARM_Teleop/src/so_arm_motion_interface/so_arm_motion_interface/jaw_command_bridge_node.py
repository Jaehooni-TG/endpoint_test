#!/usr/bin/env python3
"""Bridge a scalar Jaw command into a JointState on the HW command topic.

This node subscribes to a Float64 topic carrying a *delta* (radians per
message), accumulates it on the latest Jaw state, and publishes a JointState
message containing only the configured jaw joint. The Feetech bridge
(`so101_lerobot_bridge_node`) interprets this as a command for the gripper
motor, leaving other joints unchanged.
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
    self.declare_parameter("jaw_state_topic", "/so_arm/hw_joint_states")
    self.declare_parameter("hw_joint_command_topic", "/so_arm/hw_joint_command")
    self.declare_parameter("joint_name", "Jaw")

    jaw_cmd_topic = (
        self.get_parameter("jaw_command_topic").get_parameter_value().string_value
    )
    jaw_state_topic = (
        self.get_parameter("jaw_state_topic").get_parameter_value().string_value
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
    self._state_sub = self.create_subscription(
        JointState, jaw_state_topic, self._on_joint_state, qos
    )

    self._current_jaw: Optional[float] = None
    self._last_target: Optional[float] = None
    self._warned_missing_state = False

    self.get_logger().info(
        f"Jaw command bridge ready | jaw_topic: {jaw_cmd_topic} (delta) | "
        f"jaw_state_topic: {jaw_state_topic} | hw_joint_command_topic: {hw_cmd_topic} "
        f"| joint_name: {self._joint_name}"
    )

  def _on_joint_state(self, msg: JointState) -> None:
    if not msg.name or not msg.position:
      return
    try:
      idx = msg.name.index(self._joint_name)
    except ValueError:
      return
    if idx >= len(msg.position):
      return
    self._current_jaw = float(msg.position[idx])

  def _on_jaw_command(self, msg: Float64) -> None:
    delta = float(msg.data)
    base = self._current_jaw
    if base is None:
      base = self._last_target
    if base is None:
      if not self._warned_missing_state:
        self.get_logger().warn(
            "No Jaw state received yet; using 0.0 as baseline for delta."
        )
        self._warned_missing_state = True
      base = 0.0

    target = base + delta
    self._last_target = target

    js = JointState()
    js.header.stamp = self.get_clock().now().to_msg()
    js.name = [self._joint_name]
    js.position = [target]
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
