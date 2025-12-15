#!/usr/bin/env python3
"""Convert geometry_msgs/Twist commands into TwistStamped for MoveIt Servo."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile


class TwistToServoNode(Node):
    """Forward Twist commands to MoveIt Servo with appropriate header information."""

    def __init__(self) -> None:
        super().__init__("so_arm_twist_to_servo")

        self.declare_parameter("input_topic", "/so_arm/cmd_vel")
        self.declare_parameter("output_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("command_frame", "base")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._command_frame = (
            self.get_parameter("command_frame").get_parameter_value().string_value
        )

        qos = QoSProfile(depth=10)
        self._publisher = self.create_publisher(TwistStamped, output_topic, qos)
        self.create_subscription(Twist, input_topic, self._on_twist, qos)

        self.get_logger().info(
            f"Forwarding Twist commands from {input_topic} to {output_topic} in frame '{self._command_frame}'"
        )

    def _on_twist(self, msg: Twist) -> None:
        """Wrap Twist in TwistStamped with the configured frame."""
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._command_frame
        stamped.twist = msg
        self._publisher.publish(stamped)


def main() -> None:
    rclpy.init()
    node = TwistToServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
