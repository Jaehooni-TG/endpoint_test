#!/usr/bin/env python3
"""
Publish a single PoseStamped test command to /so_arm/pose_cmd.

Usage:
  source /opt/ros/humble/setup.bash
  source SO-ARM101_MoveIt_IsaacSim/install/setup.bash
  python3 SO-ARM101_MoveIt_IsaacSim/scripts/publish_test_pose.py --x 0.30 --y 0.00 --z 0.20

This helps validate the Pose -> Servo -> JointTrajectory pipeline without any WebSocket input.
"""

from __future__ import annotations

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a test PoseStamped to /so_arm/pose_cmd")
    parser.add_argument("--topic", default="/so_arm/pose_cmd", help="Pose topic")
    parser.add_argument("--frame", default="base", help="Reference frame")
    parser.add_argument("--x", type=float, default=0.30)
    parser.add_argument("--y", type=float, default=0.00)
    parser.add_argument("--z", type=float, default=0.20)
    args = parser.parse_args()

    rclpy.init()
    node = Node("publish_test_pose")
    pub = node.create_publisher(PoseStamped, args.topic, 10)

    msg = PoseStamped()
    msg.header.frame_id = args.frame
    msg.pose.position.x = float(args.x)
    msg.pose.position.y = float(args.y)
    msg.pose.position.z = float(args.z)
    # Keep current orientation (identity)
    msg.pose.orientation.w = 1.0

    # Stamp and publish once
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    node.get_logger().info(
        f"Published test pose to {args.topic} in frame '{args.frame}': x={args.x:.3f}, y={args.y:.3f}, z={args.z:.3f}"
    )

    # Small spin to let message flush
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

