#!/usr/bin/env python3
"""One-shot initial self-collision guard using MoveIt.

This node:
  1. Waits for a /joint_states message.
  2. Calls MoveIt's /check_state_validity service with that state.
  3. Exits with code:
       0 → state is valid or check could not be performed
       1 → state is in self-collision

It is intended to be run once from a shell script before starting Servo so that
teleoperation does not begin from an obviously colliding configuration.
"""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity


class InitialStateCollisionGuard(Node):
    """Check current joint state against MoveIt's self-collision service."""

    def __init__(self) -> None:
        super().__init__("initial_state_collision_guard")

        # Parameters (kept minimal on purpose).
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("timeout_sec", 10.0)

        joint_topic = self.get_parameter("joint_topic").get_parameter_value().string_value
        self._group_name = self.get_parameter("group_name").get_parameter_value().string_value
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)

        self._latest_joint_state: Optional[JointState] = None

        qos = QoSProfile(depth=10)
        self._joint_sub = self.create_subscription(
            JointState, joint_topic, self._on_joint_state, qos
        )

        self._client = self.create_client(GetStateValidity, "/check_state_validity")

    def _on_joint_state(self, msg: JointState) -> None:
        if self._latest_joint_state is None:
            self._latest_joint_state = msg

    def wait_for_joint_state(self) -> bool:
        """Wait for a single JointState message within timeout."""
        self.get_logger().info(
            f"Waiting for '{self._joint_sub.topic}' to provide joint states "
            f"(timeout={self._timeout_sec:.1f}s)..."
        )
        elapsed = 0.0
        step = 0.1
        while rclpy.ok() and self._latest_joint_state is None and elapsed < self._timeout_sec:
            rclpy.spin_once(self, timeout_sec=step)
            elapsed += step

        if self._latest_joint_state is None:
            self.get_logger().warn(
                "No JointState received within timeout; skipping collision guard."
            )
            return False
        return True

    def wait_for_service(self) -> bool:
        """Wait for the MoveIt validity service."""
        self.get_logger().info("Waiting for /check_state_validity service...")
        if not self._client.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().warn(
                "MoveIt /check_state_validity service not available; "
                "skipping collision guard."
            )
            return False
        return True

    def check_current_state(self) -> Optional[bool]:
        """Return True if valid, False if in collision, None on failure."""
        if self._latest_joint_state is None:
            return None

        req = GetStateValidity.Request()
        robot_state = RobotState()
        robot_state.joint_state = self._latest_joint_state
        req.robot_state = robot_state
        req.group_name = self._group_name

        future = self._client.call_async(req)
        self.get_logger().info(
            f"Calling /check_state_validity for group '{self._group_name}'..."
        )
        rclpy.spin_until_future_complete(self, future)
        if future.cancelled():
            self.get_logger().warn("GetStateValidity request was cancelled.")
            return None
        if future.exception() is not None:
            self.get_logger().warn(
                f"GetStateValidity raised an exception: {future.exception()}"
            )
            return None

        resp = future.result()
        if resp is None:
            self.get_logger().warn("GetStateValidity returned no response.")
            return None

        if resp.valid:
            self.get_logger().info("Initial joint state is collision-free.")
            return True

        # In collision: report a few contact pairs for debugging.
        self.get_logger().error("Initial joint state is in self-collision.")
        if resp.contacts:
            pairs = []
            for c in resp.contacts[:5]:
                pairs.append(f"{c.contact_body_1} ↔ {c.contact_body_2}")
            self.get_logger().error(
                "Example contact pairs: " + "; ".join(pairs)
            )
        return False


def main() -> None:
    rclpy.init()
    node = InitialStateCollisionGuard()
    try:
        # Best-effort: if we cannot get joint states or the service, do not block startup.
        if not node.wait_for_joint_state():
            rc = 0
        elif not node.wait_for_service():
            rc = 0
        else:
            result = node.check_current_state()
            # None → check failed; treat as "do not block".
            rc = 0 if result is None or result else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()

