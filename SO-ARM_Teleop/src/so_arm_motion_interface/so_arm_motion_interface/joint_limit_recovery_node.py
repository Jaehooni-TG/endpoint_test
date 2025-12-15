#!/usr/bin/env python3
"""Joint-limit-aware recovery helper for MoveIt Servo.

When Servo reports a non-zero status (e.g., JOINT_BOUND = 5) and a monitored
joint is close to a configured soft limit, this node briefly sends joint-based
commands to move that joint back toward the interior of its range.

This is intended to:
  - Help escape joint-limit stalls without fully stopping teleop
  - Keep the rest of the arm free to move once the critical joint is safe
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from control_msgs.msg import JointJog
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_srvs.srv import Trigger


class JointLimitRecoveryNode(Node):
    """Monitor Servo status and joint limits, nudge a single joint away from its limit when needed."""

    def __init__(self) -> None:
        super().__init__("joint_limit_recovery")

        # Parameters
        self.declare_parameter("status_topic", "/servo_node/status")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("command_out_topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("joint_name", "Wrist_Pitch")
        # Soft limits: slightly inside URDF hard limits, to keep away from the true mechanical stop.
        self.declare_parameter("upper_soft_limit", 1.62)
        self.declare_parameter("lower_soft_limit", -1.62)
        # Status codes that should trigger recovery (5 = JOINT_BOUND by default)
        self.declare_parameter("trigger_status_codes", [5])
        # If true, automatically request Servo restart for any non-zero status,
        # not just joint-limit stalls.
        self.declare_parameter("restart_on_any_error", True)
        # Recovery behavior
        self.declare_parameter("recovery_velocity", 0.25)  # rad/s magnitude
        self.declare_parameter("recovery_duration", 0.2)  # seconds (JointJog duration field)
        self.declare_parameter("check_period", 0.05)  # seconds

        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        joint_topic = self.get_parameter("joint_topic").get_parameter_value().string_value
        command_topic = self.get_parameter("command_out_topic").get_parameter_value().string_value
        self._joint_name = self.get_parameter("joint_name").get_parameter_value().string_value
        self._upper_soft = float(self.get_parameter("upper_soft_limit").value)
        self._lower_soft = float(self.get_parameter("lower_soft_limit").value)
        raw_codes = self.get_parameter("trigger_status_codes").value
        self._trigger_codes: List[int] = [int(c) for c in raw_codes] if isinstance(raw_codes, (list, tuple)) else [
            int(raw_codes)
        ]
        self._recovery_vel = float(self.get_parameter("recovery_velocity").value)
        self._recovery_duration = float(self.get_parameter("recovery_duration").value)
        check_period = max(0.02, float(self.get_parameter("check_period").value))
        self._restart_on_any_error = bool(self.get_parameter("restart_on_any_error").value)

        qos_status = QoSProfile(depth=5)
        qos_joint = QoSProfile(depth=50)
        self._status_sub = self.create_subscription(Int8, status_topic, self._on_status, qos_status)
        self._joint_sub = self.create_subscription(JointState, joint_topic, self._on_joint_state, qos_joint)
        self._cmd_pub = self.create_publisher(JointJog, command_topic, QoSProfile(depth=10))
        self._start_client = self.create_client(Trigger, "/servo_node/start_servo")

        self._latest_status: int = 0
        self._last_restart_reason: Optional[str] = None
        self._latest_joint_position: Optional[float] = None
        self._in_recovery: bool = False
        self._pending_restart: bool = False

        self._timer = self.create_timer(check_period, self._on_timer)

        self.get_logger().info(
            f"Joint limit recovery active for joint '{self._joint_name}' "
            f"(soft limits: [{self._lower_soft}, {self._upper_soft}])"
        )

    def _on_status(self, msg: Int8) -> None:
        previous = self._latest_status
        self._latest_status = int(msg.data)

        # If configured, auto-request restart whenever Servo status transitions
        # from "OK" (0) to any non-zero error/warning code.
        if (
            self._restart_on_any_error
            and previous == 0
            and self._latest_status != 0
        ):
            reason = f"status_code={self._latest_status}"
            self.get_logger().warn(
                f"Servo status changed from {previous} to {self._latest_status}; "
                f"requesting restart ({reason})."
            )
            self._maybe_restart_servo(reason=reason)

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self._joint_name)
        except ValueError:
            return
        if index < len(msg.position):
            self._latest_joint_position = float(msg.position[index])

    def _should_recover(self) -> Optional[float]:
        """Return recovery velocity direction (+/-) if we should recover, else None."""
        if self._latest_status not in self._trigger_codes:
            return None
        if self._latest_joint_position is None:
            return None

        pos = self._latest_joint_position
        # If above soft upper, move back down (negative velocity).
        if pos > self._upper_soft:
            return -abs(self._recovery_vel)
        # If below soft lower, move back up (positive velocity).
        if pos < self._lower_soft:
            return abs(self._recovery_vel)
        return None

    def _maybe_restart_servo(self, reason: Optional[str] = None) -> None:
        if self._pending_restart:
            return
        if not self._start_client.wait_for_service(timeout_sec=0.0):
            return
        # Store reason for logging once the service returns.
        self._last_restart_reason = reason
        request = Trigger.Request()
        future = self._start_client.call_async(request)
        future.add_done_callback(self._on_restart_response)
        self._pending_restart = True

    def _on_restart_response(self, future) -> None:
        self._pending_restart = False
        exc = future.exception()
        reason = self._last_restart_reason or "unspecified"
        self._last_restart_reason = None
        if exc is not None:
            self.get_logger().warn(
                f"Servo restart service raised an exception (reason={reason}): {exc}"
            )
            return
        response = future.result()
        if not response.success:
            self.get_logger().warn(
                f"Servo restart failed (reason={reason}): {response.message}"
            )
        else:
            self.get_logger().info(
                f"Servo restart requested successfully (reason={reason})."
            )

    def _on_timer(self) -> None:
        direction = self._should_recover()
        if direction is None:
            if self._in_recovery:
                self.get_logger().debug("Joint limit recovery finished.")
            self._in_recovery = False
            return

        # We are in a limit-triggered state for the monitored joint.
        self._in_recovery = True
        # Try to ensure Servo is in RUNNING state.
        self._maybe_restart_servo(
            reason=f"joint_limit_recovery:{self._joint_name},status={self._latest_status}"
        )

        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = ""  # not used by Servo
        jog.joint_names = [self._joint_name]
        jog.displacements = []  # velocity-only jog
        jog.velocities = [direction]
        jog.duration = self._recovery_duration

        self._cmd_pub.publish(jog)


def main() -> None:
    rclpy.init()
    node = JointLimitRecoveryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
