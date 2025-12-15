#!/usr/bin/env python3
"""One-shot helper node to enable orientation drift in MoveIt Servo.

This tells Servo that rotational DOFs (roll, pitch, yaw) are allowed to drift,
while translations (x, y, z) remain controlled. Effectively it prioritizes
position tracking over strict orientation tracking.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ChangeDriftDimensions


class EnableServoOrientationDrift(Node):
    """Call /servo_node/change_drift_dimensions once, then exit."""

    def __init__(self) -> None:
        super().__init__("enable_servo_orientation_drift")

        self._service_name = "/servo_node/change_drift_dimensions"
        self._client = self.create_client(ChangeDriftDimensions, self._service_name)

        timeout_sec = 5.0
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Service {self._service_name} not available after {timeout_sec} s; "
                "orientation drift will not be enabled."
            )
            # Exit quickly; teleop can still run without this.
            rclpy.shutdown()
            sys.exit(0)

        self.get_logger().info(
            "Enabling orientation drift in MoveIt Servo "
            "(translations controlled, rotations allowed to drift)."
        )
        self._call_service()

    def _call_service(self) -> None:
        req = ChangeDriftDimensions.Request()
        # Disable drift on all dimensions so Servo fully controls both
        # translations (x, y, z) and rotations (roll, pitch, yaw).
        req.drift_x_translation = False
        req.drift_y_translation = False
        req.drift_z_translation = False
        req.drift_x_rotation = False
        req.drift_y_rotation = False
        req.drift_z_rotation = False
        # Leave transform_jog_frame_to_drift_frame as default (identity assumed by implementation).

        future = self._client.call_async(req)

        def _on_done(fut) -> None:
            try:
                resp = fut.result()
            except Exception as exc:  # pragma: no cover - unexpected error path
                self.get_logger().warn(f"Failed to enable orientation drift: {exc}")
            else:
                if not resp.success:
                    self.get_logger().warn("ChangeDriftDimensions reported failure; drift not enabled.")
                else:
                    self.get_logger().info("Orientation drift enabled successfully.")
            # Shutdown after one-shot call.
            rclpy.shutdown()
            sys.exit(0)

        future.add_done_callback(_on_done)


def main() -> None:
    rclpy.init()
    node = EnableServoOrientationDrift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
