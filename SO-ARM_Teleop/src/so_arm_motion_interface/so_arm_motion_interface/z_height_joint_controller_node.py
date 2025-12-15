#!/usr/bin/env python3
"""Simple Z-height joint-space controller for SO-ARM101.

Goal:
  - Track desired EE Z position using a linear combination of
    Pitch, Elbow, and Wrist_Pitch joint velocities.
  - Designed as a testbed for \"position-prioritized\" behavior,
    separate from the Jacobian-based Twist Servo.

Inputs:
  - target_pose_topic (PoseStamped): desired EE pose (we only use .position.z)
  - current_pose_topic (PoseStamped): current EE pose (from feedback/TF)

Output:
  - JointJog on /servo_node/delta_joint_cmds, affecting joints:
      Pitch, Elbow, Wrist_Pitch

Sign conventions (from user observation):
  - For Z only, more negative Pitch, Elbow, Wrist_Pitch angles
    correspond to a higher EE Z.
  - Therefore, for z_error = z_target - z_current:
      z_error > 0 (want higher Z)  → negative joint velocities
      z_error < 0 (want lower Z)   → positive joint velocities
"""

from __future__ import annotations

import math
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointJog


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class ZHeightJointController(Node):
    """Joint-space Z controller using Pitch, Elbow, Wrist_Pitch."""

    def __init__(self) -> None:
        super().__init__("z_height_joint_controller")

        # Topics
        self.declare_parameter("target_pose_topic", "/so_arm/pose_cmd")
        self.declare_parameter("current_pose_topic", "/so_arm/pose_state")
        self.declare_parameter("joint_command_topic", "/servo_node/delta_joint_cmds")

        # Joint mapping (order matters)
        self.declare_parameter(
            "joint_names", ["Pitch", "Elbow", "Wrist_Pitch"]
        )

        # Control gains: v_joint = -k_j * z_error  (k_j > 0)
        self.declare_parameter("kp_pitch", 2.0)
        self.declare_parameter("kp_elbow", 1.5)
        self.declare_parameter("kp_wrist_pitch", 1.0)

        # Limits
        self.declare_parameter("max_joint_velocity", 1.5)  # rad/s
        self.declare_parameter("deadband_z", 0.002)  # m

        # Control rate
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("pose_timeout", 0.5)

        target_topic = (
            self.get_parameter("target_pose_topic").get_parameter_value().string_value
        )
        current_topic = (
            self.get_parameter("current_pose_topic").get_parameter_value().string_value
        )
        cmd_topic = (
            self.get_parameter("joint_command_topic").get_parameter_value().string_value
        )
        joint_names_param = self.get_parameter("joint_names").value
        self._joint_names: List[str] = [str(n) for n in joint_names_param]

        self._kp_pitch = float(self.get_parameter("kp_pitch").value)
        self._kp_elbow = float(self.get_parameter("kp_elbow").value)
        self._kp_wrist_pitch = float(self.get_parameter("kp_wrist_pitch").value)
        self._max_vel = abs(float(self.get_parameter("max_joint_velocity").value))
        self._deadband_z = abs(float(self.get_parameter("deadband_z").value))
        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._pose_timeout = max(0.0, float(self.get_parameter("pose_timeout").value))

        qos_pose = QoSProfile(depth=10)
        self.create_subscription(
            PoseStamped, target_topic, self._on_target_pose, qos_pose
        )
        self.create_subscription(
            PoseStamped, current_topic, self._on_current_pose, qos_pose
        )

        self._cmd_pub = self.create_publisher(JointJog, cmd_topic, QoSProfile(depth=10))

        self._z_target: Optional[float] = None
        self._z_target_stamp: Optional[Time] = None
        self._z_current: Optional[float] = None
        self._z_current_stamp: Optional[Time] = None

        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.02
        self._timer = self.create_timer(period, self._control_step)

        self.get_logger().info(
            f"Z-height joint controller active | target: {target_topic} | "
            f"current: {current_topic} | cmd: {cmd_topic} | joints: {self._joint_names}"
        )

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._z_target = float(msg.pose.position.z)
        try:
            self._z_target_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self._z_target_stamp = self.get_clock().now()

    def _on_current_pose(self, msg: PoseStamped) -> None:
        self._z_current = float(msg.pose.position.z)
        try:
            self._z_current_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self._z_current_stamp = self.get_clock().now()

    def _is_pose_stale(self, stamp: Optional[Time]) -> bool:
        if self._pose_timeout <= 0.0 or stamp is None:
            return False
        now = self.get_clock().now()
        return (now - stamp) > Duration(seconds=self._pose_timeout)

    def _control_step(self) -> None:
        # Need both target and current Z
        if self._z_target is None or self._z_current is None:
            return
        if self._is_pose_stale(self._z_target_stamp) or self._is_pose_stale(
            self._z_current_stamp
        ):
            return

        z_err = self._z_target - self._z_current
        if abs(z_err) < self._deadband_z:
            return

        # For Z: more negative joint angles → higher Z (사용자 관찰 기준)
        # 따라서 z_err>0 (위로 올리기)일 때 joint 속도는 음수 방향으로 가야 한다.
        # v = -k * z_err (k > 0).
        v_pitch = -self._kp_pitch * z_err
        v_elbow = -self._kp_elbow * z_err
        v_wpitch = -self._kp_wrist_pitch * z_err

        v_pitch = clamp(v_pitch, -self._max_vel, self._max_vel)
        v_elbow = clamp(v_elbow, -self._max_vel, self._max_vel)
        v_wpitch = clamp(v_wpitch, -self._max_vel, self._max_vel)

        # Build JointJog
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # not used by Servo
        msg.joint_names = self._joint_names
        # Map velocities to joint_names order
        name_to_vel: Dict[str, float] = {
            "Pitch": v_pitch,
            "Elbow": v_elbow,
            "Wrist_Pitch": v_wpitch,
        }
        msg.velocities = [name_to_vel.get(name, 0.0) for name in self._joint_names]
        try:
            msg.duration = Duration(seconds=self._timer.timer_period_ns / 1e9).to_msg()
        except Exception:
            pass

        self._cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ZHeightJointController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
