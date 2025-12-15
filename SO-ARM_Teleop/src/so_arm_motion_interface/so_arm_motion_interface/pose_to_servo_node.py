#!/usr/bin/env python3
"""Convert target PoseStamped commands into Twist commands for MoveIt Servo.

Minimal version: position-only tracking in the reference frame (base).
"""

from __future__ import annotations

import math
from typing import Optional
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf_transformations import quaternion_matrix, quaternion_conjugate, quaternion_multiply


class PoseToServoNode(Node):
    """Track a streamed end-effector pose by publishing velocity commands."""

    def __init__(self) -> None:
        super().__init__("so_arm_pose_to_servo")

        self.declare_parameter("input_topic", "/so_arm/pose_cmd")
        # Direct publication to MoveIt Servo (TwistStamped)
        self.declare_parameter("servo_output_topic", "/servo_node/delta_twist_cmds")
        # Backward-compat: ignored but declared to avoid launch-time errors
        self.declare_parameter("publish_servo_twiststamped", True)
        self.declare_parameter("publish_cmd_vel", False)
        self.declare_parameter("output_topic", "/so_arm/cmd_vel")
        self.declare_parameter("reference_frame", "base")
        self.declare_parameter("end_effector_frame", "gripper")
        self.declare_parameter("linear_gain", 4.0)
        self.declare_parameter("max_linear_speed", 0.25)
        # Toggle: command twist in EE(local) frame vs base(reference) frame
        # Default: base frame for more intuitive vertical motion
        self.declare_parameter("command_in_ee", False)
        self.declare_parameter("angular_gain", 2.0)
        self.declare_parameter("max_angular_speed", 1.5)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        servo_output_topic = (
            self.get_parameter("servo_output_topic").get_parameter_value().string_value
        )
        self._reference_frame = (
            self.get_parameter("reference_frame").get_parameter_value().string_value
        )
        self._ee_frame = (
            self.get_parameter("end_effector_frame").get_parameter_value().string_value
        )
        self._linear_gain = self.get_parameter("linear_gain").get_parameter_value().double_value
        self._max_linear = (
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )
        self._angular_gain = self.get_parameter("angular_gain").get_parameter_value().double_value
        self._max_angular = (
            self.get_parameter("max_angular_speed").get_parameter_value().double_value
        )
        # MoveIt Servo 설정(robot_link_command_frame: gripper)에 맞춰 EE(local) 프레임으로 명령(토글 가능)
        self._command_in_ee = (
            self.get_parameter("command_in_ee").get_parameter_value().bool_value
        )

        # Orientation tracking enabled (angular velocity from orientation error)

        qos = QoSProfile(depth=10)
        self._servo_pub = self.create_publisher(TwistStamped, servo_output_topic, qos)
        self.create_subscription(PoseStamped, input_topic, self._on_pose, qos)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

        self.get_logger().info(
            f"Pose tracking active | input: {input_topic} | output: servo:{servo_output_topic} | ref: {self._reference_frame} | cmd_frame: {'gripper' if self._command_in_ee else self._reference_frame}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        try:
            target_pose = self._transform_pose(msg, self._reference_frame)
            current_tf = self._tf_buffer.lookup_transform(
                self._reference_frame,
                self._ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.TimeoutException) as err:
            self.get_logger().warn_once(f"TF lookup failed: {err}")
            return

        target_pos = np.array(
            [
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
            ],
            dtype=float,
        )
        current_pos = np.array(
            [
                current_tf.transform.translation.x,
                current_tf.transform.translation.y,
                current_tf.transform.translation.z,
            ],
            dtype=float,
        )

        linear_error = target_pos - current_pos
        raw_linear_cmd = self._linear_gain * linear_error
        linear_cmd = raw_linear_cmd

        # No Z-pitch coupling in baseline

        linear_norm = np.linalg.norm(linear_cmd)
        if linear_norm > self._max_linear > 0.0:
            linear_cmd *= self._max_linear / linear_norm

        # Orientation error → angular velocity
        tq = target_pose.pose.orientation
        cq = current_tf.transform.rotation
        # Use xyzw ordering for tf_transformations
        q_target = np.array([tq.x, tq.y, tq.z, tq.w], dtype=float)
        q_current = np.array([cq.x, cq.y, cq.z, cq.w], dtype=float)
        # rotation bringing current -> target
        q_err = quaternion_multiply(q_target, quaternion_conjugate(q_current))
        # Ensure ndarray type for safe math operations
        q_err = np.asarray(q_err, dtype=float)
        # Ensure sign consistency for shortest path
        if q_err[3] < 0.0:
            q_err = -q_err
        w = float(q_err[3])
        w = max(min(w, 1.0), -1.0)
        angle = 2.0 * math.acos(w)
        s = math.sqrt(max(1.0 - w * w, 0.0))
        if s < 1e-6 or angle < 1e-6:
            axis = np.array([0.0, 0.0, 0.0], dtype=float)
        else:
            axis = (q_err[0:3] / s).astype(float, copy=False)
        angular_error = axis * angle
        raw_angular_cmd = self._angular_gain * angular_error
        ang_norm = float(np.linalg.norm(raw_angular_cmd))
        if ang_norm > self._max_angular > 0.0:
            angular_cmd = raw_angular_cmd * (self._max_angular / ang_norm)
        else:
            angular_cmd = raw_angular_cmd

        # Optionally express commanded twist in EE(local) frame
        if self._command_in_ee:
            # current_tf gives transform ee -> reference. Its rotation R_be maps ee-local to base.
            # To express base-frame vectors in ee-local, use R_eb = R_be^T.
            q = current_tf.transform.rotation
            R_be = quaternion_matrix([q.x, q.y, q.z, q.w])[0:3, 0:3]
            R_eb = R_be.T
            linear_cmd_local = R_eb.dot(linear_cmd)
            angular_cmd_local = R_eb.dot(angular_cmd)
            frame_id = self._ee_frame
            out_linear = linear_cmd_local
            out_angular = angular_cmd_local
        else:
            frame_id = self._reference_frame
            out_linear = linear_cmd
            out_angular = angular_cmd

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = out_linear
        twist.angular.x, twist.angular.y, twist.angular.z = out_angular

        if self._servo_pub is not None:
            stamped = TwistStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = frame_id
            stamped.twist = twist
            self._servo_pub.publish(stamped)

        # no raw Twist publication

    def _transform_pose(self, pose: PoseStamped, frame: str) -> PoseStamped:
        if pose.header.frame_id == frame or pose.header.frame_id == "":
            if pose.header.frame_id == "":
                pose.header.frame_id = frame
            return pose
        return self._tf_buffer.transform(
            pose,
            frame,
            timeout=rclpy.duration.Duration(seconds=0.2),
        )

    # Orientation/adaptive logic removed in minimal node


def main() -> None:
    rclpy.init()
    node = PoseToServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
