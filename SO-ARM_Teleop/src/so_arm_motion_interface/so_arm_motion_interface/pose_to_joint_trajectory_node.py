#!/usr/bin/env python3
"""Pose → JointTrajectory interface without MoveIt Servo.

This node:
  - Subscribes to a target end-effector pose (PoseStamped)
  - Uses MoveIt's GetPositionIK service to compute a joint solution
  - Publishes a JointTrajectory directly to the arm_controller

It is intended as a simple, global-IK-based alternative path to test
\"곱게 뻗는\" 움직임 when Servo's Jacobian-based behavior is not desired.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK


class PoseToJointTrajectoryNode(Node):
    """Convert target EE pose into joint-space commands via MoveIt IK."""

    def __init__(self) -> None:
        super().__init__("pose_to_joint_trajectory")

        # Parameters
        self.declare_parameter("input_topic", "/so_arm/pose_cmd")
        self.declare_parameter("joint_trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link_name", "gripper")
        self.declare_parameter("ik_timeout", 0.2)
        self.declare_parameter("ik_attempts", 1)
        self.declare_parameter("motion_time", 0.6)  # seconds to reach new pose
        # Order must match ros2_controllers.yaml arm_controller.joints
        self.declare_parameter(
            "controlled_joints",
            ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"],
        )

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        traj_topic = (
            self.get_parameter("joint_trajectory_topic").get_parameter_value().string_value
        )
        joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self._group_name = self.get_parameter("group_name").get_parameter_value().string_value
        self._ik_link_name = self.get_parameter("ik_link_name").get_parameter_value().string_value
        self._ik_timeout = float(self.get_parameter("ik_timeout").value)
        self._ik_attempts = int(self.get_parameter("ik_attempts").value)
        joints_param = self.get_parameter("controlled_joints").value
        self._controlled_joints: List[str] = [str(n) for n in joints_param]

        qos = QoSProfile(depth=10)
        self.create_subscription(PoseStamped, input_topic, self._on_pose, qos)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, qos)
        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, qos)

        self._ik_client = self.create_client(GetPositionIK, "compute_ik")

        self._latest_joint_state: Optional[JointState] = None

        self.get_logger().info(
            "Pose→JointTrajectory IK node started | "
            f"input: {input_topic} | traj: {traj_topic} | group: {self._group_name}"
        )

    # -------------------- Callbacks --------------------

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        if self._latest_joint_state is None:
            self.get_logger().debug("No joint_state yet; skipping pose command.")
            return

        if not self._ik_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn_once("compute_ik service not available; cannot execute pose commands.")
            return

        req = GetPositionIK.Request()
        req.ik_request.group_name = self._group_name
        req.ik_request.ik_link_name = self._ik_link_name
        req.ik_request.pose_stamped = msg
        req.ik_request.timeout = Duration(seconds=self._ik_timeout).to_msg()
        req.ik_request.attempts = self._ik_attempts
        # Seed with current joint state for smoother solutions
        req.ik_request.robot_state.joint_state = self._latest_joint_state

        future = self._ik_client.call_async(req)
        future.add_done_callback(self._on_ik_result)

    # -------------------- IK handling --------------------

    def _on_ik_result(self, future) -> None:
        try:
            res = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"IK service call failed: {exc}")
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().debug(f"IK failed with error code {res.error_code.val}; skipping command.")
            return

        solution_js = res.solution.joint_state
        if not solution_js.name:
            self.get_logger().warn("IK solution has empty joint_state; skipping.")
            return

        name_to_pos: Dict[str, float] = {
            n: p for n, p in zip(solution_js.name, solution_js.position)
        }

        # Build JointTrajectory in controller joint order
        positions: List[float] = []
        for joint in self._controlled_joints:
            if joint not in name_to_pos:
                self.get_logger().warn_once(
                    f"IK solution missing joint '{joint}'; using last known or zero."
                )
                positions.append(0.0)
            else:
                positions.append(float(name_to_pos[joint]))

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self._controlled_joints

        pt = JointTrajectoryPoint()
        pt.positions = positions
        motion_time = float(self.get_parameter("motion_time").value)
        if motion_time <= 0.0:
            motion_time = 0.6
        pt.time_from_start = Duration(seconds=motion_time).to_msg()
        traj.points.append(pt)

        self._traj_pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = PoseToJointTrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

