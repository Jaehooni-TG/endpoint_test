#!/usr/bin/env python3
"""Custom planar IK controller for SO-ARM101 driven by web pose.

Consumes PoseStamped (from websocket bridge) and publishes JointJog to MoveIt Servo.

Behavior:
- Lateral motion (Y) → shoulder pan (Rotation) only
- Planar IK on X-Z plane → shoulder lift (Pitch) and elbow flex (Elbow)
- Wrist pitch (Wrist_Pitch) couples to keep EE pitch: wrist = -(Pitch+Elbow) + pitch
- Wrist roll (Wrist_Roll) follows EE roll

Angles are handled in degrees for readability internally and converted to rad for output.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from tf_transformations import euler_from_quaternion


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class PlanarKinematics:
    """2-link planar IK with joint-axis offsets tailored for SO-ARM101.

    Link lengths (l1, l2) and theta offsets are derived from the URDF geometry.
    Returns angles for shoulder_lift (Pitch) and elbow_flex (Elbow) in degrees.
    """

    l1: float = 0.11257
    l2: float = 0.1349
    theta1_offset: float = math.atan2(0.028, 0.11257)  # when Pitch==0
    theta2_offset: float = math.atan2(0.0052, 0.1349) + math.atan2(0.028, 0.11257)  # when Elbow==0

    def inverse_kinematics(self, x: float, y: float, elbow_up: bool = False, lift_transform_intuitive: bool = True) -> tuple[float, float]:
        """Compute IK for end-effector point (x,y) in meters in the vertical X-Z plane.

        Returns (Pitch_deg, Elbow_deg) as degrees in the robot joint sign convention used by the URDF.
        The computation mirrors the reference logic provided by the user.
        """
        l1, l2 = self.l1, self.l2

        r = math.hypot(x, y)
        r_max = l1 + l2
        if r > r_max and r > 1e-9:
            scale = r_max / r
            x *= scale
            y *= scale
            r = r_max

        r_min = abs(l1 - l2)
        if 0.0 < r < r_min:
            scale = r_min / r
            x *= scale
            y *= scale
            r = r_min

        cos_theta2 = -(r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        cos_theta2 = clamp(cos_theta2, -1.0, 1.0)
        # Choose elbow branch: down (default) or up (mirrored)
        base_angle = math.pi - math.acos(cos_theta2)
        theta2 = -base_angle if elbow_up else base_angle

        beta = math.atan2(y, x)
        gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        theta1 = beta + gamma

        # Convert to actual joint references with axis offsets
        joint2 = theta1 + self.theta1_offset
        joint3 = theta2 + self.theta2_offset

        # Clamp to URDF joint limits (radians)
        joint2 = clamp(joint2, -0.1, 3.45)  # Pitch
        joint3 = clamp(joint3, -0.2, math.pi)  # Elbow

        # Degrees; mapping mode:
        # - Raw mapping (URDF 0° == neutral 0°): joint*_deg = deg(joint*)
        # - Intuitive mapping (optional): subtract 90° so z+ tends to lift shoulder
        joint2_deg = math.degrees(joint2)
        joint3_deg = math.degrees(joint3)
        if lift_transform_intuitive:
            joint2_deg = joint2_deg - 90.0
            joint3_deg = joint3_deg - 90.0
        return joint2_deg, joint3_deg


class CustomIKControllerNode(Node):
    """Custom IK controller that converts Pose to joint velocities for MoveIt Servo."""

    def __init__(self) -> None:
        super().__init__('so_arm_custom_ik_controller')

        # Topics and frames
        self.declare_parameter('input_pose_topic', '/so_arm/pose_cmd')
        self.declare_parameter('joint_command_topic', '/servo_node/delta_joint_cmds')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('reference_frame', 'base')

        # Geometry and mapping params
        self.declare_parameter('x_offset', 0.0)  # meters (subtractive)
        self.declare_parameter('z_offset', 0.0)  # meters (subtractive)
        self.declare_parameter('pan_gain_deg_per_m', 250.0)
        self.declare_parameter('pan_invert', False)
        self.declare_parameter('roll_scale', 1.0)  # multiplier on roll(deg)
        self.declare_parameter('pitch_scale', 1.0)  # multiplier on pitch(deg)
        self.declare_parameter('pitch_bias_deg', 0.0)

        # Kinematics link lengths and offsets
        self.declare_parameter('link1_length', 0.11257)
        self.declare_parameter('link2_length', 0.1349)

        # Control loop params
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('velocity_gain', 0.6)  # rad/s per rad error
        self.declare_parameter('max_velocity_deg_s', 80.0)
        self.declare_parameter('require_pose_to_activate', True)
        self.declare_parameter('pose_timeout', 0.8)  # seconds, 0 or negative disables timeout
        self.declare_parameter('initialize_targets_from_joint_state', True)
        self.declare_parameter('elbow_up', False)
        # Use raw mapping by default (URDF 0° == neutral 0°). Set true to apply the old intuitive transform.
        self.declare_parameter('lift_transform_intuitive', False)
        # Branch/selection preferences
        self.declare_parameter('prefer_elbow_for_vertical', True)
        self.declare_parameter('prefer_neutral_pitch_for_vertical', True)
        self.declare_parameter('neutral_pitch_deg', 0.0)

        # Joint limits (deg) for safety clamps
        self.declare_parameter('limit_rotation_deg', 110.0)
        self.declare_parameter('limit_wrist_roll_deg', 160.0)
        self.declare_parameter('limit_wrist_pitch_deg', 95.0)

        # Resolve parameters
        self._input_pose_topic = self.get_parameter('input_pose_topic').get_parameter_value().string_value
        self._joint_command_topic = self.get_parameter('joint_command_topic').get_parameter_value().string_value
        self._joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self._reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self._x_offset = float(self.get_parameter('x_offset').value)
        self._z_offset = float(self.get_parameter('z_offset').value)
        self._pan_gain = float(self.get_parameter('pan_gain_deg_per_m').value)
        self._roll_scale = float(self.get_parameter('roll_scale').value)
        self._pitch_scale = float(self.get_parameter('pitch_scale').value)
        self._pitch_bias = float(self.get_parameter('pitch_bias_deg').value)
        l1 = float(self.get_parameter('link1_length').value)
        l2 = float(self.get_parameter('link2_length').value)
        ctrl_rate = float(self.get_parameter('control_rate_hz').value)
        self._vel_gain = float(self.get_parameter('velocity_gain').value)
        self._max_vel = math.radians(float(self.get_parameter('max_velocity_deg_s').value))
        self._rot_lim = abs(float(self.get_parameter('limit_rotation_deg').value))
        self._wroll_lim = abs(float(self.get_parameter('limit_wrist_roll_deg').value))
        self._wpitch_lim = abs(float(self.get_parameter('limit_wrist_pitch_deg').value))
        self._pan_invert = bool(self.get_parameter('pan_invert').value)
        self._require_pose = bool(self.get_parameter('require_pose_to_activate').value)
        self._pose_timeout = float(self.get_parameter('pose_timeout').value)
        self._init_from_js = bool(self.get_parameter('initialize_targets_from_joint_state').value)
        self._elbow_up = bool(self.get_parameter('elbow_up').value)
        self._lift_transform_intuitive = bool(self.get_parameter('lift_transform_intuitive').value)
        self._prefer_elbow_for_vertical = bool(self.get_parameter('prefer_elbow_for_vertical').value)
        self._prefer_neutral_pitch_for_vertical = bool(self.get_parameter('prefer_neutral_pitch_for_vertical').value)
        self._neutral_pitch_deg = float(self.get_parameter('neutral_pitch_deg').value)

        # Kinematics
        self.kinematics = PlanarKinematics(l1=l1, l2=l2)

        # Joint name mapping to URDF
        self._joint_order: List[str] = [
            'Rotation',      # shoulder pan
            'Pitch',         # shoulder lift
            'Elbow',         # elbow flex
            'Wrist_Pitch',   # wrist pitch
            'Wrist_Roll',    # wrist roll
        ]

        # Targets (deg) and current joint states (rad)
        self._target_deg: Dict[str, float] = {name: 0.0 for name in self._joint_order}
        self._current_rad: Dict[str, float] = {}
        self._active: bool = False
        self._last_pose_stamp = None

        qos = QoSProfile(depth=20)
        self.create_subscription(PoseStamped, self._input_pose_topic, self._on_pose, qos)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, qos)
        self._pub = self.create_publisher(JointJog, self._joint_command_topic, qos)

        # Timer for velocity control loop
        self._timer = self.create_timer(1.0 / max(1.0, ctrl_rate), self._control_step)

        self.get_logger().info(
            f"Custom IK active | in: {self._input_pose_topic} → out: {self._joint_command_topic} | frame: {self._reference_frame}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._current_rad[name] = float(pos)

        # Optionally initialize targets to current joint positions to avoid "moving by itself"
        if self._init_from_js and not self._active and self._require_pose:
            for jn in self._joint_order:
                cur = self._current_rad.get(jn)
                if cur is not None:
                    self._target_deg[jn] = math.degrees(cur)

    def _on_pose(self, msg: PoseStamped) -> None:
        # Extract base-frame pose
        px = float(msg.pose.position.x)
        py = float(msg.pose.position.y)
        pz = float(msg.pose.position.z)

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)

        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion([qx, qy, qz, qw])
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)

        # 1) shoulder pan from lateral Y
        pan_input = -py * self._pan_gain if self._pan_invert else py * self._pan_gain
        rotation_target = clamp(pan_input, -self._rot_lim, self._rot_lim)

        # 2) planar IK on X-Z plane (with offsets)
        x_plan = px - self._x_offset
        z_plan = pz - self._z_offset
        try:
            # Compute both branches; select per preference and current motion intent
            sol_down = self.kinematics.inverse_kinematics(
                x_plan, z_plan, elbow_up=False, lift_transform_intuitive=self._lift_transform_intuitive
            )
            sol_up = self.kinematics.inverse_kinematics(
                x_plan, z_plan, elbow_up=True, lift_transform_intuitive=self._lift_transform_intuitive
            )

            # Default selection from parameter
            pitch_deg_target, elbow_deg_target = sol_up if self._elbow_up else sol_down

            # Optional adaptive selection: for vertical raise/lower, prefer keeping shoulder intuitive
            if self._prefer_elbow_for_vertical or self._prefer_neutral_pitch_for_vertical:
                cur_pitch_deg = math.degrees(self._current_rad.get('Pitch', 0.0)) if self._current_rad else 0.0
                prev_z = getattr(self, '_prev_cmd_z', None)
                dz = None if prev_z is None else (pz - prev_z)

                def better_for_raising(candidate_a, candidate_b):
                    # If requested, prefer solution that moves Pitch toward a neutral target (usually 0 deg)
                    if self._prefer_neutral_pitch_for_vertical:
                        a_err = abs(candidate_a[0] - self._neutral_pitch_deg)
                        b_err = abs(candidate_b[0] - self._neutral_pitch_deg)
                        if a_err != b_err:
                            return candidate_a if a_err < b_err else candidate_b
                    # Otherwise prefer the one that increases shoulder (Pitch) when z increases
                    a_pitch, b_pitch = candidate_a[0], candidate_b[0]
                    if a_pitch >= cur_pitch_deg and b_pitch < cur_pitch_deg:
                        return candidate_a
                    if b_pitch >= cur_pitch_deg and a_pitch < cur_pitch_deg:
                        return candidate_b
                    # Otherwise prefer larger pitch to keep arm upright-ish
                    return candidate_a if a_pitch >= b_pitch else candidate_b

                def better_for_lowering(candidate_a, candidate_b):
                    # When lowering, optionally still prefer neutrality if requested
                    if self._prefer_neutral_pitch_for_vertical:
                        a_err = abs(candidate_a[0] - self._neutral_pitch_deg)
                        b_err = abs(candidate_b[0] - self._neutral_pitch_deg)
                        if a_err != b_err:
                            return candidate_a if a_err < b_err else candidate_b
                    # Default: prefer the one that decreases shoulder when z decreases
                    a_pitch, b_pitch = candidate_a[0], candidate_b[0]
                    if a_pitch <= cur_pitch_deg and b_pitch > cur_pitch_deg:
                        return candidate_a
                    if b_pitch <= cur_pitch_deg and a_pitch > cur_pitch_deg:
                        return candidate_b
                    # Otherwise prefer smaller pitch to fold down
                    return candidate_a if a_pitch <= b_pitch else candidate_b

                if dz is not None:
                    if dz > 1e-6:
                        chosen = better_for_raising(sol_up, sol_down)
                    elif dz < -1e-6:
                        chosen = better_for_lowering(sol_up, sol_down)
                    else:
                        chosen = sol_up if self._elbow_up else sol_down
                    pitch_deg_target, elbow_deg_target = chosen

                # Track last commanded Z for next step
                self._prev_cmd_z = pz

        except Exception as exc:  # pragma: no cover - safety
            self.get_logger().warn(f"IK failed: {exc}")
            return

        # 3) wrist pitch coupling: keep EE pitch
        wrist_pitch_target = -(pitch_deg_target + elbow_deg_target) + (self._pitch_scale * pitch_deg) + self._pitch_bias
        wrist_pitch_target = clamp(wrist_pitch_target, -self._wpitch_lim, self._wpitch_lim)

        # 4) wrist roll follows roll
        wrist_roll_target = clamp(self._roll_scale * roll_deg, -self._wroll_lim, self._wroll_lim)


        # Update targets
        self._target_deg['Rotation'] = rotation_target
        self._target_deg['Pitch'] = pitch_deg_target
        self._target_deg['Elbow'] = elbow_deg_target
        self._target_deg['Wrist_Pitch'] = wrist_pitch_target
        self._target_deg['Wrist_Roll'] = wrist_roll_target
        self._active = True
        try:
            self._last_pose_stamp = self.get_clock().now()
        except Exception:
            self._last_pose_stamp = None

    def _control_step(self) -> None:
        # Need current joint states to compute velocity commands
        if not self._current_rad:
            return

        # Require a pose before activating, and timeout if pose stream stops
        if self._require_pose:
            if not self._active:
                return
            if self._pose_timeout > 0.0 and self._last_pose_stamp is not None:
                now = self.get_clock().now()
                if (now - self._last_pose_stamp).nanoseconds > int(self._pose_timeout * 1e9):
                    # Stop sending commands; Servo will halt due to timeout
                    self._active = False
                    return

        # Compute velocity commands towards targets
        names: List[str] = []
        vels: List[float] = []
        for name in self._joint_order:
            target_deg = self._target_deg.get(name, 0.0)
            target_rad = math.radians(target_deg)
            current_rad = self._current_rad.get(name)
            if current_rad is None:
                # Joint not reported yet
                continue
            error = target_rad - current_rad
            vel = clamp(self._vel_gain * error, -self._max_vel, self._max_vel)
            names.append(name)
            vels.append(vel)

        if not names:
            return

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._reference_frame
        msg.joint_names = names
        msg.velocities = vels
        # Optional: duration serves as the expected update period
        try:
            msg.duration = Duration(seconds=1.0 / 50.0).to_msg()
        except Exception:
            pass
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CustomIKControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
