#!/usr/bin/env python3
"""Lecabot-style direct joint controller for SO-ARM101.

This node bypasses MoveIt Servo and drives the ros2_control JointTrajectoryController
directly from streamed end-effector poses. The goal is to get a very responsive,
“arcade-style” teleop similar to the phospho LeCabot/Phosphobot demos.

WARNING: This controller applies minimal safety checks. Use in Isaac Sim first.
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401


def clamp(value: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, value))


@dataclass
class PlanarKinematics:
  """Very simple 2-link planar IK for shoulder Pitch / Elbow.

  This is an approximate model tailored for SO-ARM101 in the vertical X–Z plane.
  It is intentionally simple and meant for fast teleop, not precise planning.
  """

  l1: float = 0.11257
  l2: float = 0.1349

  def inverse(self, x: float, z: float, elbow_up: bool = False) -> tuple[float, float]:
    """Return (pitch_rad, elbow_rad) that reaches (x,z) as best as possible."""
    r = math.hypot(x, z)
    r_max = self.l1 + self.l2
    if r > r_max and r > 1e-6:
      scale = r_max / r
      x *= scale
      z *= scale
      r = r_max

    r_min = abs(self.l1 - self.l2)
    if 0.0 < r < r_min:
      scale = r_min / r
      x *= scale
      z *= scale
      r = r_min

    # Law of cosines
    cos_elbow = (self.l1 * self.l1 + self.l2 * self.l2 - r * r) / (2.0 * self.l1 * self.l2)
    cos_elbow = clamp(cos_elbow, -1.0, 1.0)
    base_angle = math.acos(cos_elbow)
    # Choose elbow configuration
    elbow = base_angle if elbow_up else -base_angle

    # Shoulder
    beta = math.atan2(z, x)
    gamma = math.atan2(self.l2 * math.sin(elbow), self.l1 + self.l2 * math.cos(elbow))
    pitch = beta - gamma

    return pitch, elbow


class LecabotDirectControllerNode(Node):
  """Direct joint controller that converts Pose → JointTrajectory."""

  def __init__(self) -> None:
    super().__init__("so_arm_lecabot_direct_controller")

    # Topics / frames
    self.declare_parameter("input_pose_topic", "/so_arm/pose_cmd")
    self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
    self.declare_parameter("reference_frame", "base")
    self.declare_parameter("joint_state_topic", "/joint_states")
    # If true, treat incoming pose coordinates as raw web axes and apply the
    # same mapping used in websocket_pose_bridge_node:
    #   base_x = -web_z
    #   base_y = -web_x
    #   base_z =  web_y
    self.declare_parameter("input_is_web_axes", False)

    # MoveIt IK configuration (기본값: 사용 안 함, 필요할 때만 켠다)
    self.declare_parameter("use_moveit_ik", False)
    self.declare_parameter("group_name", "arm")
    self.declare_parameter("ik_link_name", "gripper")
    self.declare_parameter("ik_timeout", 0.2)
    self.declare_parameter("end_effector_frame", "gripper")

    # Simple geometric parameters (tunable offsets)
    # Approx shoulder origin offsets from base (m)
    self.declare_parameter("x_offset", 0.06)  # forward/back offset
    self.declare_parameter("z_offset", 0.11)  # height offset
    # Further reduce base pan sensitivity (used only in planar/heuristic mode)
    self.declare_parameter("pan_gain_deg_per_m", 60.0)
    self.declare_parameter("pan_limit_deg", 110.0)

    # HW joint command topic (JointState) for direct Feetech bridge
    self.declare_parameter("hw_joint_command_topic", "/so_arm/hw_joint_command")

    # Heuristic joint-space controller gains (deg / meter)
    # - forward/back (웹 x): 먼저 Pitch, 그 다음 Elbow
    #   → X축 감도를 키우기 위해 기본 gain 을 크게 설정
    # - up/down       (웹 y): Pitch 를 0 쪽으로, 이후 Elbow (더 강하게)
    self.declare_parameter("forward_pitch_gain_deg_per_m", 800.0)
    self.declare_parameter("forward_elbow_gain_deg_per_m", -480.0)
    self.declare_parameter("up_pitch_gain_deg_per_m", 400.0)
    self.declare_parameter("up_elbow_gain_deg_per_m", -320.0)
    # Pitch 조건용 한계값
    #  - pitch_forward_limit_deg : "앞으로" 펼 때 Pitch 를 이 각도까지 먼저 사용
    #  - pitch_zero_band_deg     : "위로" 보낼 때 Pitch 가 0 근처라고 보는 범위
    self.declare_parameter("pitch_forward_limit_deg", 80.0)
    self.declare_parameter("pitch_zero_band_deg", 5.0)

    # Wrist behavior (기본: Elbow에 종속, 위쪽으로 들도록 bias)
    self.declare_parameter("wrist_follow_elbow", True)
    self.declare_parameter("wrist_pitch_bias_deg", -10.0)
    self.declare_parameter("wrist_pitch_limit_deg", 95.0)
    self.declare_parameter("wrist_roll_limit_deg", 160.0)
    self.declare_parameter("wrist_follow_scale", 1.0)  # Elbow 추종 비율 (1.0 = 완전 추종)

    # Direction 기반 스텝: 거리에 덜 민감하게 만들기 위한 고정 스텝(미터)
    self.declare_parameter("direction_step_m", 0.05)
    self.declare_parameter("direction_deadzone_m", 0.005)

    # Joint smoothing / rate limit (deg per update)
    # 점프를 줄이기 위해 스텝을 약간 낮추고, 데드밴드는 작게 유지
    self.declare_parameter("max_joint_step_deg", 6.5)
    self.declare_parameter("joint_deadband_deg", 0.02)

    # Position gain relative to initial pose (1.0 → 그대로, <1.0 → 덜 민감)
    self.declare_parameter("position_gain", 1.0)

    # Trajectory timing (longer horizon → 느린 움직임)
    self.declare_parameter("time_horizon", 0.12)  # seconds for each step

    input_topic = (
        self.get_parameter("input_pose_topic").get_parameter_value().string_value
    )
    trajectory_topic = (
        self.get_parameter("trajectory_topic").get_parameter_value().string_value
    )
    joint_state_topic = (
        self.get_parameter("joint_state_topic").get_parameter_value().string_value
    )
    self._reference_frame = (
        self.get_parameter("reference_frame").get_parameter_value().string_value
    )

    self._x_offset = float(self.get_parameter("x_offset").value)
    self._z_offset = float(self.get_parameter("z_offset").value)
    self._pan_gain = float(self.get_parameter("pan_gain_deg_per_m").value)
    self._pan_limit = abs(float(self.get_parameter("pan_limit_deg").value))
    # gains in rad / meter for heuristic controller
    self._fwd_pitch_gain = math.radians(
        float(self.get_parameter("forward_pitch_gain_deg_per_m").value)
    )
    self._fwd_elbow_gain = math.radians(
        float(self.get_parameter("forward_elbow_gain_deg_per_m").value)
    )
    self._up_pitch_gain = math.radians(
        float(self.get_parameter("up_pitch_gain_deg_per_m").value)
    )
    self._up_elbow_gain = math.radians(
        float(self.get_parameter("up_elbow_gain_deg_per_m").value)
    )
    self._pitch_forward_limit = math.radians(
        float(self.get_parameter("pitch_forward_limit_deg").value)
    )
    self._pitch_zero_band = math.radians(
        float(self.get_parameter("pitch_zero_band_deg").value)
    )
    self._wrist_follow_elbow = bool(self.get_parameter("wrist_follow_elbow").value)
    self._wrist_pitch_bias = float(self.get_parameter("wrist_pitch_bias_deg").value)
    self._wrist_pitch_limit = abs(float(self.get_parameter("wrist_pitch_limit_deg").value))
    self._wrist_roll_limit = abs(float(self.get_parameter("wrist_roll_limit_deg").value))
    self._wrist_follow_scale = float(self.get_parameter("wrist_follow_scale").value)
    self._direction_step = float(self.get_parameter("direction_step_m").value)
    self._direction_deadzone = float(self.get_parameter("direction_deadzone_m").value)
    self._time_horizon = max(0.02, float(self.get_parameter("time_horizon").value))
    self._input_is_web_axes = bool(self.get_parameter("input_is_web_axes").value)
    self._use_moveit_ik = bool(self.get_parameter("use_moveit_ik").value)
    self._hw_joint_cmd_topic = (
        self.get_parameter("hw_joint_command_topic").get_parameter_value().string_value
    )

    self._group_name = (
        self.get_parameter("group_name").get_parameter_value().string_value
    )
    self._ik_link_name = (
        self.get_parameter("ik_link_name").get_parameter_value().string_value
    )
    self._ik_timeout = float(self.get_parameter("ik_timeout").value)
    self._ee_frame = (
        self.get_parameter("end_effector_frame").get_parameter_value().string_value
    )

    max_step_deg = abs(float(self.get_parameter("max_joint_step_deg").value))
    deadband_deg = abs(float(self.get_parameter("joint_deadband_deg").value))
    self._max_joint_step = math.radians(max(0.1, max_step_deg))
    self._joint_deadband = math.radians(deadband_deg)
    self._position_gain = float(self.get_parameter("position_gain").value)

    self._kin = PlanarKinematics()

    # Joint ordering must match ros2_controllers.yaml arm_controller.joints
    self._joint_names: List[str] = [
        "Rotation",
        "Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
    ]

    # Last commanded / measured joint positions (rad) for smoothing
    self._last_positions: Optional[List[float]] = None
    self._current_positions: Optional[List[float]] = None

    # Initial EE pose (for relative offset if needed)
    self._origin_position: Optional[tuple[float, float, float]] = None

    # MoveIt IK client and joint_state subscriber
    self._ik_client = self.create_client(GetPositionIK, "compute_ik")
    # TF listener for EE 기준 delta 계산
    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

    qos = QoSProfile(depth=10)
    self.create_subscription(PoseStamped, input_topic, self._on_pose, qos)
    self.create_subscription(JointState, joint_state_topic, self._on_joint_state, qos)
    self._traj_pub = self.create_publisher(JointTrajectory, trajectory_topic, qos)
    self._hw_js_pub = self.create_publisher(JointState, self._hw_joint_cmd_topic, qos)

    self.get_logger().info(
        "LeCabot-style direct controller active | "
        f"in: {input_topic} → out: {trajectory_topic} | "
        f"input_is_web_axes={self._input_is_web_axes} | "
        f"use_moveit_ik={self._use_moveit_ik}"
    )

  # -------------------- callbacks --------------------

  def _on_joint_state(self, msg: JointState) -> None:
    """Cache current joint positions for smoothing and IK seeding."""
    name_to_pos: Dict[str, float] = dict(zip(msg.name, msg.position))
    positions: List[float] = []
    for joint in self._joint_names:
      if joint not in name_to_pos:
        return
      positions.append(float(name_to_pos[joint]))
    self._current_positions = positions

  def _on_pose(self, msg: PoseStamped) -> None:
    """Convert incoming pose to joint targets via MoveIt IK (or fallback planar IK)."""
    # Position mapping
    x = float(msg.pose.position.x)
    y = float(msg.pose.position.y)
    z = float(msg.pose.position.z)

    if self._input_is_web_axes:
      # web → base
      web_x, web_y, web_z = x, y, z
      base_x = -web_z
      base_y = -web_x
      base_z = web_y
    else:
      base_x, base_y, base_z = x, y, z

    # Record initial EE pose for relative offset
    if self._origin_position is None:
      self._origin_position = (base_x, base_y, base_z)
    else:
      if not math.isclose(self._position_gain, 1.0):
        ox, oy, oz = self._origin_position
        dx = base_x - ox
        dy = base_y - oy
        dz = base_z - oz
        base_x = ox + dx * self._position_gain
        base_y = oy + dy * self._position_gain
        base_z = oz + dz * self._position_gain

    if self._use_moveit_ik:
      self._handle_pose_with_moveit_ik(msg, base_x, base_y, base_z)
    else:
      self._handle_pose_with_planar_ik(base_x, base_y, base_z)

  # -------------------- MoveIt IK path --------------------

  def _handle_pose_with_moveit_ik(
      self,
      incoming_pose: PoseStamped,
      base_x: float,
      base_y: float,
      base_z: float,
  ) -> None:
    if self._current_positions is None:
      # 아직 joint_state 를 못 받은 경우에는 MoveIt IK 대신 기존 평면 IK 로 동작
      self.get_logger().warn_once(
          "No joint_state yet; falling back to planar IK."
      )
      self._handle_pose_with_planar_ik(base_x, base_y, base_z)
      return

    if not self._ik_client.wait_for_service(timeout_sec=0.0):
      self.get_logger().warn_once(
          "compute_ik service not available; falling back to planar IK."
      )
      self._handle_pose_with_planar_ik(base_x, base_y, base_z)
      return

    target_pose = PoseStamped()
    target_pose.header = incoming_pose.header
    if not target_pose.header.frame_id:
      target_pose.header.frame_id = self._reference_frame
    target_pose.pose.position.x = base_x
    target_pose.pose.position.y = base_y
    target_pose.pose.position.z = base_z
    target_pose.pose.orientation = incoming_pose.pose.orientation

    req = GetPositionIK.Request()
    req.ik_request.group_name = self._group_name
    req.ik_request.ik_link_name = self._ik_link_name
    req.ik_request.pose_stamped = target_pose
    req.ik_request.timeout = Duration(seconds=self._ik_timeout).to_msg()

    # Seed with current joint state for smoother solutions
    js = JointState()
    js.name = list(self._joint_names)
    js.position = list(self._current_positions)
    req.ik_request.robot_state.joint_state = js

    future = self._ik_client.call_async(req)
    future.add_done_callback(self._on_ik_result)

  def _on_ik_result(self, future) -> None:
    try:
      res = future.result()
    except Exception as exc:  # pragma: no cover
      self.get_logger().warn(f"IK service call failed: {exc}")
      return

    if res.error_code.val != res.error_code.SUCCESS:
      self.get_logger().debug(
          f"IK failed with error code {res.error_code.val}; skipping command."
      )
      return

    solution_js = res.solution.joint_state
    if not solution_js.name:
      self.get_logger().warn("IK solution has empty joint_state; skipping.")
      return

    name_to_pos: Dict[str, float] = {
        n: p for n, p in zip(solution_js.name, solution_js.position)
    }
    target_positions: List[float] = []
    for joint in self._joint_names:
      if joint not in name_to_pos:
        self.get_logger().warn_once(
            f"IK solution missing joint '{joint}'; using 0.0."
        )
        target_positions.append(0.0)
      else:
        target_positions.append(float(name_to_pos[joint]))

    base_positions = self._current_positions or self._last_positions
    if base_positions is None or len(base_positions) != len(target_positions):
      smoothed = target_positions
    else:
      smoothed: List[float] = []
      for current, target in zip(base_positions, target_positions):
        delta = target - current
        if abs(delta) < self._joint_deadband:
          smoothed.append(current)
        else:
          if abs(delta) > self._max_joint_step:
            delta = math.copysign(self._max_joint_step, delta)
          smoothed.append(current + delta)

    self._last_positions = list(smoothed)
    self._publish_trajectory(smoothed)

  # -------------------- Fallback planar IK path --------------------

  def _handle_pose_with_planar_ik(
      self,
      base_x: float,
      base_y: float,
      base_z: float,
  ) -> None:
    """Heuristic joint-space controller (no geometric IK).

    아이디어:
    - 웹 포즈의 변화량(dy, dz)을 보고
      - 앞/뒤(dy 우세)  → Pitch 를 먼저 움직이고, 이후 Elbow 를 보조로 사용
      - 위/아래(dz 우세) → Pitch 를 0 쪽으로 보정하고, 이후 Elbow 로 높이를 맞춤
    - Wrist_Pitch 는 Pitch 와 거의 동기화시켜 손목만 따로 꺾이는 느낌을 줄인다.
    """

    # 현재 관절 상태 기준으로 동작 (없으면 마지막 명령 또는 0)
    if self._current_positions is not None:
      pan, pitch, elbow, wrist_pitch, wrist_roll = self._current_positions
    elif self._last_positions is not None:
      pan, pitch, elbow, wrist_pitch, wrist_roll = self._last_positions
    else:
      pan = pitch = elbow = wrist_pitch = wrist_roll = 0.0

    # 기준 포즈(origin) 대비 이번 명령의 변화량 계산 (EE 실제 위치 기준)
    ee_y = ee_z = None
    try:
      tf = self._tf_buffer.lookup_transform(
          self._reference_frame, self._ee_frame, rclpy.time.Time()
      )
      ee_y = tf.transform.translation.y
      ee_z = tf.transform.translation.z
    except Exception:
      pass
    if ee_y is not None and ee_z is not None:
      dy_raw = base_y - ee_y
      dz_raw = base_z - ee_z
    else:
      # fallback: origin 기반
      if self._origin_position is not None:
        _, origin_y, origin_z = self._origin_position
      else:
        origin_y = base_y
        origin_z = base_z
      dy_raw = base_y - origin_y
      dz_raw = base_z - origin_z

    # 거리보다는 방향 위주: 데드존을 넘으면 고정 스텝 크기만 사용
    dy = 0.0
    dz = 0.0
    if abs(dy_raw) > self._direction_deadzone:
      dy = math.copysign(self._direction_step, dy_raw)
    if abs(dz_raw) > self._direction_deadzone:
      dz = math.copysign(self._direction_step, dz_raw)

    # Left/right(웹 z) → Rotation 은 기존 방식 그대로 사용 (절대 위치 기반)
    py = -base_x
    pan_deg = clamp(py * self._pan_gain, -self._pan_limit, self._pan_limit)
    pan = math.radians(pan_deg)

    # 어느 축을 더 많이 움직였는지에 따라 모드 선택 (고정 스텝 적용 이전의 raw 값으로 비교)
    # - |dy_raw| > |dz_raw| : 앞/뒤 모드 → Pitch/Elbow를 함께 이동
    # - 그 외에는 위/아래 모드 → Pitch/Elbow를 함께 이동
    if abs(dy_raw) > abs(dz_raw):
      # Forward / Backward (웹 x): Pitch, Elbow 동시 반영
      forward = -dy  # = Δweb_x
      pitch += self._fwd_pitch_gain * forward
      elbow += self._fwd_elbow_gain * forward
      pitch = clamp(pitch, -self._pitch_forward_limit, self._pitch_forward_limit)
    else:
      # Up / Down (웹 y): Pitch, Elbow 동시 반영
      up = dz  # = Δweb_y
      pitch += self._up_pitch_gain * up
      # Z 이동은 Elbow 비중을 키워서 높이 조절을 Elbow 중심으로
      elbow += (self._up_elbow_gain * 2.0) * up
      # Pitch는 위아래에서는 0(수평) 이상으로는 가지 않게 제한
      if pitch > 0.0:
        pitch = 0.0

    # Wrist_Pitch 를 Elbow 에 종속: wrist_pitch = -elbow + bias
    if self._wrist_follow_elbow:
      wrist_pitch = -self._wrist_follow_scale * elbow + math.radians(self._wrist_pitch_bias)

    # 안전 범위 클램프 (URDF 리밋 기준)
    limit = math.radians(self._wrist_pitch_limit)
    wrist_pitch = clamp(wrist_pitch, -limit, limit)
    wrist_roll = clamp(
        wrist_roll,
        -math.radians(self._wrist_roll_limit),
        math.radians(self._wrist_roll_limit),
    )

    positions = [pan, pitch, elbow, wrist_pitch, wrist_roll]

    # 기존과 동일한 joint smoothing 적용
    if self._last_positions is None:
      self._last_positions = list(positions)
      smoothed = positions
    else:
      smoothed = []
      for last, target in zip(self._last_positions, positions):
        delta = target - last
        if abs(delta) < self._joint_deadband:
          smoothed.append(last)
        else:
          if abs(delta) > self._max_joint_step:
            delta = math.copysign(self._max_joint_step, delta)
          smoothed.append(last + delta)
      self._last_positions = smoothed

    self._publish_trajectory(smoothed)

  # -------------------- Trajectory publisher --------------------

  def _publish_trajectory(self, positions: List[float]) -> None:
    traj = JointTrajectory()
    traj.header.stamp = self.get_clock().now().to_msg()
    traj.joint_names = list(self._joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = int(self._time_horizon)
    point.time_from_start.nanosec = int((self._time_horizon % 1.0) * 1e9)
    traj.points.append(point)
    self._traj_pub.publish(traj)

    # Publish JointState for Feetech bridge (same ordering)
    js = JointState()
    js.header = traj.header
    js.name = list(self._joint_names)
    js.position = list(positions)
    self._hw_js_pub.publish(js)


def main() -> None:
  rclpy.init()
  node = LecabotDirectControllerNode()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
