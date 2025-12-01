#!/usr/bin/env python3
"""Bridge pose commands between a WebSocket stream and ROS 2 topics."""

from __future__ import annotations

import asyncio
import copy
import json
import math
from dataclasses import dataclass
from threading import Event, Thread
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointJog
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.task import Future
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from tf_transformations import (
    euler_from_quaternion,
    quaternion_about_axis,
    quaternion_conjugate,
    quaternion_from_euler,
    quaternion_multiply,
    quaternion_slerp,
)

try:
    import websockets
    from websockets.client import WebSocketClientProtocol
except ImportError as exc:  # pragma: no cover - dependency check
    raise RuntimeError("Please install the 'websockets' package to use websocket_pose_bridge_node") from exc

import tf2_ros
from tf2_ros import TransformListener


@dataclass
class PosePayload:
    frame_id: str
    position: Dict[str, float]
    orientation: Dict[str, float]


def _coerce_float(value: Any, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"'{name}' must be numeric") from exc


def _parse_pose_message(message: Any, default_frame: str) -> PosePayload:
    """Parse a variety of pose JSON formats into PosePayload.

    Accepted forms:
    - {"type":"pose"|"type_pose", "position":{"x","y","z"}, "orientation":{"x","y","z","w"}}
    - Same as above without "type"
    - Orientation as {"rx","ry","rz","rw"}
    - position/orientation as arrays: position:[x,y,z], orientation:[x,y,z,w]
    - Bundle wrapper: {"type":"bundle", "data": <any accepted pose>, ...} or {"payload": <pose>}
    - Flat keys: {"x","y","z","rx","ry","rz","rw"}
    """
    def _as_pose_dict(obj: Any) -> Optional[dict]:
        if not isinstance(obj, dict):
            return None
        # Unwrap common bundle wrappers
        t = obj.get("type")
        if t == "bundle":
            inner = obj.get("data") or obj.get("payload") or obj.get("pose")
            return _as_pose_dict(inner)
        # Flat layout
        keys = set(obj.keys())
        if {"x", "y", "z", "rx", "ry", "rz", "rw"}.issubset(keys):
            return {
                "position": {"x": obj["x"], "y": obj["y"], "z": obj["z"]},
                "orientation": {"x": obj["rx"], "y": obj["ry"], "z": obj["rz"], "w": obj["rw"]},
            }
        # position/orientation as dicts
        pos = obj.get("position")
        ori = obj.get("orientation")
        if isinstance(pos, dict) and isinstance(ori, dict):
            # Accept rx/ry/rz/rw as well
            if {"rx", "ry", "rz", "rw"}.issubset(ori.keys()):
                ori = {"x": ori["rx"], "y": ori["ry"], "z": ori["rz"], "w": ori["rw"]}
            return {"position": pos, "orientation": ori, "frame_id": obj.get("frame_id")}
        # position/orientation as arrays
        if isinstance(pos, (list, tuple)) and isinstance(ori, (list, tuple)) and len(pos) == 3 and len(ori) == 4:
            return {
                "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
                "orientation": {"x": ori[0], "y": ori[1], "z": ori[2], "w": ori[3]},
                "frame_id": obj.get("frame_id"),
            }
        # Unknown
        return None

    pose_obj = _as_pose_dict(message)
    if pose_obj is None:
        raise ValueError("Unsupported pose message format.")

    position = pose_obj.get("position")
    orientation = pose_obj.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError("Message must contain 'position' and 'orientation'.")

    pose_position = {axis: _coerce_float(position.get(axis), f"position.{axis}") for axis in ("x", "y", "z")}
    pose_orientation = {axis: _coerce_float(orientation.get(axis), f"orientation.{axis}") for axis in ("x", "y", "z", "w")}

    frame_id = str(pose_obj.get("frame_id") or message.get("frame_id") or default_frame)
    return PosePayload(frame_id=frame_id, position=pose_position, orientation=pose_orientation)


class WebsocketPoseBridge(Node):
    """ROS 2 node keeping a WebSocket connection in sync with pose commands."""

    def __init__(self) -> None:
        super().__init__("websocket_pose_bridge")

        self.declare_parameter("websocket_url", "ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle") 
        self.declare_parameter("publish_topic", "/so_arm/pose_cmd")
        self.declare_parameter("reference_frame", "base")
        self.declare_parameter("end_effector_frame", "gripper")
        self.declare_parameter("ack_enabled", True)
        self.declare_parameter("reconnect_delay", 3.0)
        self.declare_parameter("transform_quaternion", [0.5, 0.5, -0.5, -0.5])  # w, x, y, z
        # If true, treat web Z-rotation as yaw and ignore web roll/pitch
        # Disabled by default; previous experiment only.
        self.declare_parameter("use_web_z_as_yaw", False)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("status_publish_period", 0.2)
        # WebSocket keepalive settings (set interval<=0 to disable pings)
        self.declare_parameter("ws_ping_interval", 20.0)
        self.declare_parameter("ws_ping_timeout", 20.0)
        self.declare_parameter("tf_lookup_timeout", 0.5)
        self.declare_parameter("tf_retry_period", 1.0)
        self.declare_parameter("feedback_topic", "/so_arm/pose_state")
        # Moderately responsive defaults (roll back from aggressive tuning)
        self.declare_parameter("position_smoothing_alpha", 0.99)
        self.declare_parameter("orientation_smoothing_alpha", 0.97)
        self.declare_parameter("max_position_step", 1.0)
        self.declare_parameter("max_orientation_step", 1.0)
        self.declare_parameter("enable_status_recovery", True)
        # Optional decoupling of position / orientation updates:
        # - pure position change  → translate only (keep last orientation)
        # - pure orientation change → rotate only (keep last position)
        # Disabled by default; kept for experimentation only.
        self.declare_parameter("decouple_pos_orientation", False)
        self.declare_parameter("position_change_threshold", 1e-4)
        self.declare_parameter("orientation_change_threshold_deg", 0.5)
        # Map web Z-axis input directly to a joint jog on the base "Rotation" joint.
        # This provides a simple way to spin the base without affecting Cartesian pose.
        self.declare_parameter("map_web_z_to_rotation_joint", True)
        self.declare_parameter("rotation_joint_name", "Rotation")
        self.declare_parameter("rotation_joint_gain", 2.0)  # rad/s per unit web-Z
        self.declare_parameter("rotation_joint_max_speed", 1.0)  # rad/s
        self.declare_parameter("rotation_joint_deadband", 1e-3)
        self.declare_parameter("rotation_joint_command_topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("recovery_pose", [0.05, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("recovery_interval", 2.0)
        self.declare_parameter("recovery_height_offset", 0.05)
        self.declare_parameter("recovery_xy_offset", 0.03)
        self.declare_parameter("recovery_yaw_offset_deg", 25.0)
        # Workspace limits: [min_x, max_x, min_y, max_y, min_z, max_z] in meters (base frame)
        # User preference:
        #   x: [-0.3, 0.3]
        #   y: [-0.25, 0.35]
        #   z: [ 0.02, 0.45]
        self.declare_parameter("position_limits", [-0.3, 0.3, -0.25, 0.35, 0.02, 0.45])
        # Do not enforce any rotational limits by default
        self.declare_parameter("orientation_limits_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("enforce_position_limits", True)
        self.declare_parameter("enforce_orientation_limits", False)

        url = self.get_parameter("websocket_url").get_parameter_value().string_value
        if not url:
            raise ValueError("Parameter 'websocket_url' must be provided.")

        topic = self.get_parameter("publish_topic").value
        self._reference_frame = str(self.get_parameter("reference_frame").value)
        self._end_effector_frame = str(self.get_parameter("end_effector_frame").value)
        self._ack_enabled = bool(self.get_parameter("ack_enabled").value)
        self._reconnect_delay = float(self.get_parameter("reconnect_delay").value)
        self._transform_quaternion = self._parse_vector_param("transform_quaternion", 4)
        self._transform_quaternion_xyzw = (
            self._transform_quaternion[1],
            self._transform_quaternion[2],
            self._transform_quaternion[3],
            self._transform_quaternion[0],
        )
        self._use_web_z_as_yaw = bool(self.get_parameter("use_web_z_as_yaw").value)
        self._initial_pose = self._parse_vector_param("initial_pose", 7)
        self._status_period = max(
            0.0,
            float(self.get_parameter("status_publish_period").value),
        )
        self._tf_timeout = max(0.1, float(self.get_parameter("tf_lookup_timeout").value))
        self._tf_retry_period = max(0.1, float(self.get_parameter("tf_retry_period").value))
        self._position_alpha = min(max(float(self.get_parameter("position_smoothing_alpha").value), 0.0), 1.0)
        self._orientation_alpha = min(
            max(float(self.get_parameter("orientation_smoothing_alpha").value), 0.0), 1.0
        )
        self._max_position_step = max(0.0, float(self.get_parameter("max_position_step").value))
        self._max_orientation_step = max(0.0, float(self.get_parameter("max_orientation_step").value))
        self._enable_recovery = bool(self.get_parameter("enable_status_recovery").value)
        self._recovery_pose = self._parse_vector_param("recovery_pose", 7)
        self._recovery_interval = max(0.5, float(self.get_parameter("recovery_interval").value))
        self._recovery_payload = self._create_payload_from_list(self._recovery_pose)
        self._recovery_height_offset = float(self.get_parameter("recovery_height_offset").value)
        self._recovery_xy_offset = float(self.get_parameter("recovery_xy_offset").value)
        self._recovery_yaw_offset = math.radians(float(self.get_parameter("recovery_yaw_offset_deg").value))
        self._pos_limits = self._parse_vector_param("position_limits", 6)
        ori_limits_deg = self._parse_vector_param("orientation_limits_deg", 3)
        self._ori_limits_rad = [
            math.radians(abs(limit)) if abs(limit) > 0.0 else 0.0 for limit in ori_limits_deg
        ]
        self._enforce_pos_limits = bool(self.get_parameter("enforce_position_limits").value)
        self._enforce_ori_limits = bool(self.get_parameter("enforce_orientation_limits").value)
        self._safe_zone_warned = False

        self._decouple_pos_orientation = bool(
            self.get_parameter("decouple_pos_orientation").value
        )
        self._pos_change_threshold = max(
            0.0, float(self.get_parameter("position_change_threshold").value)
        )
        self._ori_change_threshold = math.radians(
            max(0.0, float(self.get_parameter("orientation_change_threshold_deg").value))
        )

        self._map_web_z_to_rotation_joint = bool(
            self.get_parameter("map_web_z_to_rotation_joint").value
        )
        self._rotation_joint_name = (
            self.get_parameter("rotation_joint_name").get_parameter_value().string_value
        )
        self._rotation_joint_gain = float(self.get_parameter("rotation_joint_gain").value)
        self._rotation_joint_max_speed = abs(
            float(self.get_parameter("rotation_joint_max_speed").value)
        )
        self._rotation_joint_deadband = max(
            0.0, float(self.get_parameter("rotation_joint_deadband").value)
        )
        rotation_cmd_topic = (
            self.get_parameter("rotation_joint_command_topic").get_parameter_value().string_value
        )
        self._rotation_joint_pub = None
        if self._map_web_z_to_rotation_joint:
            self._rotation_joint_pub = self.create_publisher(
                JointJog, rotation_cmd_topic, QoSProfile(depth=10)
            )
        self._last_web_z: Optional[float] = None

        self._initial_status_message = {
            "type": "type_pose",
            "position": {
                "x": float(self._initial_pose[0]),
                "y": float(self._initial_pose[1]),
                "z": float(self._initial_pose[2]),
            },
            "orientation": {
                "x": float(self._initial_pose[3]),
                "y": float(self._initial_pose[4]),
                "z": float(self._initial_pose[5]),
                "w": float(self._initial_pose[6]),
            },
        }
        self._last_base_position: Optional[tuple[float, float, float]] = None
        self._last_base_quat: Optional[tuple[float, float, float, float]] = None
        self._send_lock: Optional[asyncio.Lock] = None
        self._pose_queue: Optional[asyncio.Queue[PosePayload]] = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._tf_timer = self.create_timer(self._tf_retry_period, self._attempt_initial_pose_from_tf)
        self._attempt_initial_pose_from_tf()
        self._last_recovery_time: Optional[Time] = None
        self._pending_restart = False
        self._in_recovery = False
        self._deferred_payload: Optional[PosePayload] = None
        qos_status = QoSProfile(depth=5)
        self._status_sub = self.create_subscription(
            Int8,
            "/servo_node/status",
            self._on_servo_status,
            qos_status,
        )
        self._start_servo_client = self.create_client(Trigger, "/servo_node/start_servo")

        qos = QoSProfile(depth=10)
        self._publisher = self.create_publisher(PoseStamped, topic, qos)
        feedback_topic = str(self.get_parameter("feedback_topic").value)
        self._feedback_pub = (
            self.create_publisher(PoseStamped, feedback_topic, qos)
            if feedback_topic
            else None
        )

        self._url = url
        self._ws_ping_interval = float(self.get_parameter("ws_ping_interval").value)
        self._ws_ping_timeout = float(self.get_parameter("ws_ping_timeout").value)
        self._stop_event = Event()
        self._loop = asyncio.new_event_loop()
        self._thread = Thread(target=self._run_event_loop, name="ws-bridge", daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"Connecting to WebSocket {self._url} and publishing to {topic} (frame '{self._reference_frame}')"
        )

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._send_lock = asyncio.Lock()
        self._pose_queue = asyncio.Queue(maxsize=1)  # keep only the most recent command for low latency
        try:
            self._loop.run_until_complete(self._main_loop())
        finally:
            pending = asyncio.all_tasks(self._loop)
            for task in pending:
                task.cancel()
            self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self._loop.close()

    async def _main_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                # Allow tuning keepalive pings for unstable proxies/NATs.
                ping_interval = None if self._ws_ping_interval <= 0 else self._ws_ping_interval
                ping_timeout = None if self._ws_ping_timeout <= 0 else self._ws_ping_timeout
                async with websockets.connect(
                    self._url, ping_interval=ping_interval, ping_timeout=ping_timeout
                ) as ws:
                    self.get_logger().info("WebSocket connection established.")
                    await self._send_initial_pose(ws)
                    await self._handle_connection(ws)
            except asyncio.CancelledError:  # pragma: no cover - loop shutdown path
                break
            except Exception as exc:
                self.get_logger().warn(f"WebSocket connection error: {exc}")
                if self._reconnect_delay > 0:
                    await asyncio.sleep(self._reconnect_delay)

    async def _handle_connection(self, ws: WebSocketClientProtocol) -> None:
        status_task: Optional[asyncio.Task] = None
        processor_task: Optional[asyncio.Task] = None
        try:
            if self._pose_queue is not None:
                processor_task = asyncio.create_task(self._process_pose_queue(ws))
            if self._status_period > 0.0:
                status_task = asyncio.create_task(self._periodic_status_sender(ws))

            async for raw_message in ws:
                if isinstance(raw_message, bytes):
                    lowered_bytes = raw_message.lower()
                    if lowered_bytes in (b"ping", b"pong"):
                        self.get_logger().debug(f"Ignoring binary keepalive frame {lowered_bytes!r}.")
                        continue
                    try:
                        raw_message = raw_message.decode("utf-8")
                    except UnicodeDecodeError:
                        self.get_logger().warn("Received non-UTF8 message; ignoring.")
                        continue

                text = raw_message.strip()
                if not text:
                    self.get_logger().debug("Ignoring empty WebSocket frame.")
                    continue

                if text[0] not in ("{", "["):
                    lowered = text.lower()
                    if lowered in ("ping", "pong"):
                        self.get_logger().debug(f"Ignoring keepalive frame '{text}'.")
                    else:
                        self.get_logger().debug(f"Ignoring non-JSON WebSocket payload: '{text[:20]}'")
                    continue

                try:
                    payload = json.loads(text)
                    pose_payload = _parse_pose_message(payload, self._reference_frame)
                except ValueError as exc:
                    self.get_logger().warn(f"Ignoring malformed pose message: {exc}")
                    continue

                if self._pose_queue is None:
                    if self._in_recovery:
                        self._deferred_payload = pose_payload
                        continue
                    base_position, base_quat = self._publish_pose(pose_payload)
                    if self._ack_enabled:
                        await self._send_ack(ws, base_position, base_quat)
                else:
                    queue = self._pose_queue
                    while queue.full():
                        try:
                            queue.get_nowait()
                            queue.task_done()
                        except asyncio.QueueEmpty:
                            break
                    if self._in_recovery:
                        self._deferred_payload = pose_payload
                    else:
                        await queue.put(pose_payload)
        finally:
            if status_task is not None:
                status_task.cancel()
                try:
                    await status_task
                except asyncio.CancelledError:
                    pass
            if processor_task is not None:
                processor_task.cancel()
                try:
                    await processor_task
                except asyncio.CancelledError:
                    pass
            self.get_logger().info("WebSocket connection closed.")

    def _publish_pose(self, pose_payload: PosePayload) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose_payload.frame_id or self._reference_frame

        web_x = float(pose_payload.position["x"])
        web_y = float(pose_payload.position["y"])
        web_z = float(pose_payload.position["z"])

        px = -web_z
        py = -web_x
        pz = web_y
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz

        web_quat_xyzw = (
            pose_payload.orientation["x"],
            pose_payload.orientation["y"],
            pose_payload.orientation["z"],
            pose_payload.orientation["w"],
        )

        # Optional joint jog mapping driven directly from web input:
        if self._map_web_z_to_rotation_joint and self._rotation_joint_pub is not None:
            self._maybe_publish_rotation_joint_jog(web_z)
        # Decompose web orientation once into roll/pitch/yaw
        roll_w, pitch_w, yaw_w = euler_from_quaternion(web_quat_xyzw)
        if self._use_web_z_as_yaw:
            # Extract yaw around web Z axis and ignore web roll/pitch so that
            # “Z-rotation” on the web side maps cleanly to yaw on the robot side.
            yaw_only_web = quaternion_from_euler(0.0, 0.0, yaw_w)
            base_quat_xyzw = quaternion_multiply(self._transform_quaternion_xyzw, yaw_only_web)
        else:
            base_quat_xyzw = quaternion_multiply(
                self._transform_quaternion_xyzw, web_quat_xyzw
            )
        base_quat_xyzw = tuple(float(v) for v in base_quat_xyzw)
        base_position = (px, py, pz)

        # Optionally decouple pure position vs pure orientation changes:
        # - If only position changed → update position, keep last orientation.
        # - If only orientation changed → update orientation, keep last position.
        if (
            self._decouple_pos_orientation
            and self._last_base_position is not None
            and self._last_base_quat is not None
        ):
            dx = base_position[0] - self._last_base_position[0]
            dy = base_position[1] - self._last_base_position[1]
            dz = base_position[2] - self._last_base_position[2]
            pos_delta = math.sqrt(dx * dx + dy * dy + dz * dz)

            dot = sum(a * b for a, b in zip(self._last_base_quat, base_quat_xyzw))
            dot = max(min(dot, 1.0), -1.0)
            ori_angle = 2.0 * math.acos(dot)

            pos_changed = pos_delta > self._pos_change_threshold
            ori_changed = ori_angle > self._ori_change_threshold

            if pos_changed and not ori_changed:
                # Pure translation command: ignore small orientation change/noise.
                base_quat_xyzw = self._last_base_quat
            elif ori_changed and not pos_changed:
                # Pure rotation command: keep position fixed.
                base_position = self._last_base_position

        base_position = self._filter_position(base_position)
        base_quat_xyzw = self._filter_orientation(base_quat_xyzw)
        base_position, base_quat_xyzw = self._enforce_safe_zone(base_position, base_quat_xyzw)

        msg.pose.position.x = base_position[0]
        msg.pose.position.y = base_position[1]
        msg.pose.position.z = base_position[2]
        msg.pose.orientation.x = base_quat_xyzw[0]
        msg.pose.orientation.y = base_quat_xyzw[1]
        msg.pose.orientation.z = base_quat_xyzw[2]
        msg.pose.orientation.w = base_quat_xyzw[3]

        norm = math.sqrt(
            base_quat_xyzw[0] ** 2
            + base_quat_xyzw[1] ** 2
            + base_quat_xyzw[2] ** 2
            + base_quat_xyzw[3] ** 2
        )
        if not math.isclose(norm, 1.0, rel_tol=1e-3, abs_tol=1e-3):
            self.get_logger().warn_once("Resulting quaternion is not normalized.")

        self._publisher.publish(msg)
        base_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self._last_base_position = base_position
        self._last_base_quat = base_quat_xyzw
        return base_position, base_quat_xyzw

    def _maybe_publish_rotation_joint_jog(self, web_z: float) -> None:
        if self._rotation_joint_pub is None:
            return

        vel = self._rotation_joint_gain * web_z
        if abs(vel) < self._rotation_joint_deadband:
            self._last_web_z = web_z
            return

        if self._rotation_joint_max_speed > 0.0 and abs(vel) > self._rotation_joint_max_speed:
            vel = math.copysign(self._rotation_joint_max_speed, vel)

        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = ""
        jog.joint_names = [self._rotation_joint_name]
        jog.displacements = []
        jog.velocities = [vel]
        jog.duration = 0.2

        self._rotation_joint_pub.publish(jog)
        self._last_web_z = web_z

    async def _send_ack(
        self,
        ws: WebSocketClientProtocol,
        base_position: tuple[float, float, float],
        base_quat_xyzw: tuple[float, float, float, float],
    ) -> None:
        actual = self._lookup_current_pose()
        if actual is not None:
            base_position, base_quat_xyzw = actual
            self._last_base_position = base_position
            self._last_base_quat = base_quat_xyzw
            if self._feedback_pub is not None:
                feedback = PoseStamped()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.header.frame_id = self._reference_frame
                feedback.pose.position.x = base_position[0]
                feedback.pose.position.y = base_position[1]
                feedback.pose.position.z = base_position[2]
                feedback.pose.orientation.x = base_quat_xyzw[0]
                feedback.pose.orientation.y = base_quat_xyzw[1]
                feedback.pose.orientation.z = base_quat_xyzw[2]
                feedback.pose.orientation.w = base_quat_xyzw[3]
                self._feedback_pub.publish(feedback)
        else:
            self._last_base_position = base_position
            self._last_base_quat = base_quat_xyzw
        ack_message = self._build_ack_message(base_position, base_quat_xyzw)
        try:
            await self._send_json_message(ws, ack_message)
        except Exception as exc:
            self.get_logger().warn(f"Failed to send acknowledgement: {exc}")

    def _build_ack_message(
        self,
        base_position: tuple[float, float, float],
        base_quat_xyzw: tuple[float, float, float, float],
    ) -> Dict[str, Any]:
        web_position = {
            "x": -base_position[1],
            "y": base_position[2],
            "z": -base_position[0],
        }

        transform_conj = quaternion_conjugate(self._transform_quaternion_xyzw)
        web_quat_xyzw = quaternion_multiply(transform_conj, base_quat_xyzw)
        web_quat_xyzw = [float(v) for v in web_quat_xyzw]

        return {
            "type": "type_pose",
            "position": web_position,
            "orientation": {
                "x": web_quat_xyzw[0],
                "y": web_quat_xyzw[1],
                "z": web_quat_xyzw[2],
                "w": web_quat_xyzw[3],
            },
        }

    def _lookup_current_pose(self) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._reference_frame,
                self._end_effector_frame,
                Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except Exception:
            return None

        position = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        quat = (
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        )
        return position, quat

    def _attempt_initial_pose_from_tf(self) -> None:
        if self._tf_buffer is None:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._reference_frame,
                self._end_effector_frame,
                Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self.get_logger().debug(f"Initial TF lookup failed: {exc}")
            return
        except Exception as exc:  # pragma: no cover - unexpected errors
            self.get_logger().warn(f"Unexpected error during TF lookup: {exc}")
            return

        base_position = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        base_quat_xyzw = (
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        )

        self._last_base_position = base_position
        self._last_base_quat = base_quat_xyzw
        self._initial_status_message = self._build_ack_message(base_position, base_quat_xyzw)

        if self._tf_timer is not None:
            self._tf_timer.cancel()
            self._tf_timer = None

        self.get_logger().info(
            f"Initial pose synchronized from TF ({self._reference_frame} -> {self._end_effector_frame})."
        )

    async def _send_json_message(self, ws: WebSocketClientProtocol, message: Dict[str, Any]) -> None:
        # Send as UTF-8 encoded JSON bytes (binary frame) per project convention.
        payload = json.dumps(message).encode("utf-8")
        if self._send_lock is not None:
            async with self._send_lock:
                await ws.send(payload)
        else:
            await ws.send(payload)

    async def _process_pose_queue(self, ws: WebSocketClientProtocol) -> None:
        if self._pose_queue is None:
            return
        try:
            while True:
                payload = await self._pose_queue.get()
                try:
                    base_position, base_quat = self._publish_pose(payload)
                    if self._ack_enabled:
                        await self._send_ack(ws, base_position, base_quat)
                except Exception as exc:
                    self.get_logger().warn(f"Failed to process pose message: {exc}")
                finally:
                    self._pose_queue.task_done()
        except asyncio.CancelledError:
            raise

    async def _send_initial_pose(self, ws: WebSocketClientProtocol) -> None:
        try:
            message = self._current_status_message()
            await self._send_json_message(ws, message)
            self.get_logger().debug("Initial pose sent to WebSocket client.")
        except Exception as exc:
            self.get_logger().warn(f"Failed to send initial pose: {exc}")

    async def _periodic_status_sender(self, ws: WebSocketClientProtocol) -> None:
        try:
            while True:
                message = self._current_status_message()
                try:
                    await self._send_json_message(ws, message)
                except Exception as exc:
                    self.get_logger().debug(f"Stopping status publisher due to send error: {exc}")
                    break
                await asyncio.sleep(self._status_period)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # pragma: no cover - unexpected errors
            self.get_logger().warn(f"Status publisher encountered an error: {exc}")

    def _current_status_message(self) -> Dict[str, Any]:
        actual = self._lookup_current_pose()
        if actual is not None:
            self._last_base_position, self._last_base_quat = actual
            return self._build_ack_message(self._last_base_position, self._last_base_quat)
        if self._last_base_position is not None and self._last_base_quat is not None:
            return self._build_ack_message(self._last_base_position, self._last_base_quat)
        return copy.deepcopy(self._initial_status_message)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)
        if self._tf_timer is not None:
            self._tf_timer.cancel()
            self._tf_timer = None
        return super().destroy_node()

    def _parse_vector_param(self, name: str, size: int) -> list[float]:
        param_value = self.get_parameter(name).value
        if isinstance(param_value, (list, tuple)):
            values = [float(v) for v in param_value]
        else:
            values = [float(param_value)]

        if len(values) != size:
            raise ValueError(f"Parameter '{name}' must contain exactly {size} values.")

        return values

    # (position remap helpers removed during rollback)

    def _create_payload_from_list(self, values: list[float]) -> PosePayload:
        base_position = (float(values[0]), float(values[1]), float(values[2]))
        base_quat_xyzw = (
            float(values[3]),
            float(values[4]),
            float(values[5]),
            float(values[6]),
        )
        transform_conj = quaternion_conjugate(self._transform_quaternion_xyzw)
        web_quat_xyzw = quaternion_multiply(transform_conj, base_quat_xyzw)
        return PosePayload(
            frame_id=self._reference_frame,
            position={
                "x": -base_position[1],
                "y": base_position[2],
                "z": -base_position[0],
            },
            orientation={
                "x": float(web_quat_xyzw[0]),
                "y": float(web_quat_xyzw[1]),
                "z": float(web_quat_xyzw[2]),
                "w": float(web_quat_xyzw[3]),
            },
        )

    def _on_servo_status(self, msg: Int8) -> None:
        if not self._enable_recovery:
            return

        code = int(msg.data)
        # Auto-recover only on singularity halt (2) or joint-bound halt (5)
        if code in (2, 5):
            now = self.get_clock().now()
            if (
                self._last_recovery_time is None
                or (now - self._last_recovery_time).nanoseconds > int(self._recovery_interval * 1e9)
            ):
                self._last_recovery_time = now
                self._in_recovery = True
                self.get_logger().warn(
                    f"Servo status {code} detected; injecting recovery pose to escape singularity."
                )
                payload = self._build_recovery_payload()
                self._schedule_payload(payload, force=True)
                self._trigger_servo_restart()
        elif code == 0:
            self._pending_restart = False
            if self._in_recovery:
                self._in_recovery = False
                if self._deferred_payload is not None:
                    self.get_logger().info("Applying deferred pose command after recovery.")
                    self._schedule_payload(self._deferred_payload)
                    self._deferred_payload = None

    def _schedule_payload(self, payload: PosePayload, force: bool = False) -> None:
        if self._pose_queue is not None and self._loop.is_running():
            def enqueue() -> None:
                queue = self._pose_queue
                while queue.full():
                    try:
                        queue.get_nowait()
                        queue.task_done()
                    except asyncio.QueueEmpty:
                        break
                if not force and self._in_recovery:
                    return
                queue.put_nowait(payload)

            self._loop.call_soon_threadsafe(enqueue)
        else:
            if force or not self._in_recovery:
                self._publish_pose(payload)

    def _build_recovery_payload(self) -> PosePayload:
        pose = self._lookup_current_pose()
        if pose is None:
            if self._last_base_position is not None and self._last_base_quat is not None:
                pose = (self._last_base_position, self._last_base_quat)
            else:
                self.get_logger().warn_once("Recovery falling back to static pose.")
                return self._recovery_payload

        base_position, base_quat = pose
        offset_position = (
            base_position[0] + self._recovery_xy_offset * math.cos(self._recovery_yaw_offset),
            base_position[1] + self._recovery_xy_offset * math.sin(self._recovery_yaw_offset),
            base_position[2] + self._recovery_height_offset,
        )
        yaw_quat = quaternion_about_axis(self._recovery_yaw_offset, (0.0, 0.0, 1.0))
        offset_quat = quaternion_multiply(base_quat, yaw_quat)
        norm = math.sqrt(sum(v * v for v in offset_quat))
        if norm == 0.0:
            offset_quat = base_quat
        else:
            offset_quat = tuple(v / norm for v in offset_quat)

        return self._create_payload_from_list(
            [
                offset_position[0],
                offset_position[1],
                offset_position[2],
                offset_quat[0],
                offset_quat[1],
                offset_quat[2],
                offset_quat[3],
            ]
        )

    def _trigger_servo_restart(self) -> None:
        if self._pending_restart:
            return
        if not self._start_servo_client.wait_for_service(timeout_sec=0.0):
            return
        request = Trigger.Request()
        future = self._start_servo_client.call_async(request)
        future.add_done_callback(self._on_restart_response)
        self._pending_restart = True

    def _on_restart_response(self, future: Future) -> None:
        self._pending_restart = False
        if future.cancelled():
            self.get_logger().warn("Servo restart service call was cancelled.")
            return
        exc = future.exception()
        if exc is not None:
            self.get_logger().warn(f"Servo restart service raised an exception: {exc}")
            return
        response = future.result()
        if not response.success:
            self.get_logger().warn(f"Servo restart failed: {response.message}")
        else:
            self.get_logger().info("Servo restart requested successfully.")

    def _enforce_safe_zone(
        self,
        position: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        px, py, pz = position
        min_x, max_x, min_y, max_y, min_z, max_z = self._pos_limits

        if self._enforce_pos_limits:
            clamped_px = min(max(px, min_x), max_x)
            clamped_py = min(max(py, min_y), max_y)
            clamped_pz = min(max(pz, min_z), max_z)
        else:
            clamped_px, clamped_py, clamped_pz = px, py, pz

        roll, pitch, yaw = euler_from_quaternion(quat_xyzw)
        limits = self._ori_limits_rad

        def _clamp_angle(angle: float, limit: float) -> float:
            if limit <= 0.0:
                return angle
            if angle > limit:
                return limit
            if angle < -limit:
                return -limit
            return angle

        if self._enforce_ori_limits:
            clamped_roll = _clamp_angle(roll, limits[0])
            clamped_pitch = _clamp_angle(pitch, limits[1])
            clamped_yaw = _clamp_angle(yaw, limits[2])
        else:
            clamped_roll, clamped_pitch, clamped_yaw = roll, pitch, yaw

        pos_changed = (clamped_px, clamped_py, clamped_pz) != (px, py, pz)
        ori_changed = (clamped_roll, clamped_pitch, clamped_yaw) != (roll, pitch, yaw)

        if ori_changed:
            quat_xyzw = quaternion_from_euler(clamped_roll, clamped_pitch, clamped_yaw)

        if (pos_changed or ori_changed) and not self._safe_zone_warned:
            self.get_logger().warn(
                "Incoming pose exceeded configured safe zone; applying clamped pose."
            )
            self._safe_zone_warned = True

        return (clamped_px, clamped_py, clamped_pz), tuple(float(v) for v in quat_xyzw)

    def _filter_position(self, target: tuple[float, float, float]) -> tuple[float, float, float]:
        if self._last_base_position is None:
            return target

        last = self._last_base_position
        delta = (
            target[0] - last[0],
            target[1] - last[1],
            target[2] - last[2],
        )
        delta_norm = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)

        if self._max_position_step > 0.0 and delta_norm > self._max_position_step:
            scale = self._max_position_step / delta_norm
            target = (
                last[0] + delta[0] * scale,
                last[1] + delta[1] * scale,
                last[2] + delta[2] * scale,
            )

        alpha = self._position_alpha
        if 0.0 < alpha < 1.0:
            inv_alpha = 1.0 - alpha
            target = (
                inv_alpha * last[0] + alpha * target[0],
                inv_alpha * last[1] + alpha * target[1],
                inv_alpha * last[2] + alpha * target[2],
            )

        return target

    def _filter_orientation(self, target: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        last = self._last_base_quat
        if last is None:
            return target

        # Ensure shortest path
        dot = sum(a * b for a, b in zip(last, target))
        if dot < 0.0:
            target = tuple(-v for v in target)
            dot = -dot

        dot = max(min(dot, 1.0), -1.0)
        angle = 2.0 * math.acos(dot)

        if self._max_orientation_step > 0.0 and angle > self._max_orientation_step:
            fraction = self._max_orientation_step / angle
            target = quaternion_slerp(last, target, fraction)

        alpha = self._orientation_alpha
        if 0.0 < alpha < 1.0:
            target = quaternion_slerp(last, target, alpha)

        norm = math.sqrt(sum(v * v for v in target))
        if norm == 0.0:
            return last
        return tuple(v / norm for v in target)


def main() -> None:
    rclpy.init()
    node: Optional[WebsocketPoseBridge] = None
    try:
        node = WebsocketPoseBridge()
        try:
            rclpy.spin(node)
        except rclpy.executors.ExternalShutdownException:
            # Another thread (e.g., TF listener) requested shutdown; ignore.
            pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        # Guard against double shutdown when another thread already called it
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
