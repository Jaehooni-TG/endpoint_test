#!/usr/bin/env python3
"""Bridge selected ROS joint states to a WebSocket JSON stream.

Sends messages like:
{
  "name": ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"],
  "position": [0.12, -0.34, 0.00, 1.57, -0.20, 0.00]
}
to a configurable WebSocket URL (default: robot_joint_states track).
"""

from __future__ import annotations

import asyncio
import json
from threading import Event, Lock, Thread
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

try:
  import websockets
  from websockets.client import WebSocketClientProtocol
except ImportError as exc:  # pragma: no cover - dependency check
  raise RuntimeError(
      "Please install the 'websockets' package to use websocket_joint_state_bridge_node"
  ) from exc


class WebsocketJointStateBridge(Node):
  """ROS 2 node streaming joint states over WebSocket."""

  def __init__(self) -> None:
    super().__init__("websocket_joint_state_bridge")

    # WebSocket / source configuration
    self.declare_parameter(
        "websocket_url",
        "ws://cobot.center:8286/pang/ws/pub"
        "?channel=instant&name=so101&track=robot_joint_states&mode=bundle",
    )
    self.declare_parameter("joint_topic", "/joint_states")
    self.declare_parameter(
        "joint_names",
        ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"],
    )
    # Match pose bridge 기본 상태 주기(조금 빠르게, ≈10 Hz)
    self.declare_parameter("publish_period", 0.1)
    self.declare_parameter("reconnect_delay", 3.0)
    # WebSocket keepalive settings (set interval<=0 to disable pings)
    self.declare_parameter("ws_ping_interval", 20.0)
    self.declare_parameter("ws_ping_timeout", 20.0)

    # Optional Jaw command reception (Web → ROS).
    # When enabled, incoming JSON payloads with {"name":[...], "position":[...]}
    # are inspected and the value for the configured joint is published as a
    # simple scalar command.
    self.declare_parameter("enable_jaw_command", False)
    self.declare_parameter("jaw_joint_name", "Jaw")
    self.declare_parameter("jaw_command_topic", "/so_arm/jaw_command")

    # Resolve parameters
    self._url = (
        self.get_parameter("websocket_url").get_parameter_value().string_value
    )
    if not self._url:
      raise ValueError("Parameter 'websocket_url' must be provided.")

    joint_topic = self.get_parameter("joint_topic").get_parameter_value().string_value
    names_param = self.get_parameter("joint_names").value
    if not isinstance(names_param, (list, tuple)) or not names_param:
      raise ValueError("Parameter 'joint_names' must be a non-empty list.")
    self._joint_order: List[str] = [str(n) for n in names_param]

    self._period = max(
        0.02, float(self.get_parameter("publish_period").value)
    )  # at most 50 Hz
    self._reconnect_delay = max(
        0.5, float(self.get_parameter("reconnect_delay").value)
    )
    self._ws_ping_interval = float(
        self.get_parameter("ws_ping_interval").value
    )
    self._ws_ping_timeout = float(
        self.get_parameter("ws_ping_timeout").value
    )

    # Jaw command configuration
    self._enable_jaw_command = bool(
        self.get_parameter("enable_jaw_command").value
    )
    self._jaw_joint_name = (
        self.get_parameter("jaw_joint_name").get_parameter_value().string_value
    )
    jaw_cmd_topic = (
        self.get_parameter("jaw_command_topic").get_parameter_value().string_value
    )
    self._jaw_cmd_pub = None
    if self._enable_jaw_command:
      self._jaw_cmd_pub = self.create_publisher(Float64, jaw_cmd_topic, QoSProfile(depth=10))
      self.get_logger().info(
          f"Jaw command reception enabled | joint='{self._jaw_joint_name}' -> topic:{jaw_cmd_topic}"
      )

    # Joint state cache
    self._lock = Lock()
    self._latest_positions: Dict[str, float] = {}

    qos = QoSProfile(depth=50)
    self.create_subscription(JointState, joint_topic, self._on_joint_state, qos)

    # Async WebSocket loop in a background thread
    self._stop_event = Event()
    self._loop: Optional[asyncio.AbstractEventLoop] = None
    self._thread = Thread(target=self._run_loop, daemon=True)
    self._thread.start()

    self.get_logger().info(
        "WebSocket joint state bridge started | "
        f"url: {self._url} | topic: {joint_topic} | joints: {self._joint_order}"
    )

  def _on_joint_state(self, msg: JointState) -> None:
    # Cache only the joints we care about.
    name_to_pos = dict(zip(msg.name, msg.position))
    with self._lock:
      for joint in self._joint_order:
        if joint in name_to_pos:
          self._latest_positions[joint] = float(name_to_pos[joint])

  def _build_payload(self) -> Optional[Dict[str, object]]:
    with self._lock:
      if not self._latest_positions:
        return None
      names: List[str] = []
      positions: List[float] = []
      for joint in self._joint_order:
        if joint in self._latest_positions:
          names.append(joint)
          positions.append(self._latest_positions[joint])
    if not names:
      return None
    return {"name": names, "position": positions}

  def _run_loop(self) -> None:
    self._loop = asyncio.new_event_loop()
    asyncio.set_event_loop(self._loop)
    try:
      self._loop.run_until_complete(self._main_loop())
    finally:
      pending = asyncio.all_tasks(self._loop)
      for task in pending:
        task.cancel()
      self._loop.run_until_complete(
          asyncio.gather(*pending, return_exceptions=True)
      )
      self._loop.close()

  async def _main_loop(self) -> None:
    while not self._stop_event.is_set():
      try:
        ping_interval = (
            None if self._ws_ping_interval <= 0 else self._ws_ping_interval
        )
        ping_timeout = (
            None if self._ws_ping_timeout <= 0 else self._ws_ping_timeout
        )
        async with websockets.connect(
            self._url, ping_interval=ping_interval, ping_timeout=ping_timeout
        ) as ws:
          self.get_logger().info("WebSocket connection established.")
          # Run send loop (RPi → Web) and optional receive loop (Web → RPi, Jaw only)
          send_task = asyncio.create_task(self._send_loop(ws))
          recv_task = (
              asyncio.create_task(self._recv_loop(ws))
              if self._enable_jaw_command
              else None
          )
          if recv_task is None:
            await send_task
          else:
            done, pending = await asyncio.wait(
                {send_task, recv_task}, return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
              task.cancel()
              try:
                await task
              except asyncio.CancelledError:
                pass
      except asyncio.CancelledError:  # pragma: no cover - loop shutdown
        break
      except Exception as exc:
        self.get_logger().warn(f"WebSocket connection error: {exc}")
        if self._reconnect_delay > 0:
          await asyncio.sleep(self._reconnect_delay)

  async def _send_loop(self, ws: WebSocketClientProtocol) -> None:
    try:
      while not self._stop_event.is_set():
        payload = self._build_payload()
        if payload is not None:
          try:
            # Debug: log exactly what we are about to send to the WebSocket.
            self.get_logger().debug(f"WS joint send payload: {payload}")
            data = json.dumps(payload).encode("utf-8")
            await ws.send(data)
          except Exception as exc:
            self.get_logger().warn(f"Failed to send joint payload: {exc}")
            break
        await asyncio.sleep(self._period)
    finally:
      self.get_logger().info("WebSocket joint send loop terminated.")

  async def _recv_loop(self, ws: WebSocketClientProtocol) -> None:
    """Receive joint-space commands from WebSocket and extract Jaw target.

    Expected payloads follow the same shape as the state stream, e.g.:
      {"name":[...], "position":[...]}
    Only the configured jaw joint is used; other joints are ignored.
    """
    if self._jaw_cmd_pub is None:
      # Guard: should not happen if enable_jaw_command is False, but keep safe.
      return

    try:
      async for raw_message in ws:
        # websockets delivers str for text frames, bytes for binary.
        if isinstance(raw_message, bytes):
          try:
            raw_message = raw_message.decode("utf-8")
          except UnicodeDecodeError:
            self.get_logger().debug("Received non-UTF8 frame on joint WS; ignoring.")
            continue

        text = raw_message.strip()
        if not text:
          continue
        if text[0] not in ("{", "["):
          # Skip non-JSON helper frames
          continue

        # Log the raw JSON payload exactly as received from WebSocket.
        self.get_logger().info(f"WS joint raw payload: {text}")

        try:
          payload = json.loads(text)
        except Exception:
          self.get_logger().debug("Received non-JSON joint payload; ignoring.")
          continue

        if not isinstance(payload, dict):
          continue

        names = payload.get("name")
        positions = payload.get("position")
        if not isinstance(names, list) or not isinstance(positions, list):
          continue
        if len(names) != len(positions):
          continue

        try:
          idx = names.index(self._jaw_joint_name)
        except ValueError:
          # No jaw joint in this message
          continue

        try:
          jaw_val = float(positions[idx])
        except (TypeError, ValueError):
          self.get_logger().debug("Jaw value in joint payload is non-numeric; ignoring.")
          continue

        # Log received Jaw delta value from WebSocket for debugging/inspection.
        self.get_logger().info(f"Received Jaw delta from WS: {jaw_val:.3f} rad")

        msg = Float64()
        msg.data = jaw_val
        self._jaw_cmd_pub.publish(msg)
    finally:
      self.get_logger().info("WebSocket joint receive loop terminated.")

  def destroy_node(self) -> bool:
    self._stop_event.set()
    if self._loop is not None and self._loop.is_running():
      self._loop.call_soon_threadsafe(self._loop.stop)
    self._thread.join(timeout=2.0)
    return super().destroy_node()


def main() -> None:
  rclpy.init()
  node = WebsocketJointStateBridge()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
