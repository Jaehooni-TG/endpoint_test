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
    # Match pose bridge default status period so pose/joint are in sync
    self.declare_parameter("publish_period", 0.2)
    self.declare_parameter("reconnect_delay", 3.0)
    # WebSocket keepalive settings (set interval<=0 to disable pings)
    self.declare_parameter("ws_ping_interval", 20.0)
    self.declare_parameter("ws_ping_timeout", 20.0)

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
          await self._send_loop(ws)
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
            data = json.dumps(payload).encode("utf-8")
            await ws.send(data)
          except Exception as exc:
            self.get_logger().warn(f"Failed to send joint payload: {exc}")
            break
        await asyncio.sleep(self._period)
    finally:
      self.get_logger().info("WebSocket joint send loop terminated.")

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
