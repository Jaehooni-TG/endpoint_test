#!/usr/bin/env python3
"""Bridge camera images from ROS 2 to a WebSocket stream.

Subscribes to a sensor_msgs/Image topic, JPEG-compresses the latest frame and
sends it as a binary WebSocket message at a configurable rate.

Default WebSocket URL matches the user's head camera track:
  ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=head_camera&mode=single
"""

from __future__ import annotations

import asyncio
import math
from threading import Event, Lock, Thread
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

try:
  import websockets
  from websockets.client import WebSocketClientProtocol
except ImportError as exc:  # pragma: no cover - dependency check
  raise RuntimeError(
      "Please install the 'websockets' Python package to use websocket_image_bridge_node"
  ) from exc

import numpy as np

try:
  from PIL import Image as PILImage
except ImportError as exc:  # pragma: no cover - dependency check
  raise RuntimeError(
      "Please install Pillow for image encoding (e.g. 'sudo apt install python3-pil' "
      "or 'python3 -m pip install pillow')."
  ) from exc

Gst = None  # type: ignore[assignment]
GST_AVAILABLE = False


class H264GstEncoder:
  """Simple in-process H.264 encoder using GStreamer appsrc/appsink.

  Each call to encode(...) pushes a single RGB frame into the pipeline and
  returns one encoded H.264 access unit. The pipeline is kept alive across
  frames for efficiency.
  """

  def __init__(self, width: int, height: int, fps: float, bitrate_bits: int) -> None:
    global Gst, GST_AVAILABLE
    if not GST_AVAILABLE:
      try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst as GstModule  # type: ignore[import-not-found]

        GstModule.init(None)
        Gst = GstModule
        GST_AVAILABLE = True
      except Exception as exc:  # pragma: no cover - runtime guard
        raise RuntimeError(
            "GStreamer Python bindings are not available. Please install "
            "'python3-gi' and 'gir1.2-gst-1.0', e.g.:\n"
            "  sudo apt install python3-gi gir1.2-gst-1.0"
        ) from exc

    self._width = int(width)
    self._height = int(height)
    fps_int = max(1, int(round(fps)))
    self._fps = float(fps_int)
    # x264enc bitrate is in kbit/s
    bitrate_kbps = max(1, int(bitrate_bits // 1000))

    pipeline_desc = (
        "appsrc is-live=true block=true format=TIME name=appsrc0 "
        f"caps=video/x-raw,format=RGB,width={self._width},height={self._height},"
        f"framerate={fps_int}/1 ! "
        "videoconvert ! "
        f"x264enc name=x264enc0 tune=zerolatency bitrate={bitrate_kbps} "
        "speed-preset=ultrafast key-int-max=24 ! "
        "video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! "
        "h264parse config-interval=1 ! "
        "appsink name=appsink0 emit-signals=false sync=false drop=true max-buffers=2"
    )

    self._pipeline = Gst.parse_launch(pipeline_desc)
    self._appsrc = self._pipeline.get_by_name("appsrc0")
    self._appsink = self._pipeline.get_by_name("appsink0")
    if self._appsrc is None or self._appsink is None:
      raise RuntimeError("Failed to construct GStreamer pipeline (appsrc/appsink not found).")

    self._frame_index = 0
    self._frame_duration_ns = int(1e9 / self._fps)
    self._lock = Lock()

    self._pipeline.set_state(Gst.State.PLAYING)

  def encode(self, rgb_image: "np.ndarray") -> Optional[bytes]:
    """Encode a single RGB frame (H, W, 3) and return H.264 bytes."""
    if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
      return None
    h, w, _ = rgb_image.shape
    if h != self._height or w != self._width:
      return None

    data = rgb_image.tobytes()

    with self._lock:
      buf = Gst.Buffer.new_allocate(None, len(data), None)
      buf.fill(0, data)
      buf.pts = self._frame_index * self._frame_duration_ns
      buf.duration = self._frame_duration_ns
      self._frame_index += 1

      ret = self._appsrc.emit("push-buffer", buf)
      if ret.value_nick != "ok":  # type: ignore[attr-defined]
        return None

      # Pull the next encoded sample with a short timeout.
      sample = self._appsink.emit("try-pull-sample", int(0.5 * Gst.SECOND))
      if not sample:
        return None
      buffer = sample.get_buffer()
      success, map_info = buffer.map(Gst.MapFlags.READ)
      if not success:
        return None
      try:
        return bytes(map_info.data)
      finally:
        buffer.unmap(map_info)

  def close(self) -> None:
    with self._lock:
      if self._pipeline is not None:
        self._pipeline.set_state(Gst.State.NULL)


class WebsocketImageBridge(Node):
  """ROS 2 node streaming camera frames over WebSocket."""

  def __init__(self) -> None:
    super().__init__("websocket_image_bridge")

    # WebSocket / source configuration
    self.declare_parameter(
        "websocket_url",
        "ws://cobot.center:8286/pang/ws/pub"
        "?channel=instant&name=so101&track=head_camera&mode=single",
    )
    self.declare_parameter("image_topic", "/so_arm/overhead_camera/image_raw")
    # Publish rate and reconnection behaviour
    # Default ≈24 FPS
    self.declare_parameter("publish_period", 1.0 / 24.0)
    self.declare_parameter("reconnect_delay", 3.0)
    # WebSocket keepalive settings (set interval<=0 to disable pings)
    self.declare_parameter("ws_ping_interval", 20.0)
    self.declare_parameter("ws_ping_timeout", 20.0)
    # Encoding settings
    self.declare_parameter("codec", "h264")  # jpeg | h264
    self.declare_parameter("jpeg_quality", 70)  # 0–100
    # Target resolution; if >0, frames are resized to fit within this box.
    self.declare_parameter("max_width", 1280)
    self.declare_parameter("max_height", 720)
    # Estimated target bitrate in bits/s for H.264
    self.declare_parameter("bitrate", 4_000_000)
    # H.264 codec string advertised to the Web client
    self.declare_parameter("h264_codec_string", "avc1.42E03C")

    # Resolve parameters
    self._url = (
        self.get_parameter("websocket_url").get_parameter_value().string_value
    )
    if not self._url:
      raise ValueError("Parameter 'websocket_url' must be provided.")

    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
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
    codec_param = (
        self.get_parameter("codec")
        .get_parameter_value()
        .string_value
        .strip()
        .lower()
    )
    if codec_param not in ("jpeg", "h264"):
      self.get_logger().warn(
          f"Unsupported codec '{codec_param}', falling back to 'jpeg'."
      )
      codec_param = "jpeg"
    self._codec = codec_param
    self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
    self._jpeg_quality = max(1, min(self._jpeg_quality, 100))
    self._max_width = int(self.get_parameter("max_width").value)
    self._max_height = int(self.get_parameter("max_height").value)
    if self._max_width < 0:
      self._max_width = 0
    if self._max_height < 0:
      self._max_height = 0

    self._bitrate = int(self.get_parameter("bitrate").value)
    if self._bitrate <= 0:
      self._bitrate = 4_000_000
    self._h264_codec_string = (
        self.get_parameter("h264_codec_string")
        .get_parameter_value()
        .string_value
        .strip()
        or "avc1.42E03C"
    )

    # H.264 encoder state (we track the last encoded frame size for headers).
    self._h264_encoder: Optional[H264GstEncoder] = None
    self._enc_width: Optional[int] = None
    self._enc_height: Optional[int] = None
    self._warned_missing_gst = False
    # Derive nominal FPS from publish period
    self._fps = max(1.0, 1.0 / self._period) if self._period > 0 else 30.0
    # Stream header state (for H.264)
    self._header_sent = False

    # Latest image cache
    self._lock = Lock()
    self._latest_image: Optional[Image] = None

    qos = QoSProfile(depth=5)
    self.create_subscription(Image, image_topic, self._on_image, qos)

    # Async WebSocket loop in a background thread
    self._stop_event = Event()
    self._loop: Optional[asyncio.AbstractEventLoop] = None
    self._thread = Thread(target=self._run_loop, daemon=True)
    self._thread.start()

    self.get_logger().info(
        "WebSocket image bridge started | "
        f"url: {self._url} | topic: {image_topic} | "
        f"codec={self._codec}, jpeg_quality={self._jpeg_quality}, "
        f"max_size=({self._max_width}x{self._max_height}), fps≈{self._fps:.1f}, "
        f"bitrate≈{self._bitrate}"
    )

  def _on_image(self, msg: Image) -> None:
    with self._lock:
      self._latest_image = msg

  def _get_latest_pil_image(self) -> Optional["PILImage.Image"]:
    """Convert latest ROS Image to a (possibly downscaled) RGB Pillow image."""
    with self._lock:
      img_msg = self._latest_image
    if img_msg is None:
      return None

    try:
      if img_msg.encoding not in ("rgb8", "bgr8"):
        self.get_logger().warn_once(
            f"Unsupported image encoding '{img_msg.encoding}', expected 'rgb8' or 'bgr8'."
        )
        return None

      expected_step = img_msg.width * 3
      if img_msg.step < expected_step:
        self.get_logger().warn_once(
            f"Image step ({img_msg.step}) smaller than width*3 ({expected_step}); cannot decode frame."
        )
        return None

      data = np.frombuffer(img_msg.data, dtype=np.uint8)
      try:
        data = data.reshape(img_msg.height, img_msg.step)
      except ValueError:
        self.get_logger().warn(
            f"Failed to reshape image buffer (h={img_msg.height}, step={img_msg.step})."
        )
        return None

      data = data[:, :expected_step]
      rgb = data.reshape(img_msg.height, img_msg.width, 3)
      if img_msg.encoding == "bgr8":
        rgb = rgb[..., ::-1]

      image = PILImage.fromarray(rgb, mode="RGB")
    except Exception as exc:
      self.get_logger().warn(f"Failed to convert ROS Image to RGB frame: {exc}")
      return None

    # Resize to target resolution if specified (exact size so that the encoder
    # caps remain valid).
    target_w = self._max_width
    target_h = self._max_height
    if target_w > 0 and target_h > 0:
      image = image.resize((target_w, target_h), PILImage.BILINEAR)

    return image

  def _encode_jpeg(self, pil_image: "PILImage.Image") -> Optional[bytes]:
    try:
      import io

      buf = io.BytesIO()
      pil_image.save(buf, format="JPEG", quality=int(self._jpeg_quality))
      return buf.getvalue()
    except Exception as exc:
      self.get_logger().warn(f"Failed to JPEG-encode image frame: {exc}")
      return None

  def _encode_h264(self, bgr_image: "np.ndarray") -> Optional[bytes]:
    """Encode a single frame as H.264 using in-process GStreamer."""
    h, w = bgr_image.shape[:2]
    if self._h264_encoder is None:
      try:
        self._h264_encoder = H264GstEncoder(w, h, self._fps, self._bitrate)
      except Exception:
        if not self._warned_missing_gst:
          self.get_logger().error(
              "GStreamer (python3-gi / gir1.2-gst-1.0) is not available; "
              "set codec:=jpeg or install the GStreamer bindings."
          )
          self._warned_missing_gst = True
        return None
      self._enc_width = w
      self._enc_height = h

    # Our encoder expects RGB input; convert from BGR.
    rgb = bgr_image[..., ::-1].copy()
    return self._h264_encoder.encode(rgb)

  def _build_frame(self) -> Optional[bytes]:
    pil_image = self._get_latest_pil_image()
    if pil_image is None:
      return None

    if self._codec == "h264":
      rgb = np.asarray(pil_image)
      if rgb.ndim != 3 or rgb.shape[2] != 3:
        self.get_logger().warn("Unexpected image shape for H.264 encoding.")
        return None
      # Our helper currently expects BGR input, so flip channels.
      bgr = rgb[..., ::-1].copy()
      return self._encode_h264(bgr)
    return self._encode_jpeg(pil_image)

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
          self.get_logger().info("WebSocket connection established for image bridge.")
          # Reset header state for this new connection.
          self._header_sent = False
          await self._send_loop(ws)
      except asyncio.CancelledError:  # pragma: no cover - loop shutdown
        break
      except Exception as exc:
        self.get_logger().warn(f"WebSocket connection error (image bridge): {exc}")
        if self._reconnect_delay > 0:
          await asyncio.sleep(self._reconnect_delay)

  async def _send_loop(self, ws: WebSocketClientProtocol) -> None:
    try:
      while not self._stop_event.is_set():
        frame = self._build_frame()
        if frame is not None:
          try:
            # For H.264, send a one-time text header describing the stream
            # before the first binary frame so the web client can configure
            # its decoder.
            if self._codec == "h264" and not self._header_sent:
              width = self._enc_width
              height = self._enc_height
              if width and height:
                fps_int = max(1, int(round(self._fps)))
                header = (
                    f"video/h264;codecs={self._h264_codec_string};"
                    f"width={width};height={height};"
                    f"framerate={fps_int};bitrate={self._bitrate};"
                )
                await ws.send(header)
                self._header_sent = True
                self.get_logger().info(f"Sent H.264 stream header: {header}")

            # Send encoded frame as binary WebSocket message.
            await ws.send(frame)
          except Exception as exc:
            self.get_logger().warn(f"Failed to send image frame: {exc}")
            break
        await asyncio.sleep(self._period)
    finally:
      self.get_logger().info("WebSocket image send loop terminated.")

  def destroy_node(self) -> bool:
    self._stop_event.set()
    # Clean up encoder / event loop
    if self._h264_encoder is not None:
      try:
        self._h264_encoder.close()
      except Exception:
        pass
    if self._loop is not None and self._loop.is_running():
      self._loop.call_soon_threadsafe(self._loop.stop)
    self._thread.join(timeout=2.0)
    return super().destroy_node()


def main() -> None:
  rclpy.init()
  node = WebsocketImageBridge()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
