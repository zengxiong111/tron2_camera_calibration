"""Read and align a metric RGB-D frame from TRON2's high RealSense camera.

The vendor bridge sends compressed depth as JPEG, which is useful for a
preview but destroys the 16-bit metric measurements.  This module therefore
subscribes to the raw ROS image topic and decodes its ``RIMG`` payload.  It is
observation-only: no control or robot-state requests are made here.
"""
from __future__ import annotations

import asyncio
from collections import deque
import json
import os
import socket
import ssl
import struct
import threading
import time
import urllib.parse
import uuid
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


HIGH_COLOR_TOPIC = "/camera/top/color/image_raw/compressed"
HIGH_DEPTH_TOPIC = "/camera/top/depth/image_rect_raw"
JOINT_STATE_TOPIC = "/joint_states"


@dataclass(frozen=True)
class Tron2HighRgbdConfig:
    """Camera calibration must be supplied explicitly by a deployment profile."""

    color_k: tuple[float, ...]
    color_dist: tuple[float, ...]
    depth_k: tuple[float, ...]
    depth_to_color_r: tuple[float, ...]
    depth_to_color_t_m: tuple[float, ...]
    host: str = "127.0.0.1:18443"
    ws_path: str = "/bridge/ws"
    internal_token: str | None = None
    verify_tls: bool = True
    timeout_s: float = 12.0
    max_skew_ms: int = 100
    max_state_skew_ms: int = 200
    sample_count: int = 20
    joint_state_topic: str = JOINT_STATE_TOPIC
    ros_color_topic: str = "/camera/top/color/image_raw/compressed"
    ros_depth_topic: str = "/camera/top/depth/image_rect_raw"
    ros_queue_size: int = 30
    ros_timeout_s: float = 4.0
    ros_cache_max_age_s: float = 0.75
    ros_joint_history_size: int = 256
    ros_joint_receipt_max_age_s: float = 0.25
    ros_head_stability_window_s: float = 0.5
    ros_head_stability_rad: float = 0.01


@dataclass(frozen=True)
class _BridgeFrame:
    timestamp_ms: int
    mime: str
    payload: bytes


@dataclass(frozen=True)
class _JointFrame:
    timestamp_ms: int
    positions: tuple[float, ...]


@dataclass(frozen=True)
class _RosJointSample:
    timestamp_ms: int
    received_monotonic: float
    message: Any
    head_pitch_yaw: tuple[float, float]


def _bridge_url(config: Tron2HighRgbdConfig, topic: str, *,
                kind: str = "image", max_fps: int | None = 30) -> str:
    host = config.host.strip()
    if "://" not in host:
        host = f"wss://{host}"
    query_values = {"topic": topic, "kind": kind}
    if max_fps is not None:
        query_values["max_fps"] = str(max_fps)
    query = urllib.parse.urlencode(query_values)
    return f"{host.rstrip('/')}{config.ws_path}?{query}"


def _bridge_headers(config: Tron2HighRgbdConfig) -> dict[str, str] | None:
    """Return the bridge's internal auth header without placing secrets in URLs."""
    token = (config.internal_token or "").strip()
    return {"X-Internal-Token": token} if token else None


def _bridge_ssl_context(url: str, config: Tron2HighRgbdConfig):
    """Use TLS only for a ``wss://`` proxy; SSH-forwarded bridge is plain WS."""
    if not url.startswith("wss://"):
        return None
    context = ssl.create_default_context()
    if not config.verify_tls:
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    return context


def _decode_brdg(data: bytes) -> _BridgeFrame | None:
    if len(data) < 14 or data[:4] != b"BRDG" or data[4] != 1:
        return None
    mime_end = 6 + data[5]
    if len(data) < mime_end + 8:
        return None
    return _BridgeFrame(
        timestamp_ms=struct.unpack_from("<Q", data, mime_end)[0],
        mime=data[6:mime_end].decode("utf-8", errors="replace"),
        payload=data[mime_end + 8:],
    )


def _decode_joint_message(message: str) -> _JointFrame | None:
    try:
        parsed = json.loads(message)
    except json.JSONDecodeError:
        return None
    if parsed.get("type") != "joint_state":
        return None
    try:
        positions = tuple(float(value) for value in
                          parsed.get("data", {}).get("positions", []))
        timestamp_ms = int(parsed.get("timestamp", time.time() * 1000))
    except (TypeError, ValueError):
        return None
    if len(positions) < 16 or not np.isfinite(positions).all():
        return None
    return _JointFrame(timestamp_ms=timestamp_ms, positions=positions)


def decode_raw_depth(frame: _BridgeFrame) -> np.ndarray:
    """Decode a bridge ``application/x-ros-image`` 16UC1 frame in millimetres."""
    raw = frame.payload
    if frame.mime != "application/x-ros-image" or len(raw) < 18 or raw[:4] != b"RIMG":
        raise ValueError("TRON2 bridge did not provide a raw ROS depth image")
    width, height, step = struct.unpack_from("<III", raw, 4)
    encoding_length = raw[17]
    data_start = 18 + encoding_length
    encoding = raw[18:data_start].decode("utf-8", errors="replace")
    if encoding != "16UC1" or step < width * 2 or len(raw) < data_start + step * height:
        raise ValueError(f"unsupported TRON2 depth payload: {encoding!r}")
    rows = np.frombuffer(raw[data_start:data_start + step * height], dtype=np.uint16)
    return rows.reshape(height, step // 2)[:, :width].copy()


def decode_color(frame: _BridgeFrame) -> np.ndarray:
    if frame.mime not in ("image/jpeg", "image/png"):
        raise ValueError(f"unsupported TRON2 color payload: {frame.mime!r}")
    bgr = cv2.imdecode(np.frombuffer(frame.payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("TRON2 color frame could not be decoded")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def align_depth_to_color(depth_mm: np.ndarray, config: Tron2HighRgbdConfig) -> np.ndarray:
    """Project raw depth into the color image, retaining nearest depth per pixel."""
    depth = np.asarray(depth_mm, dtype=np.uint16)
    if depth.shape != (480, 640):
        raise ValueError(f"unexpected TRON2 high depth shape: {depth.shape}")
    valid = (depth > 0) & (depth < 10_000)  # 65535 is the RealSense no-return sentinel.
    ys, xs = np.nonzero(valid)
    aligned = np.zeros(depth.shape, dtype=np.uint16)
    if not len(xs):
        return aligned
    z = depth[ys, xs].astype(np.float64) * 0.001
    kd = np.asarray(config.depth_k, dtype=np.float64).reshape(3, 3)
    xc = (xs.astype(np.float64) - kd[0, 2]) * z / kd[0, 0]
    yc = (ys.astype(np.float64) - kd[1, 2]) * z / kd[1, 1]
    points_depth = np.column_stack((xc, yc, z))
    rotation = np.asarray(config.depth_to_color_r, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(config.depth_to_color_t_m, dtype=np.float64)
    points_color = points_depth @ rotation.T + translation
    z_color = points_color[:, 2]
    positive = z_color > 0
    points_color = points_color[positive]
    # Aligned depth is consumed with the COLOR camera intrinsics. Its metric
    # values must therefore be color-camera Z, not the source depth-camera Z.
    color_depth_mm = np.rint(z_color[positive] * 1000.0)
    kc = np.asarray(config.color_k, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(config.color_dist, dtype=np.float64)
    projected, _ = cv2.projectPoints(points_color, np.zeros(3), np.zeros(3), kc, dist)
    pixels = np.rint(projected.reshape(-1, 2)).astype(np.int32)
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < depth.shape[1]) &
              (pixels[:, 1] >= 0) & (pixels[:, 1] < depth.shape[0]) &
              (color_depth_mm > 0) & (color_depth_mm < np.iinfo(np.uint16).max))
    flat = pixels[inside, 1] * depth.shape[1] + pixels[inside, 0]
    zbuffer = np.full(depth.size, np.iinfo(np.uint16).max, dtype=np.uint16)
    np.minimum.at(zbuffer, flat, color_depth_mm[inside].astype(np.uint16))
    zbuffer[zbuffer == np.iinfo(np.uint16).max] = 0
    return zbuffer.reshape(depth.shape)


class Tron2HighRgbdCapture:
    """One synchronized, color-aligned RGB-D observation from the high camera."""

    def __init__(self, config: Tron2HighRgbdConfig | None = None) -> None:
        if config is None:
            raise ValueError("explicit camera calibration is required")
        self.config = config

    def close(self) -> None:
        """Bridge capture owns no persistent connection between calls."""

    async def _frames(self, topic: str) -> list[_BridgeFrame]:
        try:
            import websockets
        except ImportError as error:  # pragma: no cover - deployment dependency
            raise RuntimeError("websockets is required for TRON2 bridge capture") from error
        frames: list[_BridgeFrame] = []
        deadline = time.monotonic() + self.config.timeout_s
        url = _bridge_url(self.config, topic)
        async with websockets.connect(
                url, ssl=_bridge_ssl_context(url, self.config),
                additional_headers=_bridge_headers(self.config),
                open_timeout=self.config.timeout_s) as socket:
            while len(frames) < self.config.sample_count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    message = await asyncio.wait_for(
                        socket.recv(), timeout=remaining)
                except asyncio.TimeoutError:
                    break
                except websockets.exceptions.ConnectionClosed as error:
                    if (topic == HIGH_DEPTH_TOPIC
                            and getattr(error, "reason", "") ==
                            "unknown_image_msg_type"):
                        raise RuntimeError(
                            "TRON2 cam_high depth is not published in the "
                            "current robot mode. Restore topic "
                            f"{HIGH_DEPTH_TOPIC}; depth from another camera "
                            "cannot be paired with cam_high RGB.") from error
                    raise
                if isinstance(message, bytes):
                    frame = _decode_brdg(message)
                    if frame is not None:
                        frames.append(frame)
        if not frames:
            raise TimeoutError(f"TRON2 bridge produced no frames for {topic}")
        return frames

    async def _joint_frames(self) -> list[_JointFrame]:
        try:
            import websockets
        except ImportError as error:  # pragma: no cover - deployment dependency
            raise RuntimeError("websockets is required for TRON2 bridge capture") from error
        frames: list[_JointFrame] = []
        deadline = time.monotonic() + self.config.timeout_s
        url = _bridge_url(
            self.config, self.config.joint_state_topic,
            kind="joint", max_fps=None)
        async with websockets.connect(
                url, ssl=_bridge_ssl_context(url, self.config),
                additional_headers=_bridge_headers(self.config),
                open_timeout=self.config.timeout_s) as socket:
            while len(frames) < self.config.sample_count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    message = await asyncio.wait_for(
                        socket.recv(), timeout=remaining)
                except asyncio.TimeoutError:
                    break
                if not isinstance(message, str):
                    continue
                frame = _decode_joint_message(message)
                if frame is not None:
                    frames.append(frame)
        if not frames:
            raise TimeoutError(
                f"TRON2 bridge produced no joint frames for "
                f"{self.config.joint_state_topic}")
        return frames

    async def _capture_async(
        self, *, include_joint_state: bool,
    ) -> tuple[list[_BridgeFrame], list[_BridgeFrame], list[_JointFrame]]:
        if include_joint_state:
            return await asyncio.gather(
                self._frames(HIGH_COLOR_TOPIC), self._frames(HIGH_DEPTH_TOPIC),
                self._joint_frames())
        color, depth = await asyncio.gather(
            self._frames(HIGH_COLOR_TOPIC), self._frames(HIGH_DEPTH_TOPIC))
        return color, depth, []

    def capture(self, *, include_joint_state: bool = True
                ) -> tuple[np.ndarray, np.ndarray, dict[str, object]]:
        color_frames, depth_frames, joint_frames = asyncio.run(
            self._capture_async(include_joint_state=include_joint_state))
        color_frame, depth_frame = min(
            ((color, depth) for color in color_frames for depth in depth_frames),
            key=lambda pair: abs(pair[0].timestamp_ms - pair[1].timestamp_ms),
        )
        skew_ms = abs(color_frame.timestamp_ms - depth_frame.timestamp_ms)
        if skew_ms > self.config.max_skew_ms:
            raise RuntimeError(
                f"TRON2 RGB/depth skew {skew_ms}ms exceeds {self.config.max_skew_ms}ms")
        joint_frame = None
        state_skew_ms = None
        if include_joint_state:
            joint_frame = min(
                joint_frames,
                key=lambda frame: abs(
                    frame.timestamp_ms - color_frame.timestamp_ms))
            state_skew_ms = abs(
                joint_frame.timestamp_ms - color_frame.timestamp_ms)
            if state_skew_ms > self.config.max_state_skew_ms:
                raise RuntimeError(
                    f"TRON2 RGB/joint skew {state_skew_ms}ms exceeds "
                    f"{self.config.max_state_skew_ms}ms")
        color = decode_color(color_frame)
        raw_depth = decode_raw_depth(depth_frame)
        if color.shape[:2] != raw_depth.shape:
            raise ValueError("TRON2 high color and depth resolutions differ")
        aligned_depth = align_depth_to_color(raw_depth, self.config)
        return color, aligned_depth, {
            "color_timestamp_ms": color_frame.timestamp_ms,
            "depth_timestamp_ms": depth_frame.timestamp_ms,
            "skew_ms": int(skew_ms),
            "raw_depth_valid_fraction": float(np.mean((raw_depth > 0) & (raw_depth < 10_000))),
            "aligned_depth_valid_fraction": float(np.mean(aligned_depth > 0)),
            **({
                "joint_timestamp_ms": joint_frame.timestamp_ms,
                "state_skew_ms": int(state_skew_ms),
                "head_pitch_yaw": list(joint_frame.positions[14:16]),
            } if joint_frame is not None else {}),
        }


def _ros_timestamp_ms(message) -> int:
    stamp = message.header.stamp
    return int(round(float(stamp.to_sec()) * 1000.0))


def head_from_ros_joint_state(message) -> np.ndarray:
    """Return measured ``[pitch, yaw]`` independent of JointState ordering."""
    positions = np.asarray(message.position, dtype=np.float64)
    names = list(message.name)
    if names:
        by_name = dict(zip(names, positions, strict=False))
        candidates = (
            ("head_pitch_Joint", "head_yaw_Joint"),
            ("head_pitch_joint", "head_yaw_joint"),
        )
        for pitch_name, yaw_name in candidates:
            if pitch_name in by_name and yaw_name in by_name:
                head = np.array(
                    [by_name[pitch_name], by_name[yaw_name]], dtype=np.float64)
                if np.isfinite(head).all():
                    return head
    if positions.shape[0] >= 16 and np.isfinite(positions[14:16]).all():
        return positions[14:16].copy()
    raise ValueError("TRON2 ROS joint state has no finite head pitch/yaw")


def decode_ros_color(message) -> np.ndarray:
    """Decode an official Tron2 ``sensor_msgs/Image`` to RGB uint8."""
    if hasattr(message, "format"):
        bgr = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("TRON2 compressed ROS color image could not be decoded")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    encoding = str(message.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported TRON2 ROS color encoding: {message.encoding!r}")
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width * channels:
        raise ValueError("malformed TRON2 ROS color image")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("truncated TRON2 ROS color image")
    rows = raw[:height * step].reshape(height, step)
    image = rows[:, :width * channels].reshape(height, width, channels)
    if channels == 4:
        image = image[:, :, :3]
    if encoding in ("bgr8", "bgra8"):
        image = image[:, :, ::-1]
    return image.copy()


def decode_ros_depth(message) -> np.ndarray:
    """Decode a Tron2 ROS depth image to uint16 millimetres."""
    if hasattr(message, "format"):
        # compressed_depth_image_transport prepends a small codec header.
        # Locate the PNG signature instead of depending on its C struct size.
        payload = bytes(message.data)
        signature = b"\x89PNG\r\n\x1a\n"
        offset = payload.find(signature)
        if offset < 0:
            raise ValueError("TRON2 compressed ROS depth has no PNG payload")
        depth = cv2.imdecode(
            np.frombuffer(payload[offset:], dtype=np.uint8),
            cv2.IMREAD_UNCHANGED)
        if depth is None or depth.ndim != 2 or depth.dtype != np.uint16:
            raise ValueError("TRON2 compressed ROS depth is not 16-bit metric PNG")
        return depth
    encoding = str(message.encoding).lower()
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0:
        raise ValueError("malformed TRON2 ROS depth image")
    if encoding in ("16uc1", "mono16"):
        if step < width * 2 or len(message.data) < height * step:
            raise ValueError("truncated TRON2 ROS depth image")
        dtype = np.dtype(">u2" if bool(message.is_bigendian) else "<u2")
        rows = np.frombuffer(message.data[:height * step], dtype=dtype).reshape(
            height, step // 2)
        return rows[:, :width].astype(np.uint16, copy=True)
    if encoding == "32fc1":
        if step < width * 4 or len(message.data) < height * step:
            raise ValueError("truncated TRON2 ROS depth image")
        dtype = np.dtype(">f4" if bool(message.is_bigendian) else "<f4")
        rows = np.frombuffer(message.data[:height * step], dtype=dtype).reshape(
            height, step // 4)
        depth_m = rows[:, :width].astype(np.float64)
        valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < 65.535)
        depth_mm = np.zeros((height, width), dtype=np.uint16)
        depth_mm[valid] = np.rint(depth_m[valid] * 1000.0).astype(np.uint16)
        return depth_mm
    raise ValueError(f"unsupported TRON2 ROS depth encoding: {message.encoding!r}")


class Tron2RosHighRgbdCapture:
    """Synchronized high-camera RGB-D from the official local ROS topics."""

    def __init__(self, config: Tron2HighRgbdConfig | None = None) -> None:
        if config is None:
            raise ValueError("explicit camera calibration is required")
        self.config = config
        self._condition = threading.Condition()
        self._start_lock = threading.Lock()
        self._started = False
        self._closed = False
        self._sequence = 0
        self._latest = None
        self._latest_received_monotonic = 0.0
        self._joint_history: deque[_RosJointSample] = deque(
            maxlen=self.config.ros_joint_history_size)
        self._subscribers = []
        self._synchronizer = None

    def _ensure_started(self) -> None:
        with self._start_lock:
            if self._started:
                return
            if self._closed:
                raise RuntimeError("TRON2 ROS camera capture is closed")
            master_uri = os.environ.get(
                "ROS_MASTER_URI", "http://localhost:11311")
            parsed_master = urllib.parse.urlparse(master_uri)
            if not parsed_master.hostname:
                raise RuntimeError(
                    f"invalid TRON2 ROS_MASTER_URI: {master_uri!r}")
            master_port = int(parsed_master.port or 11311)
            connect_timeout_s = min(1.0, self.config.ros_timeout_s)
            try:
                with socket.create_connection(
                        (parsed_master.hostname, master_port),
                        timeout=connect_timeout_s):
                    pass
            except OSError as error:
                raise ConnectionError(
                    "TRON2 ROS master is unreachable; camera capture was not "
                    "started. Connect this host to the robot LAN and verify "
                    f"ROS_MASTER_URI={master_uri}: {error}") from error
            try:
                import message_filters
                import rospy
                from sensor_msgs.msg import CompressedImage, Image, JointState
            except ImportError as error:  # pragma: no cover - deployment dependency
                raise RuntimeError(
                    "ROS Noetic Python bindings are required for TRON2 camera "
                    f"capture; failed import: {error}. Install the system ROS "
                    "Noetic Python packages and source /opt/ros/noetic/setup.bash. "
                    "For bridge capture, install the package's optional live extra."
                ) from error
            if not rospy.core.is_initialized():
                rospy.init_node(
                    "tron2_deployment_camera", anonymous=True, disable_signals=True)
            color_type = (CompressedImage if self.config.ros_color_topic.endswith(
                "/compressed") else Image)
            depth_type = (CompressedImage if self.config.ros_depth_topic.endswith(
                "/compressed") else Image)
            color = message_filters.Subscriber(
                self.config.ros_color_topic, color_type,
                queue_size=self.config.ros_queue_size)
            depth = message_filters.Subscriber(
                self.config.ros_depth_topic, depth_type,
                queue_size=self.config.ros_queue_size)
            joint = rospy.Subscriber(
                self.config.joint_state_topic, JointState, self._on_joint,
                queue_size=1, tcp_nodelay=True)
            synchronizer = message_filters.ApproximateTimeSynchronizer(
                [color, depth], queue_size=self.config.ros_queue_size,
                slop=self.config.max_skew_ms / 1000.0,
                allow_headerless=False)
            synchronizer.registerCallback(self._on_pair)
            self._subscribers = [color, depth, joint]
            self._synchronizer = synchronizer
            self._started = True

    def start(self) -> None:
        """Pre-subscribe so the first browser request can use a warm frame."""
        self._ensure_started()

    def _on_joint(self, joint) -> None:
        try:
            head = head_from_ros_joint_state(joint)
            sample = _RosJointSample(
                timestamp_ms=_ros_timestamp_ms(joint),
                received_monotonic=time.monotonic(),
                message=joint,
                head_pitch_yaw=(float(head[0]), float(head[1])),
            )
        except (TypeError, ValueError):
            return
        with self._condition:
            self._joint_history.append(sample)

    def _on_pair(self, color, depth) -> None:
        with self._condition:
            if not self._joint_history:
                return
            received = time.monotonic()
            color_timestamp_ms = _ros_timestamp_ms(color)
            joint_sample = min(
                self._joint_history,
                key=lambda sample: abs(
                    sample.timestamp_ms - color_timestamp_ms))
            recent = [
                sample for sample in self._joint_history
                if received - sample.received_monotonic
                <= self.config.ros_head_stability_window_s
            ]
            stable_receipt_fallback = False
            if (len(recent) >= 2
                    and received - recent[-1].received_monotonic
                    <= self.config.ros_joint_receipt_max_age_s):
                heads = np.asarray(
                    [sample.head_pitch_yaw for sample in recent],
                    dtype=np.float64)
                stable_receipt_fallback = bool(
                    np.max(np.ptp(heads, axis=0))
                    <= self.config.ros_head_stability_rad)
            self._latest = (
                color, depth, joint_sample, stable_receipt_fallback)
            self._latest_received_monotonic = received
            self._sequence += 1
            self._condition.notify_all()

    def capture(self, *, include_joint_state: bool = True
                ) -> tuple[np.ndarray, np.ndarray, dict[str, object]]:
        del include_joint_state  # synchronized ROS joint state is always included.
        self._ensure_started()
        deadline = time.monotonic() + self.config.ros_timeout_s
        with self._condition:
            baseline = self._sequence
            cache_is_fresh = (
                self._latest is not None
                and time.monotonic() - self._latest_received_monotonic
                <= self.config.ros_cache_max_age_s)
            while (not cache_is_fresh and self._sequence <= baseline
                   and not self._closed):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    master = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
                    raise TimeoutError(
                        "TRON2 ROS produced no synchronized high-camera RGB-D; "
                        f"master={master}, color={self.config.ros_color_topic}, "
                        f"depth={self.config.ros_depth_topic}")
                self._condition.wait(timeout=remaining)
            if self._closed or self._latest is None:
                raise RuntimeError("TRON2 ROS camera capture stopped")
            (color_message, depth_message, joint_sample,
             stable_receipt_fallback) = self._latest
        color_timestamp_ms = _ros_timestamp_ms(color_message)
        depth_timestamp_ms = _ros_timestamp_ms(depth_message)
        skew_ms = abs(color_timestamp_ms - depth_timestamp_ms)
        if skew_ms > self.config.max_skew_ms:
            raise RuntimeError(
                f"TRON2 RGB/depth skew {skew_ms}ms exceeds "
                f"{self.config.max_skew_ms}ms")
        color = decode_ros_color(color_message)
        raw_depth = decode_ros_depth(depth_message)
        if color.shape[:2] != raw_depth.shape:
            raise ValueError("TRON2 high color and depth resolutions differ")
        aligned_depth = align_depth_to_color(raw_depth, self.config)
        report = {
            "transport": "ros",
            "color_topic": self.config.ros_color_topic,
            "depth_topic": self.config.ros_depth_topic,
            "color_timestamp_ms": color_timestamp_ms,
            "depth_timestamp_ms": depth_timestamp_ms,
            "skew_ms": int(skew_ms),
            "raw_depth_valid_fraction": float(np.mean(
                (raw_depth > 0) & (raw_depth < 10_000))),
            "aligned_depth_valid_fraction": float(np.mean(aligned_depth > 0)),
        }
        if joint_sample is not None:
            joint_timestamp_ms = joint_sample.timestamp_ms
            state_skew_ms = abs(joint_timestamp_ms - color_timestamp_ms)
            state_alignment = "nearest_header_timestamp"
            if (state_skew_ms > self.config.max_state_skew_ms
                    and not stable_receipt_fallback):
                raise RuntimeError(
                    f"TRON2 RGB/joint skew {state_skew_ms}ms exceeds "
                    f"{self.config.max_state_skew_ms}ms and measured head "
                    "state is not stable")
            if state_skew_ms > self.config.max_state_skew_ms:
                state_alignment = "stable_head_receipt_fallback"
            report.update({
                "joint_timestamp_ms": joint_timestamp_ms,
                "state_skew_ms": int(state_skew_ms),
                "state_alignment": state_alignment,
                "state_header_skew_exceeded": bool(
                    state_skew_ms > self.config.max_state_skew_ms),
                "head_pitch_yaw": list(joint_sample.head_pitch_yaw),
            })
        return color, aligned_depth, report

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        for subscriber in self._subscribers:
            try:
                subscriber.unregister()
            except Exception:  # noqa: BLE001
                pass
        self._subscribers = []
        self._synchronizer = None
