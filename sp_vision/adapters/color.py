"""Read color images with synchronized head feedback for checkerboard calibration."""
from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
import os
import socket
import threading
import time
import urllib.parse

import numpy as np

from .rgbd import (
    HIGH_COLOR_TOPIC,
    JOINT_STATE_TOPIC,
    _bridge_headers,
    _bridge_ssl_context,
    _bridge_url,
    _decode_brdg,
    _decode_joint_message,
    _ros_timestamp_ms,
    decode_color,
    decode_ros_color,
    head_from_ros_joint_state,
)


@dataclass(frozen=True)
class Tron2HighColorConfig:
    """Connection and synchronization settings; no camera calibration is needed."""

    host: str = "127.0.0.1:18443"
    ws_path: str = "/bridge/ws"
    internal_token: str | None = None
    verify_tls: bool = True
    timeout_s: float = 12.0
    max_state_skew_ms: float = 100.0
    history_size: int = 256
    color_topic: str = HIGH_COLOR_TOPIC
    joint_state_topic: str = JOINT_STATE_TOPIC
    ros_queue_size: int = 30
    ros_cache_max_age_s: float = 0.75


def _sync_report(config, transport, color_stamp, joint_stamp, head):
    skew_ms = abs(color_stamp - joint_stamp)
    if skew_ms > config.max_state_skew_ms:
        raise RuntimeError(
            f"TRON2 RGB/joint skew {skew_ms}ms exceeds "
            f"{config.max_state_skew_ms:g}ms")
    return {
        "transport": transport,
        "color_topic": config.color_topic,
        "joint_state_topic": config.joint_state_topic,
        "color_timestamp_ms": color_stamp,
        "joint_timestamp_ms": joint_stamp,
        "state_skew_ms": skew_ms,
        "state_alignment": "nearest_header_timestamp",
        "head_pitch_yaw": list(head),
    }


class Tron2HighColorCapture:
    """Keep both bridge streams open until their timestamp windows overlap."""

    def __init__(self, config: Tron2HighColorConfig) -> None:
        self.config = config

    async def _read_stream(self, topic, kind, queue):
        try:
            import websockets

            url = _bridge_url(
                self.config, topic, kind=kind,
                max_fps=30 if kind == "image" else None)
            async with websockets.connect(
                    url, ssl=_bridge_ssl_context(url, self.config),
                    additional_headers=_bridge_headers(self.config),
                    open_timeout=self.config.timeout_s,
                    close_timeout=min(1.0, self.config.timeout_s)) as stream:
                while True:
                    message = await stream.recv()
                    frame = None
                    if kind == "image" and isinstance(message, bytes):
                        frame = _decode_brdg(message)
                    elif kind == "joint" and isinstance(message, str):
                        frame = _decode_joint_message(message)
                    if frame is not None:
                        await queue.put((kind, frame))
        except Exception as error:
            await queue.put((kind, error))

    async def _capture_async(self):
        queue = asyncio.Queue(maxsize=self.config.history_size * 2)
        histories = {
            "image": deque(maxlen=self.config.history_size),
            "joint": deque(maxlen=self.config.history_size),
        }
        closest_skew = None

        async def collect():
            nonlocal closest_skew
            while True:
                kind, frame = await queue.get()
                if isinstance(frame, Exception):
                    raise frame
                histories[kind].append(frame)
                other = histories["joint" if kind == "image" else "image"]
                if not other:
                    continue
                nearest = min(other, key=lambda item: abs(
                    item.timestamp_ms - frame.timestamp_ms))
                skew = abs(nearest.timestamp_ms - frame.timestamp_ms)
                closest_skew = skew if closest_skew is None else min(closest_skew, skew)
                if skew <= self.config.max_state_skew_ms:
                    return (frame, nearest) if kind == "image" else (nearest, frame)

        readers = [
            asyncio.create_task(self._read_stream(self.config.color_topic, "image", queue)),
            asyncio.create_task(self._read_stream(self.config.joint_state_topic, "joint", queue)),
        ]
        try:
            return await asyncio.wait_for(collect(), timeout=self.config.timeout_s)
        except asyncio.TimeoutError as error:
            if closest_skew is not None:
                raise RuntimeError(
                    f"TRON2 RGB/joint skew {closest_skew}ms exceeds "
                    f"{self.config.max_state_skew_ms:g}ms after waiting "
                    f"{self.config.timeout_s:g}s for overlapping streams") from error
            raise TimeoutError(
                "TRON2 bridge produced no synchronized high-camera RGB/joint pair; "
                f"received color={len(histories['image'])}, "
                f"joint={len(histories['joint'])}") from error
        finally:
            for reader in readers:
                reader.cancel()
            await asyncio.gather(*readers, return_exceptions=True)

    def capture(self) -> tuple[np.ndarray, dict]:
        color, joint = asyncio.run(self._capture_async())
        report = _sync_report(
            self.config, "bridge", color.timestamp_ms, joint.timestamp_ms,
            joint.positions[14:16])
        return decode_color(color), report

    def close(self) -> None:
        """Each capture closes both connections before returning."""


class Tron2RosHighColorCapture:
    """Synchronize ROS color and joint headers without subscribing to depth."""

    def __init__(self, config: Tron2HighColorConfig) -> None:
        self.config = config
        self._condition = threading.Condition()
        self._start_lock = threading.Lock()
        self._started = False
        self._closed = False
        self._latest = None
        self._latest_received_monotonic = 0.0
        self._subscribers = []
        self._synchronizer = None

    def _ensure_started(self) -> None:
        with self._start_lock:
            if self._closed:
                raise RuntimeError("TRON2 ROS camera capture is closed")
            if self._started:
                return
            master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
            master = urllib.parse.urlparse(master_uri)
            if not master.hostname:
                raise RuntimeError(f"invalid TRON2 ROS_MASTER_URI: {master_uri!r}")
            try:
                with socket.create_connection(
                        (master.hostname, int(master.port or 11311)),
                        timeout=min(1.0, self.config.timeout_s)):
                    pass
            except OSError as error:
                raise ConnectionError(
                    f"TRON2 ROS master is unreachable: {master_uri}: {error}") from error
            try:
                import message_filters
                import rospy
                from sensor_msgs.msg import CompressedImage, Image, JointState
            except ImportError as error:
                raise RuntimeError(
                    "ROS Noetic Python bindings are required for TRON2 color "
                    "capture; source /opt/ros/noetic/setup.bash. "
                    "For bridge capture, install the optional live extra."
                ) from error
            if not rospy.core.is_initialized():
                rospy.init_node(
                    "tron2_calibration_camera", anonymous=True, disable_signals=True)
            color_type = (CompressedImage if self.config.color_topic.endswith(
                "/compressed") else Image)
            color = message_filters.Subscriber(
                self.config.color_topic, color_type,
                queue_size=self.config.ros_queue_size)
            joint = message_filters.Subscriber(
                self.config.joint_state_topic, JointState,
                queue_size=self.config.ros_queue_size)
            self._subscribers = [color, joint]
            synchronizer = message_filters.ApproximateTimeSynchronizer(
                self._subscribers, queue_size=self.config.ros_queue_size,
                slop=self.config.max_state_skew_ms / 1000.0,
                allow_headerless=False)
            synchronizer.registerCallback(self._on_pair)
            self._synchronizer = synchronizer
            self._started = True

    def _on_pair(self, color, joint) -> None:
        try:
            head = head_from_ros_joint_state(joint)
            report = _sync_report(
                self.config, "ros", _ros_timestamp_ms(color),
                _ros_timestamp_ms(joint), head)
        except (TypeError, ValueError, RuntimeError):
            return
        with self._condition:
            self._latest = color, report
            self._latest_received_monotonic = time.monotonic()
            self._condition.notify_all()

    def capture(self) -> tuple[np.ndarray, dict]:
        self._ensure_started()
        deadline = time.monotonic() + self.config.timeout_s
        with self._condition:
            while not self._closed:
                if (self._latest is not None
                        and time.monotonic() - self._latest_received_monotonic
                        <= self.config.ros_cache_max_age_s):
                    color, report = self._latest
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        "TRON2 ROS produced no synchronized high-camera RGB/joint "
                        f"pair within {self.config.max_state_skew_ms:g}ms; "
                        f"color={self.config.color_topic}, "
                        f"joint={self.config.joint_state_topic}")
                self._condition.wait(timeout=remaining)
            else:
                raise RuntimeError("TRON2 ROS camera capture stopped")
        return decode_ros_color(color), report

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
