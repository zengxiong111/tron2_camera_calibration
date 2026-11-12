import base64
import asyncio
from contextlib import nullcontext
import json
from pathlib import Path
import subprocess
import sys
import threading
from types import ModuleType, SimpleNamespace
import struct
import urllib.parse

import cv2
import numpy as np
import pytest


SP_ROOT = Path(__file__).parent
sys.path.insert(0, str(SP_ROOT.parent))
sys.path.insert(0, str(SP_ROOT))

from sp_vision import calibration, calibration_wrist
from sp_vision.adapters.validation import rigid
from sp_vision.adapters.arm_state import read_arm_q14
from sp_vision import capture_ros2_image
from sp_vision.adapters.color import (
    Tron2HighColorCapture, Tron2HighColorConfig, Tron2RosHighColorCapture,
)
from sp_vision.adapters.rgbd import _BridgeFrame, _JointFrame


def test_offline_imports_and_local_camera_adapter_do_not_import_deployment():
    assert "tron2_deployment" not in sys.modules
    transform = np.eye(4)
    assert np.array_equal(rigid(transform.tolist(), "camera transform"), transform)
    driver_types = calibration._import_capture_dependencies()
    assert driver_types[0](driver_types[1]()).config.color_topic.endswith("/compressed")


@pytest.mark.parametrize("skew_ms", [0, 100, 101])
def test_head_bridge_capture_needs_no_depth_or_prior_intrinsics(tmp_path, monkeypatch, skew_ms):
    profile = {"camera": {
        "backend": "bridge", "bridge_host": "ws://camera.invalid",
        "color_topic": "/camera/custom/color/compressed",
        "joint_state_topic": "/feedback/joints",
        "max_state_skew_ms": 1000,
    }}
    (tmp_path / "profile.json").write_text(json.dumps(profile), encoding="utf-8")
    config = {
        "_path": tmp_path / "head.json",
        "capture": {"profile": "profile.json", "timeout_s": 0.05},
        "quality": {"max_state_skew_ms": 100},
    }
    subscriptions = []
    color = np.full((4, 5, 3), (10, 20, 30), dtype=np.uint8)
    ok, encoded = cv2.imencode(".png", color)
    assert ok
    mime = b"image/png"
    image_message = (b"BRDG" + bytes((1, len(mime))) + mime
                     + struct.pack("<Q", 2000) + encoded.tobytes())
    joint_message = json.dumps({
        "type": "joint_state", "timestamp": 2000 + skew_ms,
        "data": {"positions": list(range(16))},
    })

    class FakeSocket:
        def __init__(self, url, **_kwargs):
            self.topic = urllib.parse.parse_qs(urllib.parse.urlparse(url).query)["topic"][0]
            subscriptions.append(self.topic)
            assert self.topic in ("/camera/custom/color/compressed", "/feedback/joints")

        async def __aenter__(self):
            return self

        async def __aexit__(self, *_args):
            pass

        async def recv(self):
            await asyncio.sleep(0)
            return joint_message if self.topic == "/feedback/joints" else image_message

    import websockets
    monkeypatch.setattr(websockets, "connect", FakeSocket)
    driver, _camera = calibration.open_camera(config)
    try:
        if skew_ms > 100:
            with pytest.raises(RuntimeError, match="RGB/joint skew 101ms exceeds 100ms"):
                driver.capture()
        else:
            rgb, report = driver.capture()
            assert np.array_equal(rgb, color[:, :, ::-1])
            assert report["head_pitch_yaw"] == [14, 15]
            assert report["state_skew_ms"] == skew_ms
            assert not any("depth" in key for key in report)
    finally:
        driver.close()
    assert sorted(subscriptions) == ["/camera/custom/color/compressed", "/feedback/joints"]


@pytest.mark.parametrize("early_stream", ["image", "joint"])
def test_bridge_waits_for_overlapping_streams_after_more_than_twenty_frames(monkeypatch, early_stream):
    ok, encoded = cv2.imencode(".png", np.zeros((4, 5, 3), dtype=np.uint8))
    assert ok
    finished = set()

    async def scenario():
        early_ready = asyncio.Event()
        late_ready = asyncio.Event()

        async def read_stream(_topic, kind, queue):
            def frame(stamp):
                if kind == "image":
                    return _BridgeFrame(stamp, "image/png", encoded.tobytes())
                return _JointFrame(stamp, tuple(range(16)))

            try:
                if kind == early_stream:
                    for stamp in range(1000, 1300, 10):
                        await queue.put((kind, frame(stamp)))
                    early_ready.set()
                    await late_ready.wait()
                    await queue.put((kind, frame(5000)))
                else:
                    await early_ready.wait()
                    late_ready.set()
                    await queue.put((kind, frame(5005)))
                await asyncio.Event().wait()
            finally:
                finished.add(kind)

        driver = Tron2HighColorCapture(Tron2HighColorConfig(timeout_s=1))
        monkeypatch.setattr(driver, "_read_stream", read_stream)
        color, joint = await driver._capture_async()
        assert abs(color.timestamp_ms - joint.timestamp_ms) == 5
        assert min(color.timestamp_ms, joint.timestamp_ms) == 5000

    asyncio.run(scenario())
    assert finished == {"image", "joint"}


@pytest.mark.parametrize("skew_ms", [50, 101])
def test_head_ros_capture_subscribes_only_to_color_and_joints(monkeypatch, skew_ms):
    import sp_vision.adapters.color as color_adapter

    subscriptions = []
    synchronizers = []
    color = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 1.0)),
        width=5, height=4, step=15, encoding="rgb8",
        data=np.zeros((4, 5, 3), dtype=np.uint8).tobytes())
    joint = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 1.0 + skew_ms / 1000)),
        name=["head_yaw_Joint", "head_pitch_Joint"], position=[0.2, 0.3])

    class Subscriber:
        def __init__(self, topic, *_args, **_kwargs):
            self.topic = topic
            self.closed = False
            subscriptions.append(self)

        def unregister(self):
            self.closed = True

    class Synchronizer:
        def __init__(self, subscribers, **kwargs):
            assert subscribers == subscriptions
            assert kwargs["slop"] == 0.1
            assert kwargs["allow_headerless"] is False
            synchronizers.append(self)

        def registerCallback(self, callback):
            callback(color, joint)

    rospy = ModuleType("rospy")
    rospy.core = SimpleNamespace(is_initialized=lambda: True)
    message_filters = ModuleType("message_filters")
    message_filters.Subscriber = Subscriber
    message_filters.ApproximateTimeSynchronizer = Synchronizer
    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs.__path__ = []
    messages = ModuleType("sensor_msgs.msg")
    messages.CompressedImage = type("CompressedImage", (), {})
    messages.Image = type("Image", (), {})
    messages.JointState = type("JointState", (), {})
    monkeypatch.setitem(sys.modules, "rospy", rospy)
    monkeypatch.setitem(sys.modules, "message_filters", message_filters)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", messages)
    monkeypatch.setattr(color_adapter.socket, "create_connection", lambda *_args, **_kwargs: nullcontext())
    monkeypatch.setenv("ROS_MASTER_URI", "http://camera.invalid:11311")
    driver = Tron2RosHighColorCapture(Tron2HighColorConfig(timeout_s=0.01))
    try:
        if skew_ms > 100:
            with pytest.raises(TimeoutError, match="no synchronized high-camera RGB/joint"):
                driver.capture()
        else:
            rgb, report = driver.capture()
            assert rgb.shape == (4, 5, 3)
            assert report["head_pitch_yaw"] == [0.3, 0.2]
            assert report["state_skew_ms"] == skew_ms
            assert report["state_alignment"] == "nearest_header_timestamp"
    finally:
        driver.close()
    assert len(synchronizers) == 1
    assert [sub.topic for sub in subscriptions] == [
        "/camera/top/color/image_raw/compressed", "/joint_states"]
    assert all(sub.closed for sub in subscriptions)


def test_arm_state_adapter_is_read_only_and_maps_named_controller_layout(monkeypatch):
    created = []
    config_module = ModuleType("tron2_env.config")

    class Tron2Config:
        def __init__(self, **kwargs):
            self.values = kwargs

    config_module.Tron2Config = Tron2Config
    transport_module = ModuleType("tron2_env.transport.websocket")

    class FakeTransport:
        def __init__(self, config):
            self.config = config
            self.joint_states = {"timestamp": 1000, "states": list(range(18))}
            self._state_lock = __import__("threading").Lock()
            self.disconnected = False
            created.append(self)

        def is_connected(self):
            return True

        def disconnect(self):
            self.disconnected = True

    transport_module.WebsocketTransport = FakeTransport
    package = ModuleType("tron2_env")
    package.__path__ = []
    transport_package = ModuleType("tron2_env.transport")
    transport_package.__path__ = []
    monkeypatch.setitem(sys.modules, "tron2_env", package)
    monkeypatch.setitem(sys.modules, "tron2_env.config", config_module)
    monkeypatch.setitem(sys.modules, "tron2_env.transport", transport_package)
    monkeypatch.setitem(sys.modules, "tron2_env.transport.websocket", transport_module)

    result = read_arm_q14({"host": "example.invalid", "port": 5000}, timeout_s=1)
    assert result == list(range(7)) + list(range(8, 15))
    config = created[0].config.values
    assert config["init_joints"] is None
    assert config["init_head"] is None
    assert config["init_ee_z_min"] is None
    assert created[0].disconnected
    # The fake transport exposes no command API, so read_arm_q14 can only poll feedback.
    assert not hasattr(FakeTransport, "send_joint_cmd")


def test_standalone_ros_capture_uses_configured_remote_setup_and_domain(monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout=b"jpeg", stderr=b"")

    monkeypatch.setattr(subprocess, "run", fake_run)
    payload = capture_ros2_image.capture(
        "calibration@camera-host", "/camera/test/compressed", 3,
        ros_setup="/srv/ros/setup.sh", ros_domain_id=17,
    )
    command, kwargs = calls[0]
    script = kwargs["input"].decode()
    assert payload == b"jpeg"
    assert command[-2:] == ["calibration@camera-host", "bash -s"]
    assert "source '/srv/ros/setup.sh'" in script
    assert "export ROS_DOMAIN_ID=17" in script
    assert "source /opt/ros/foxy/setup.bash" not in script


@pytest.mark.parametrize("use_identity", [False, True])
def test_wrist_remote_capture_uses_ros_setup_from_settings(tmp_path, monkeypatch, use_identity):
    image = np.zeros((4, 5, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    result_payload = {
        "image_b64": base64.b64encode(encoded.tobytes()).decode("ascii"),
        "image_stamp_ns": 1_000_000_000,
        "joint_stamp_ns": 1_000_000_000,
        "frame_id": "right_color_optical_frame",
        "joints": {},
    }
    captured = {}

    def fake_run(command, **kwargs):
        captured["command"] = command
        captured["script"] = kwargs["input"].decode()
        return SimpleNamespace(returncode=0, stdout=json.dumps(result_payload).encode(), stderr=b"")

    monkeypatch.setattr(subprocess, "run", fake_run)
    settings = {
        "color_topic": "/camera/right/color/image_resized/compressed",
        "host": "operator@camera-host",
        "ros_setup": "/opt/ros/custom/setup.bash",
        "ros_domain_id": 23,
        "timeout_s": 2,
        "max_state_skew_ms": 100,
    }
    if use_identity:
        (tmp_path / "camera_key").write_text("test-only identity placeholder")
        settings["ssh_identity_file"] = "camera_key"
    image, payload = calibration_wrist.capture_pair({
        "capture": settings, "_path": tmp_path / "wrist_config.json"})
    assert image.shape == (4, 5, 3)
    assert payload["frame_id"] == "right_color_optical_frame"
    assert captured["command"][-2:] == ["operator@camera-host", "bash -s"]
    if use_identity:
        assert "IdentitiesOnly=yes" in captured["command"]
        identity_index = captured["command"].index("-i")
        assert captured["command"][identity_index + 1] == str(tmp_path / "camera_key")
    else:
        assert "-i" not in captured["command"]
    assert "source '/opt/ros/custom/setup.bash'" in captured["script"]
    assert "export ROS_DOMAIN_ID=23" in captured["script"]


def _install_fake_tron2_env(monkeypatch, states, timestamp=1000):
    """Inject a feedback-only fake controller transport and return its instances."""
    created = []
    config_module = ModuleType("tron2_env.config")

    class Tron2Config:
        def __init__(self, **kwargs):
            self.values = kwargs

    config_module.Tron2Config = Tron2Config
    transport_module = ModuleType("tron2_env.transport.websocket")

    class FakeTransport:
        def __init__(self, config):
            self.config = config
            self.joint_states = {"timestamp": timestamp, "states": list(states)}
            self._state_lock = threading.Lock()
            self.disconnected = False
            created.append(self)

        def is_connected(self):
            return True

        def disconnect(self):
            self.disconnected = True

    transport_module.WebsocketTransport = FakeTransport
    package = ModuleType("tron2_env")
    package.__path__ = []
    transport_package = ModuleType("tron2_env.transport")
    transport_package.__path__ = []
    monkeypatch.setitem(sys.modules, "tron2_env", package)
    monkeypatch.setitem(sys.modules, "tron2_env.config", config_module)
    monkeypatch.setitem(sys.modules, "tron2_env.transport", transport_package)
    monkeypatch.setitem(sys.modules, "tron2_env.transport.websocket", transport_module)
    return created


def test_read_state_returns_arm_head_and_timestamp(monkeypatch):
    from sp_vision.adapters.arm_state import read_state

    _install_fake_tron2_env(monkeypatch, list(range(18)))
    state = read_state({"host": "example.invalid", "port": 5000}, timeout_s=1)
    assert state["arm_q14"] == list(range(7)) + list(range(8, 15))
    assert state["head_q2"] == [16, 17]
    assert state["timestamp_s"] == 1.0
    assert state["source"] == "real"


def test_state_command_writes_controller_state_json(tmp_path, monkeypatch):
    _install_fake_tron2_env(monkeypatch, list(range(18)))
    (tmp_path / "robot_profile.json").write_text(
        json.dumps({"robot": {"host": "example.invalid", "port": 5000}}), encoding="utf-8")
    config = {
        "schema_version": 1,
        "board": {"columns": 7, "rows": 10, "square_m": 0.021},
        "capture": {"profile": "robot_profile.json", "state_profile": "robot_profile.json", "timeout_s": 2},
        "_path": tmp_path / "head_config.json",
    }
    output = tmp_path / "tcp" / "pose-01.json"
    calibration.state_command(config, output)
    recorded = json.loads(output.read_text(encoding="utf-8"))
    assert recorded["arm_q14"] == list(range(7)) + list(range(8, 15))
    assert recorded["head_q2"] == [16, 17]
    assert recorded["timestamp_s"] == 1.0


def test_state_command_requires_a_deployment_profile(tmp_path):
    config = {"capture": {}, "_path": tmp_path / "head_config.json"}
    with pytest.raises(ValueError):
        calibration.controller_robot(config)


def test_state_command_is_wired_into_both_clis():
    head = calibration.build_parser().parse_args(["state", "--output", "pose.json"])
    assert head.command == "state"
    assert head.output == Path("pose.json")
    wrist = calibration_wrist.parser().parse_args(["state", "--output", "pose.json"])
    assert wrist.command == "state"
    assert wrist.output == Path("pose.json")


def test_example_configs_point_at_a_controller_state_profile():
    configs = SP_ROOT / "configs"
    head = json.loads((configs / "head_config.example.json").read_text(encoding="utf-8"))
    wrist = json.loads((configs / "wrist_config.example.json").read_text(encoding="utf-8"))
    assert head["capture"]["state_profile"]
    assert wrist["capture"]["state_profile"]
