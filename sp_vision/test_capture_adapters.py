import base64
import json
from pathlib import Path
import subprocess
import sys
import threading
from types import ModuleType, SimpleNamespace

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


def test_offline_imports_and_local_camera_adapter_do_not_import_deployment():
    assert "tron2_deployment" not in sys.modules
    transform = np.eye(4)
    assert np.array_equal(rigid(transform.tolist(), "camera transform"), transform)
    driver_types = calibration._import_capture_dependencies()
    assert driver_types[0](transform.tolist(), "camera transform").shape == (4, 4)


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


def test_wrist_remote_capture_uses_ros_setup_from_settings(monkeypatch):
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
    image, payload = calibration_wrist.capture_pair({"capture": settings})
    assert image.shape == (4, 5, 3)
    assert payload["frame_id"] == "right_color_optical_frame"
    assert captured["command"][-2:] == ["operator@camera-host", "bash -s"]
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
