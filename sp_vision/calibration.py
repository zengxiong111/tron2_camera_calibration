#!/usr/bin/env python3
"""Minimal moving-head camera calibration experiment.

The module is deliberately hardware-free at import time.  Only the ``capture``
subcommand opens the configured RGB-D source; every solver is offline and reads
JSON plus images from disk.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SCHEMA_VERSION = 1


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def write_json(path: Path, value) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(_json_value(value), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def read_json(path: Path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_config(path: Path):
    path = Path(path).resolve()
    config = read_json(path)
    if config.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported config schema_version in {path}")
    board = config.get("board", {})
    columns, rows = int(board.get("columns", 0)), int(board.get("rows", 0))
    square_m = float(board.get("square_m", 0))
    if columns < 3 or rows < 3 or not np.isfinite(square_m) or square_m <= 0:
        raise ValueError("board requires columns/rows >= 3 and positive square_m")
    config["_path"] = path
    return config


def apply_board_arguments(config, args) -> None:
    pattern = getattr(args, "pattern", None)
    square_m = getattr(args, "square_m", None)
    if pattern is not None:
        parts = pattern.lower().split("x")
        if len(parts) != 2:
            raise ValueError("--pattern must use COLSxROWS, for example 7x10")
        try:
            columns, rows = map(int, parts)
        except ValueError as error:
            raise ValueError("--pattern must use integer COLSxROWS, for example 7x10") from error
        if columns < 3 or rows < 3:
            raise ValueError("--pattern columns and rows must both be at least 3")
        config["board"]["columns"] = columns
        config["board"]["rows"] = rows
    if square_m is not None:
        if not np.isfinite(square_m) or square_m <= 0:
            raise ValueError("--square-m must be positive and finite")
        config["board"]["square_m"] = float(square_m)


def check_intrinsics_pattern(config, intrinsics) -> None:
    expected = {
        "columns": int(config["board"]["columns"]),
        "rows": int(config["board"]["rows"]),
        "square_m": float(config["board"]["square_m"]),
    }
    actual = intrinsics.get("pattern")
    if actual is None or int(actual.get("columns", -1)) != expected["columns"] \
            or int(actual.get("rows", -1)) != expected["rows"] \
            or not math.isclose(float(actual.get("square_m", -1)), expected["square_m"], rel_tol=0, abs_tol=1e-12):
        raise ValueError(
            f"checkerboard arguments do not match intrinsics: requested {expected}, intrinsics has {actual}"
        )


def config_path(config, value) -> Path:
    path = Path(os.path.expandvars(os.path.expanduser(str(value))))
    if not path.is_absolute():
        path = config["_path"].parent / path
    return path.resolve()


def package_path(name: str) -> Path:
    """Resolve a bundled standalone asset relative to this calibration unit."""
    return Path(__file__).resolve().parent / name


def vector(text: str, count: int) -> np.ndarray:
    values = np.fromstring(text, sep=" ", dtype=float)
    if values.size == 0 and count:
        values = np.zeros(count)
    if values.shape != (count,) or not np.isfinite(values).all():
        raise ValueError(f"expected {count} finite values, received {text!r}")
    return values


def transform(rotation=None, translation=None) -> np.ndarray:
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    if translation is not None:
        result[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return result


def invert(value: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=float).reshape(4, 4)
    result = np.eye(4)
    result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ value[:3, 3]
    return result


def rotation_angle(rotation: np.ndarray) -> float:
    return float(np.linalg.norm(Rotation.from_matrix(rotation).as_rotvec()))


def origin_transform(node: ET.Element) -> np.ndarray:
    origin = node.find("origin")
    if origin is None:
        return np.eye(4)
    xyz = vector(origin.get("xyz", "0 0 0"), 3)
    rpy = vector(origin.get("rpy", "0 0 0"), 3)
    # URDF fixed-axis roll, pitch, yaw is Rz(yaw) Ry(pitch) Rx(roll).
    rotation = Rotation.from_euler("xyz", rpy).as_matrix()
    return transform(rotation, xyz)


class UrdfKinematics:
    """Small URDF FK reader; visual meshes and ROS are intentionally ignored."""

    def __init__(self, urdf: Path):
        root = ET.parse(urdf).getroot()
        self.by_child = {}
        self.names = set()
        for node in root.findall("joint"):
            parent_node, child_node = node.find("parent"), node.find("child")
            if parent_node is None or child_node is None:
                raise ValueError(f"joint {node.get('name')} has no parent/child")
            child = child_node.get("link")
            axis_node = node.find("axis")
            axis = vector(axis_node.get("xyz", "1 0 0") if axis_node is not None else "1 0 0", 3)
            kind = node.get("type", "fixed")
            norm = float(np.linalg.norm(axis))
            if kind != "fixed" and norm == 0:
                raise ValueError(f"joint {node.get('name')} has a zero axis")
            item = {
                "name": node.get("name"),
                "type": kind,
                "parent": parent_node.get("link"),
                "child": child,
                "origin": origin_transform(node),
                "axis": axis / norm if norm else np.zeros(3),
            }
            if child in self.by_child:
                raise ValueError(f"URDF link {child} has multiple parent joints")
            self.by_child[child] = item
            self.names.add(item["name"])

    def matrix(self, base_link: str, target_link: str, positions: dict[str, float]) -> np.ndarray:
        chain = []
        current = target_link
        seen = set()
        while current != base_link:
            if current in seen:
                raise ValueError("cycle in URDF joint tree")
            seen.add(current)
            joint = self.by_child.get(current)
            if joint is None:
                raise ValueError(f"no URDF chain from {base_link} to {target_link}")
            chain.append(joint)
            current = joint["parent"]
        result = np.eye(4)
        for joint in reversed(chain):
            result = result @ joint["origin"]
            kind = joint["type"]
            if kind in ("revolute", "continuous"):
                if joint["name"] not in positions:
                    raise ValueError(f"missing state for movable joint {joint['name']}")
                motion = transform(Rotation.from_rotvec(joint["axis"] * positions[joint["name"]]).as_matrix())
                result = result @ motion
            elif kind == "prismatic":
                if joint["name"] not in positions:
                    raise ValueError(f"missing state for movable joint {joint['name']}")
                result = result @ transform(translation=joint["axis"] * positions[joint["name"]])
            elif kind != "fixed":
                raise ValueError(f"unsupported URDF joint type {kind}")
        return result


def mjcf_pose(node: ET.Element) -> np.ndarray:
    position = vector(node.get("pos", "0 0 0"), 3)
    if node.get("quat") is not None:
        wxyz = vector(node.get("quat"), 4)
        norm = float(np.linalg.norm(wxyz))
        if norm == 0:
            raise ValueError("MJCF body has a zero quaternion")
        wxyz /= norm
        rotation = Rotation.from_quat([wxyz[1], wxyz[2], wxyz[3], wxyz[0]]).as_matrix()
    elif node.get("euler") is not None:
        rotation = Rotation.from_euler("xyz", vector(node.get("euler"), 3)).as_matrix()
    else:
        rotation = np.eye(3)
    return transform(rotation, position)


class MjcfKinematics:
    """Relevant MJCF body/joint FK without loading meshes or opening MuJoCo."""

    def __init__(self, xml_path: Path):
        root = ET.parse(xml_path).getroot()
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise ValueError("scene.xml has no worldbody")
        self.bodies = {}

        def visit(node, parent):
            name = node.get("name")
            if not name:
                raise ValueError("all relevant MJCF bodies must be named")
            joints = []
            for joint in node.findall("joint"):
                kind = joint.get("type", "hinge")
                axis = vector(joint.get("axis", "0 0 1"), 3)
                norm = float(np.linalg.norm(axis))
                if norm == 0:
                    raise ValueError(f"MJCF joint {joint.get('name')} has a zero axis")
                joints.append(
                    {
                        "name": joint.get("name"),
                        "type": kind,
                        "axis": axis / norm,
                        "pos": vector(joint.get("pos", "0 0 0"), 3),
                    }
                )
            self.bodies[name] = {
                "parent": parent,
                "fixed": mjcf_pose(node),
                "joints": joints,
            }
            for child in node.findall("body"):
                visit(child, name)

        for body in worldbody.findall("body"):
            visit(body, None)

    def matrix(self, base_body: str, target_body: str, positions: dict[str, float]) -> np.ndarray:
        chain = []
        current = target_body
        seen = set()
        while current != base_body:
            if current in seen:
                raise ValueError("cycle in MJCF body tree")
            seen.add(current)
            body = self.bodies.get(current)
            if body is None:
                raise ValueError(f"scene.xml has no body named {current}")
            chain.append(body)
            current = body["parent"]
            if current is None:
                raise ValueError(f"no MJCF chain from {base_body} to {target_body}")
        result = np.eye(4)
        for body in reversed(chain):
            result = result @ body["fixed"]
            for joint in body["joints"]:
                if joint["name"] not in positions:
                    raise ValueError(f"missing state for MJCF joint {joint['name']}")
                value = float(positions[joint["name"]])
                if joint["type"] == "hinge":
                    pivot = transform(translation=joint["pos"])
                    unpivot = transform(translation=-joint["pos"])
                    motion = transform(Rotation.from_rotvec(joint["axis"] * value).as_matrix())
                    result = result @ pivot @ motion @ unpivot
                elif joint["type"] == "slide":
                    result = result @ transform(translation=joint["axis"] * value)
                else:
                    raise ValueError(f"unsupported MJCF joint type {joint['type']}")
        return result


def robot_kinematics(config):
    robot = config["robot"]
    return UrdfKinematics(config_path(config, robot["urdf"]))


def head_positions(config, head_q2) -> dict[str, float]:
    names = list(config["robot"]["head_joint_names"])
    values = np.asarray(head_q2, dtype=float)
    if len(names) != 2 or values.shape != (2,) or not np.isfinite(values).all():
        raise ValueError("head_q2 must match the two configured head_joint_names")
    return dict(zip(names, map(float, values)))


def head_matrix(config, kinematics: UrdfKinematics, head_q2) -> np.ndarray:
    robot = config["robot"]
    return kinematics.matrix(
        robot["base_link"], robot["pitch_link"], head_positions(config, head_q2)
    )


def nominal_camera_matrix(config, kinematics: UrdfKinematics) -> np.ndarray:
    robot = config["robot"]
    camera_frame = robot.get("camera_frame")
    if not camera_frame:
        raise ValueError("robot.camera_frame is required")
    return kinematics.matrix(robot["pitch_link"], camera_frame, {})


def check_model_consistency(config, kinematics: UrdfKinematics, head_states) -> dict:
    """Cross-check the final URDF against the MJCF scene for relevant frames."""
    robot = config["robot"]
    model_path = robot.get("model_xml")
    if not model_path:
        raise ValueError("robot.model_xml is required for final-model consistency checks")
    scene = MjcfKinematics(config_path(config, model_path))
    errors = []
    nominal_urdf = nominal_camera_matrix(config, kinematics)
    for head_q2 in head_states:
        positions = head_positions(config, head_q2)
        pitch_scene = scene.matrix(robot["base_link"], robot["pitch_link"], positions)
        camera_scene = scene.matrix(robot["pitch_link"], robot["camera_frame"], {})
        pitch_urdf = head_matrix(config, kinematics, head_q2)
        errors.append(
            {
                "head_q2": list(map(float, head_q2)),
                "pitch_translation_m": float(np.linalg.norm(pitch_scene[:3, 3] - pitch_urdf[:3, 3])),
                "pitch_rotation_rad": rotation_angle(pitch_scene[:3, :3].T @ pitch_urdf[:3, :3]),
                "camera_translation_m": float(np.linalg.norm(camera_scene[:3, 3] - nominal_urdf[:3, 3])),
                "camera_rotation_rad": rotation_angle(camera_scene[:3, :3].T @ nominal_urdf[:3, :3]),
            }
        )
    maximum = max(max(item[key] for item in errors) for key in (
        "pitch_translation_m", "pitch_rotation_rad", "camera_translation_m", "camera_rotation_rad"
    ))
    tolerance = float(config.get("quality", {}).get("model_consistency_tolerance", 1e-6))
    if maximum > tolerance:
        raise ValueError(
            f"assembly.urdf and scene.xml disagree on the head/camera chain: max error {maximum}"
        )
    return {"passed": True, "tolerance": tolerance, "maximum_error": maximum, "samples": errors}


def arm_positions(config, arm_q14) -> dict[str, float]:
    arm = np.asarray(arm_q14, dtype=float)
    names = list(config["robot"]["arm_joint_names"]["left"]) + list(
        config["robot"]["arm_joint_names"]["right"]
    )
    if len(names) != 14 or arm.shape != (14,) or not np.isfinite(arm).all():
        raise ValueError("arm_q14 must match the fourteen configured arm joint names")
    return dict(zip(names, map(float, arm)))


def wrist_matrix(config, kinematics: UrdfKinematics, arm_q14, side: str) -> np.ndarray:
    robot = config["robot"]
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    return kinematics.matrix(
        robot["base_link"], robot["wrist_links"][side], arm_positions(config, arm_q14)
    )


def board_points(config) -> np.ndarray:
    board = config["board"]
    columns, rows, square = int(board["columns"]), int(board["rows"]), float(board["square_m"])
    return np.array(
        [[column * square, row * square, 0.0] for row in range(rows) for column in range(columns)],
        dtype=np.float32,
    )


def detection_quality(image: np.ndarray, corners: np.ndarray, config) -> dict:
    height, width = image.shape[:2]
    board = config["board"]
    quality = config.get("quality", {})
    columns, rows = int(board["columns"]), int(board["rows"])
    points = corners.reshape(rows, columns, 2)
    crosses = []
    for row in range(rows - 1):
        for column in range(columns - 1):
            right = points[row, column + 1] - points[row, column]
            down = points[row + 1, column] - points[row, column]
            crosses.append(float(right[0] * down[1] - right[1] * down[0]))
    crosses = np.asarray(crosses)
    topology_ok = bool(
        crosses.size
        and np.min(np.abs(crosses)) > 0.25
        and (np.all(crosses > 0) or np.all(crosses < 0))
    )
    hull = cv2.convexHull(corners.astype(np.float32))
    coverage = float(abs(cv2.contourArea(hull)) / (width * height))
    margin = float(
        min(
            corners[:, 0].min(),
            corners[:, 1].min(),
            width - 1 - corners[:, 0].max(),
            height - 1 - corners[:, 1].max(),
        )
    )
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    laplacian_variance = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    accepted = bool(
        topology_ok
        and coverage >= float(quality.get("min_board_area_ratio", 0.015))
        and margin >= float(quality.get("min_border_px", 3.0))
        and laplacian_variance >= float(quality.get("min_laplacian_variance", 0.0))
    )
    return {
        "accepted": accepted,
        "topology_ok": topology_ok,
        "board_area_ratio": coverage,
        "border_margin_px": margin,
        "laplacian_variance": laplacian_variance,
    }


def detect_board(image: np.ndarray, config, flip_180: bool = False):
    if image is None or image.size == 0:
        raise ValueError("empty image")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    board = config["board"]
    pattern = (int(board["columns"]), int(board["rows"]))
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, raw = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    if not found or raw is None or len(raw) != pattern[0] * pattern[1]:
        return None, {
            "accepted": False,
            "detected": False,
            "reason": f"expected {pattern[0] * pattern[1]} inner corners",
        }
    corners = np.asarray(raw, dtype=np.float32).reshape(-1, 2)
    if flip_180:
        corners = corners[::-1].copy()
    metrics = detection_quality(image, corners, config)
    metrics.update({"detected": True, "corner_count": int(len(corners)), "flip_180": bool(flip_180)})
    return corners, metrics


def draw_detection(image: np.ndarray, corners, config, metrics, detailed=False) -> np.ndarray:
    drawing = image.copy()
    columns, rows = int(config["board"]["columns"]), int(config["board"]["rows"])
    if corners is not None:
        grid = corners.reshape(rows, columns, 2)
        for row in range(rows):
            hue = int(179 * row / max(rows - 1, 1))
            color = cv2.cvtColor(np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
            color = tuple(map(int, color))
            line = np.rint(grid[row]).astype(int).reshape(-1, 1, 2)
            cv2.polylines(drawing, [line], False, color, 2, cv2.LINE_AA)
            for column, point in enumerate(grid[row]):
                center = tuple(np.rint(point).astype(int))
                cv2.circle(drawing, center, 3, color, -1, cv2.LINE_AA)
                if detailed:
                    cv2.putText(
                        drawing,
                        f"{row},{column}",
                        (center[0] + 3, center[1] - 3),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.32,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
        for row, column in ((0, 0), (0, columns - 1), (rows - 1, 0), (rows - 1, columns - 1)):
            point = tuple(np.rint(grid[row, column]).astype(int))
            cv2.circle(drawing, point, 7, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(
                drawing,
                f"({row},{column})",
                (point[0] + 7, point[1] + 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
    status = "ACCEPT" if metrics.get("accepted") else "REJECT"
    summary = (
        f"{status} corners={metrics.get('corner_count', 0)} "
        f"area={metrics.get('board_area_ratio', 0):.3f} "
        f"sharp={metrics.get('laplacian_variance', 0):.0f} "
        f"flip180={metrics.get('flip_180', False)}"
    )
    cv2.rectangle(drawing, (0, 0), (drawing.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(
        drawing,
        summary,
        (8, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 220, 0) if metrics.get("accepted") else (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    return drawing


def enumerate_views(session: Path):
    return [path for path in sorted(Path(session).glob("view-*")) if (path / "color.png").is_file()]


def load_view(path: Path, config):
    image = cv2.imread(str(path / "color.png"), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot read {path / 'color.png'}")
    frame_path = path / "frame.json"
    frame = read_json(frame_path) if frame_path.is_file() else {}
    flip = bool(frame.get("corner_order_reversed_180", False))
    corners, metrics = detect_board(image, config, flip)
    return image, frame, corners, metrics


def check_frame_sync(frame, config) -> None:
    if "head_q2" not in frame:
        raise ValueError("frame.json lacks synchronized head_q2")
    head_positions(config, frame["head_q2"])
    sync = frame.get("sensor_sync", {})
    quality = config.get("quality", {})
    if sync.get("state_header_skew_exceeded"):
        raise ValueError("joint/image header skew exceeded; fallback samples are not allowed")
    alignment = str(sync.get("state_alignment", ""))
    if "fallback" in alignment.lower():
        raise ValueError(f"unsynchronized state alignment is not allowed: {alignment}")
    state_skew = sync.get("state_skew_ms")
    if state_skew is not None and float(state_skew) > float(quality.get("max_state_skew_ms", 100.0)):
        raise ValueError(f"state skew {state_skew} ms exceeds the configured limit")


def _import_capture_dependencies():
    try:
        from tron2_deployment.config import rigid
        from tron2_deployment.rgbd import (
            Tron2HighRgbdCapture,
            Tron2HighRgbdConfig,
            Tron2RosHighRgbdCapture,
        )
    except ImportError as error:
        raise RuntimeError(
            "head capture requires the configured camera adapter; offline "
            "calibration commands do not require tron2_deployment. Install the "
            "adapter in the runtime environment or provide pre-captured views."
        ) from error
    return rigid, Tron2HighRgbdCapture, Tron2HighRgbdConfig, Tron2RosHighRgbdCapture


def open_camera(config):
    """Open the existing bridge/ROS adapter without imposing fixed-head calibration."""
    capture_config = config.get("capture", {})
    profile_path = config_path(config, capture_config["profile"])
    profile = read_json(profile_path)
    camera = profile["camera"]
    rigid, bridge_type, options_type, ros_type = _import_capture_dependencies()
    depth_to_color = rigid(camera["depth_to_color"], "depth_to_color")
    options = {
        "color_k": tuple(np.asarray(camera["intrinsics"], dtype=float).ravel()),
        "color_dist": tuple(camera["distortion"]),
        "depth_k": tuple(np.asarray(camera["depth_intrinsics"], dtype=float).ravel()),
        "depth_to_color_r": tuple(depth_to_color[:3, :3].ravel()),
        "depth_to_color_t_m": tuple(depth_to_color[:3, 3]),
        "ros_color_topic": camera["color_topic"],
        "ros_depth_topic": camera["depth_topic"],
        "joint_state_topic": camera.get("joint_state_topic", "/joint_states"),
        "host": camera.get("bridge_host", "127.0.0.1:18443"),
        "ws_path": camera.get("bridge_path", "/bridge/ws"),
        "internal_token": os.environ.get(camera.get("token_env", "TRON2_BRIDGE_TOKEN")),
        "verify_tls": camera.get("verify_tls", True),
        "max_skew_ms": camera.get("max_skew_ms", 100),
        "max_state_skew_ms": camera.get("max_state_skew_ms", 100),
    }
    if camera["backend"] == "ros":
        os.environ["ROS_MASTER_URI"] = camera["ros_master_uri"]
        if camera.get("ros_ip"):
            os.environ["ROS_IP"] = camera["ros_ip"]
        driver_type = ros_type
    elif camera["backend"] == "bridge":
        driver_type = bridge_type
    else:
        raise ValueError("capture requires a real bridge or ROS camera profile")
    return driver_type(options_type(**options)), camera


def _replace_capture_views(session: Path, staged: Path) -> None:
    """Replace numbered views after a complete capture, restoring them on failure."""
    backup = staged.parent / "previous"
    backup.mkdir()
    previous = [path for path in session.iterdir()
                if path.is_dir() and re.fullmatch(r"view-\d+", path.name)]
    installed = []
    try:
        for path in previous:
            path.replace(backup / path.name)
        for path in sorted(staged.iterdir()):
            destination = session / path.name
            path.replace(destination)
            installed.append(destination)
    except OSError:
        for path in installed:
            path.replace(staged / path.name)
        for path in backup.iterdir():
            path.replace(session / path.name)
        raise


def capture_command(config, session: Path, count: int, append: bool = False) -> None:
    session = Path(session)
    session.mkdir(parents=True, exist_ok=True)
    existing = enumerate_views(session) if append else []
    next_index = max([int(path.name.split("-")[-1]) for path in existing] or [0]) + 1
    with tempfile.TemporaryDirectory(prefix=f".{session.name}-capture-", dir=session.parent) as temporary:
        staged = Path(temporary) / "new"
        staged.mkdir()
        driver, camera_config = open_camera(config)
        print("Keys: s=save accepted sample, r/space=recapture, f=flip corner order 180 deg, q=quit")
        saved = 0
        try:
            while saved < count:
                rgb, _depth, sync = driver.capture(include_joint_state=True)
                image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                flip = False
                while True:
                    corners, metrics = detect_board(image, config, flip)
                    frame = {
                        "schema_version": SCHEMA_VERSION,
                        "timestamp_s": time.time(),
                        "camera_id": camera_config.get("identity"),
                        "width": int(image.shape[1]),
                        "height": int(image.shape[0]),
                        "head_q2": sync.get("head_pitch_yaw"),
                        "sensor_sync": sync,
                        "corner_order_reversed_180": flip,
                        "pattern": {
                            "columns": int(config["board"]["columns"]),
                            "rows": int(config["board"]["rows"]),
                            "square_m": float(config["board"]["square_m"]),
                        },
                        "detection": metrics,
                    }
                    try:
                        check_frame_sync(frame, config)
                    except ValueError as error:
                        metrics = dict(metrics)
                        metrics["accepted"] = False
                        metrics["sync_error"] = str(error)
                        frame["detection"] = metrics
                    drawing = draw_detection(image, corners, config, metrics)
                    cv2.imshow("sp_vision calibration capture", drawing)
                    key = cv2.waitKey(0) & 0xFF
                    if key in (ord("q"), 27):
                        return
                    if key == ord("f") and corners is not None:
                        flip = not flip
                        continue
                    if key == ord("s"):
                        if not metrics.get("accepted"):
                            print(f"Rejected: {metrics}")
                            continue
                        output = (session if append else staged) / f"view-{next_index:03d}"
                        output.mkdir()
                        if not cv2.imwrite(str(output / "color.png"), image):
                            raise OSError(f"cannot write {output / 'color.png'}")
                        write_json(output / "frame.json", frame)
                        cv2.imwrite(str(output / "corners.png"), drawing)
                        print(
                            f"Saved {output.name}: head_q2={frame['head_q2']} "
                            f"area={metrics['board_area_ratio']:.3f}"
                        )
                        next_index += 1
                        saved += 1
                        break
                    # r, space, Enter, and every unhandled key recapture without saving.
                    break
        finally:
            driver.close()
            cv2.destroyAllWindows()
        if not append:
            _replace_capture_views(session, staged)
            print("Replaced previous view-* images; rerun intrinsics and extrinsics for this session.")


def _camera_fit(object_sets, image_sets, size):
    outputs = cv2.calibrateCameraExtended(
        object_sets,
        [points.reshape(-1, 1, 2) for points in image_sets],
        size,
        None,
        None,
    )
    rms, camera, distortion, rvecs, tvecs, std_intrinsic, _std_extrinsic, per_view = outputs
    return {
        "rms": float(rms),
        "camera_matrix": camera,
        "distortion": distortion.reshape(-1),
        "rvecs": rvecs,
        "tvecs": tvecs,
        "intrinsic_std": np.asarray(std_intrinsic).reshape(-1),
        "per_view_rms": np.asarray(per_view).reshape(-1),
    }


def radial_monotonicity(camera, distortion, size):
    distortion = np.pad(np.asarray(distortion, dtype=float), (0, max(0, 5 - len(distortion))))
    k1, k2, _p1, _p2, k3 = distortion[:5]
    width, height = size
    fx, fy, cx, cy = camera[0, 0], camera[1, 1], camera[0, 2], camera[1, 2]
    radii = [math.hypot((x - cx) / fx, (y - cy) / fy) for x in (0, width - 1) for y in (0, height - 1)]
    r = np.linspace(0, max(radii), 1000)
    derivative = 1 + 3 * k1 * r**2 + 5 * k2 * r**4 + 7 * k3 * r**6
    return {"max_normalized_radius": float(r[-1]), "minimum_radial_derivative": float(derivative.min())}


def intrinsics_command(config, session: Path, output: Path):
    views = enumerate_views(session)
    holdout_count = int(config.get("quality", {}).get("holdout_count", 6))
    minimum = int(config.get("quality", {}).get("min_intrinsic_views", 12))
    if len(views) < minimum + holdout_count:
        raise ValueError(f"need at least {minimum + holdout_count} views, found {len(views)}")
    split = len(views) - holdout_count
    training_paths, holdout_paths = views[:split], views[split:]
    object_template = board_points(config)
    accepted = []
    rejected = []
    image_size = None
    diagnostics = Path(session) / "diagnostics" / "intrinsics"
    diagnostics.mkdir(parents=True, exist_ok=True)
    for path in views:
        image, frame, corners, metrics = load_view(path, config)
        if image_size is None:
            image_size = (image.shape[1], image.shape[0])
        elif image_size != (image.shape[1], image.shape[0]):
            raise ValueError("all calibration images must have identical dimensions")
        cv2.imwrite(str(diagnostics / f"{path.name}.png"), draw_detection(image, corners, config, metrics))
        item = {"name": path.name, "path": path, "corners": corners, "metrics": metrics, "frame": frame}
        if path in training_paths and corners is not None and metrics.get("accepted"):
            accepted.append(item)
        elif path in training_paths:
            rejected.append({"view": path.name, "reason": "corner detection quality", "metrics": metrics})
    if len(accepted) < minimum:
        raise ValueError(f"only {len(accepted)} training views passed detection; require {minimum}")
    threshold = float(config.get("quality", {}).get("max_intrinsic_view_rms_px", 1.5))
    while True:
        fit = _camera_fit(
            [object_template.copy() for _ in accepted],
            [item["corners"] for item in accepted],
            image_size,
        )
        worst = int(np.argmax(fit["per_view_rms"]))
        if fit["per_view_rms"][worst] <= threshold or len(accepted) <= minimum:
            break
        item = accepted.pop(worst)
        rejected.append(
            {
                "view": item["name"],
                "reason": "intrinsic per-view reprojection error",
                "rms_px": float(fit["per_view_rms"][worst]),
            }
        )
    monotonic = radial_monotonicity(fit["camera_matrix"], fit["distortion"], image_size)
    passed = bool(
        len(accepted) >= minimum
        and float(np.max(fit["per_view_rms"])) <= threshold
        and monotonic["minimum_radial_derivative"] > 0
    )
    result = {
        "schema_version": SCHEMA_VERSION,
        "kind": "sp_vision_intrinsics",
        "passed": passed,
        "image_size": list(image_size),
        "pattern": {
            "columns": int(config["board"]["columns"]),
            "rows": int(config["board"]["rows"]),
            "square_m": float(config["board"]["square_m"]),
        },
        "camera_matrix": fit["camera_matrix"],
        "distortion": fit["distortion"],
        "rms_px": fit["rms"],
        "intrinsic_std": fit["intrinsic_std"],
        "training_views": [item["name"] for item in accepted],
        "training_view_rms_px": {
            item["name"]: float(error) for item, error in zip(accepted, fit["per_view_rms"])
        },
        "holdout_views": [path.name for path in holdout_paths],
        "rejected_views": rejected,
        "radial_monotonicity": monotonic,
        "detector": "cv2.findChessboardCornersSB(NORMALIZE_IMAGE|EXHAUSTIVE|ACCURACY)",
    }
    write_json(output, result)
    print(json.dumps({"output": str(output), "passed": passed, "rms_px": fit["rms"], "views": len(accepted)}, indent=2))
    return passed


def solve_pnp(corners, camera, distortion, objects):
    generic = cv2.solvePnPGeneric(
        objects,
        corners.reshape(-1, 1, 2),
        camera,
        distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not generic[0]:
        raise ValueError("IPPE PnP failed")
    candidates = []
    for rvec, tvec in zip(generic[1], generic[2]):
        rotation, _ = cv2.Rodrigues(rvec)
        depths = (rotation @ objects.astype(float).T + np.asarray(tvec).reshape(3, 1))[2]
        if np.all(depths > 0):
            projected, _ = cv2.projectPoints(objects, rvec, tvec, camera, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners) ** 2, axis=1))))
            candidates.append((error, rvec, tvec))
    if not candidates:
        raise ValueError("PnP produced no positive-depth board pose")
    _error, rvec, tvec = min(candidates, key=lambda item: item[0])
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(objects, corners.reshape(-1, 1, 2), camera, distortion, rvec, tvec)
    projected, _ = cv2.projectPoints(objects, rvec, tvec, camera, distortion)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners) ** 2, axis=1))))
    rotation, _ = cv2.Rodrigues(rvec)
    return transform(rotation, np.asarray(tvec).reshape(3)), rms


def mean_transform(values):
    rotations = Rotation.from_matrix(np.asarray([value[:3, :3] for value in values])).mean().as_matrix()
    translations = np.mean([value[:3, 3] for value in values], axis=0)
    return transform(rotations, translations)


def board_residuals(A, X, Y, Z):
    result = []
    for a_value, y_value in zip(A, Y):
        current = a_value @ X @ y_value
        delta = invert(Z) @ current
        result.append(
            {
                "translation_m": float(np.linalg.norm(delta[:3, 3])),
                "rotation_rad": rotation_angle(delta[:3, :3]),
            }
        )
    return result


def _pack(value):
    return np.r_[Rotation.from_matrix(value[:3, :3]).as_rotvec(), value[:3, 3]]


def _unpack(value):
    return transform(Rotation.from_rotvec(value[:3]).as_matrix(), value[3:6])


def refine_handeye(A, Y, initial_x, rotation_weight):
    initial_z = mean_transform([a @ initial_x @ y for a, y in zip(A, Y)])

    def residual(parameters):
        x_value, z_value = _unpack(parameters[:6]), _unpack(parameters[6:])
        errors = []
        for a_value, y_value in zip(A, Y):
            delta = invert(z_value) @ a_value @ x_value @ y_value
            errors.extend(delta[:3, 3])
            errors.extend(Rotation.from_matrix(delta[:3, :3]).as_rotvec() * rotation_weight)
        return np.asarray(errors)

    solved = least_squares(
        residual,
        np.r_[_pack(initial_x), _pack(initial_z)],
        loss="soft_l1",
        f_scale=0.005,
        max_nfev=1000,
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12,
    )
    return _unpack(solved.x[:6]), _unpack(solved.x[6:]), solved


def solve_handeye(A, Y, rotation_weight=0.1):
    if len(A) != len(Y) or len(A) < 5:
        raise ValueError("hand-eye calibration requires at least five paired poses")
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    candidates = []
    failures = {}
    for name, method in methods.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                [value[:3, :3] for value in A],
                [value[:3, 3].reshape(3, 1) for value in A],
                [value[:3, :3] for value in Y],
                [value[:3, 3].reshape(3, 1) for value in Y],
                method=method,
            )
            initial = transform(rotation, np.asarray(translation).reshape(3))
            if not np.isfinite(initial).all() or np.linalg.det(initial[:3, :3]) < 0.9:
                raise ValueError("non-finite or improper result")
            z_value = mean_transform([a @ initial @ y for a, y in zip(A, Y)])
            errors = board_residuals(A, initial, Y, z_value)
            score = float(
                np.sqrt(np.mean([item["translation_m"] ** 2 for item in errors]))
                + rotation_weight * np.sqrt(np.mean([item["rotation_rad"] ** 2 for item in errors]))
            )
            candidates.append((score, name, initial))
        except (cv2.error, ValueError) as error:
            failures[name] = str(error)
    if not candidates:
        raise ValueError(f"all hand-eye methods failed: {failures}")
    score, name, initial = min(candidates, key=lambda item: item[0])
    x_value, z_value, optimization = refine_handeye(A, Y, initial, rotation_weight)
    return x_value, z_value, {
        "initial_method": name,
        "initial_score": score,
        "optimizer_success": bool(optimization.success),
        "optimizer_cost": float(optimization.cost),
        "method_failures": failures,
    }


def _pose_records(config, session, names, intrinsics, kinematics):
    camera = np.asarray(intrinsics["camera_matrix"], dtype=float)
    distortion = np.asarray(intrinsics["distortion"], dtype=float)
    objects = board_points(config)
    threshold = float(config.get("quality", {}).get("max_pnp_rms_px", 1.5))
    records, rejected = [], []
    for name in names:
        path = Path(session) / name
        image, frame, corners, metrics = load_view(path, config)
        try:
            if corners is None or not metrics.get("accepted"):
                raise ValueError("corner detection quality failed")
            check_frame_sync(frame, config)
            target_to_camera, pnp_rms = solve_pnp(corners, camera, distortion, objects)
            if pnp_rms > threshold:
                raise ValueError(f"PnP RMS {pnp_rms:.3f} px exceeds {threshold:.3f} px")
            records.append(
                {
                    "name": name,
                    "pitch_to_base": head_matrix(config, kinematics, frame["head_q2"]),
                    "target_to_camera": target_to_camera,
                    "pnp_rms_px": pnp_rms,
                    "head_q2": frame["head_q2"],
                }
            )
        except ValueError as error:
            rejected.append({"view": name, "reason": str(error)})
    return records, rejected


def extrinsics_command(config, session: Path, intrinsics_path: Path, output: Path):
    intrinsics = read_json(intrinsics_path)
    if not intrinsics.get("passed"):
        raise ValueError("intrinsics result has not passed its quality gates")
    check_intrinsics_pattern(config, intrinsics)
    kinematics = robot_kinematics(config)
    training, rejected = _pose_records(
        config, session, intrinsics["training_views"], intrinsics, kinematics
    )
    minimum = int(config.get("quality", {}).get("min_extrinsic_views", 12))
    if len(training) < minimum:
        raise ValueError(f"only {len(training)} extrinsic views passed; require {minimum}: {rejected}")
    heads = np.asarray([item["head_q2"] for item in training], dtype=float)
    spans = np.ptp(heads, axis=0)
    required_span = float(config.get("quality", {}).get("min_head_span_rad", 0.20))
    if np.any(spans < required_span):
        raise ValueError(f"insufficient head excitation: spans={spans.tolist()}, require each >= {required_span}")
    model_consistency = check_model_consistency(
        config,
        kinematics,
        [training[0]["head_q2"], training[len(training) // 2]["head_q2"], training[-1]["head_q2"]],
    )
    A = [item["pitch_to_base"] for item in training]
    Y = [item["target_to_camera"] for item in training]
    rotation_weight = float(config.get("quality", {}).get("rotation_weight_m_per_rad", 0.1))
    x_value, z_value, solver = solve_handeye(A, Y, rotation_weight)
    residuals = board_residuals(A, x_value, Y, z_value)
    max_translation = float(config.get("quality", {}).get("max_board_residual_m", 0.015))
    max_rotation = math.radians(float(config.get("quality", {}).get("max_board_residual_deg", 2.0)))
    keep = [
        index
        for index, item in enumerate(residuals)
        if item["translation_m"] <= max_translation and item["rotation_rad"] <= max_rotation
    ]
    outliers = []
    if len(keep) != len(training):
        if len(keep) < minimum:
            raise ValueError("too many inconsistent hand-eye samples; inspect corner order and synchronization")
        outliers = [
            {"view": training[index]["name"], **residuals[index]}
            for index in range(len(training))
            if index not in keep
        ]
        training = [training[index] for index in keep]
        A = [item["pitch_to_base"] for item in training]
        Y = [item["target_to_camera"] for item in training]
        x_value, z_value, solver = solve_handeye(A, Y, rotation_weight)
        residuals = board_residuals(A, x_value, Y, z_value)
    holdout, holdout_rejected = _pose_records(
        config, session, intrinsics.get("holdout_views", []), intrinsics, kinematics
    )
    holdout_residuals = board_residuals(
        [item["pitch_to_base"] for item in holdout],
        x_value,
        [item["target_to_camera"] for item in holdout],
        z_value,
    ) if holdout else []
    training_report = [
        {"view": item["name"], "pnp_rms_px": item["pnp_rms_px"], **error}
        for item, error in zip(training, residuals)
    ]
    holdout_report = [
        {"view": item["name"], "pnp_rms_px": item["pnp_rms_px"], **error}
        for item, error in zip(holdout, holdout_residuals)
    ]
    passed = bool(
        training_report
        and max(item["translation_m"] for item in training_report) <= max_translation
        and max(item["rotation_rad"] for item in training_report) <= max_rotation
        and holdout_report
        and max(item["translation_m"] for item in holdout_report) <= max_translation
        and max(item["rotation_rad"] for item in holdout_report) <= max_rotation
    )
    nominal = nominal_camera_matrix(config, kinematics)
    nominal_delta = invert(nominal) @ x_value
    result = {
        "schema_version": SCHEMA_VERSION,
        "kind": "sp_vision_pitch_camera_extrinsics",
        "passed": passed,
        "frame_convention": "T_A_B maps coordinates from frame B into frame A",
        "T_pitch_camera": x_value,
        "nominal_T_pitch_camera": nominal,
        "calibrated_from_nominal": {
            "translation_m": float(np.linalg.norm(x_value[:3, 3] - nominal[:3, 3])),
            "rotation_rad": rotation_angle(nominal_delta[:3, :3]),
            "delta_transform": nominal_delta,
        },
        "T_base_board": z_value,
        "head_joint_names": config["robot"]["head_joint_names"],
        "pitch_link": config["robot"]["pitch_link"],
        "training": training_report,
        "holdout": holdout_report,
        "rejected": rejected + holdout_rejected,
        "outliers": outliers,
        "head_span_rad": spans,
        "solver": solver,
        "model_consistency": model_consistency,
        "quality_limits": {
            "max_board_residual_m": max_translation,
            "max_board_residual_deg": math.degrees(max_rotation),
        },
    }
    write_json(output, result)
    summary = {
        "output": str(output),
        "passed": passed,
        "views": len(training_report),
        "holdout_views": len(holdout_report),
        "max_training_mm": 1000 * max(item["translation_m"] for item in training_report),
        "max_holdout_mm": 1000 * max([item["translation_m"] for item in holdout_report] or [float("inf")]),
    }
    print(json.dumps(summary, indent=2))
    return passed


def pivot_command(config, side: str, state_paths, output: Path):
    if len(state_paths) < 4:
        raise ValueError("TCP pivot calibration requires at least four touch states")
    kinematics = robot_kinematics(config)
    wrists = []
    for path in state_paths:
        state = read_json(path)
        wrists.append(wrist_matrix(config, kinematics, state["arm_q14"], side))
    matrix, right = [], []
    for value in wrists:
        matrix.append(np.c_[value[:3, :3], -np.eye(3)])
        right.append(-value[:3, 3])
    solution, _residuals, rank, singular = np.linalg.lstsq(np.vstack(matrix), np.hstack(right), rcond=None)
    if rank < 6:
        raise ValueError("TCP pivot poses are degenerate; use clearly different wrist orientations")
    tip, fixed = solution[:3], solution[3:]
    errors = [float(np.linalg.norm(value[:3, :3] @ tip + value[:3, 3] - fixed)) for value in wrists]
    max_rotation = max(
        rotation_angle(wrists[0][:3, :3].T @ value[:3, :3]) for value in wrists[1:]
    )
    threshold = float(config.get("quality", {}).get("max_tcp_pivot_residual_m", 0.005))
    passed = bool(max_rotation >= math.radians(15) and max(errors) <= threshold)
    result = {
        "schema_version": SCHEMA_VERSION,
        "kind": "sp_vision_tcp_pivot",
        "passed": passed,
        "side": side,
        "tip_in_wrist_m": tip,
        "fixed_point_in_base_m": fixed,
        "residuals_m": errors,
        "max_residual_m": max(errors),
        "max_relative_wrist_rotation_deg": math.degrees(max_rotation),
        "condition_number": float(singular[0] / singular[-1]),
        "states": [str(path) for path in state_paths],
    }
    write_json(output, result)
    print(json.dumps({"output": str(output), "passed": passed, "tip_in_wrist_m": tip.tolist(), "max_error_mm": 1000 * max(errors)}, indent=2))
    return passed


def parse_corner(text: str):
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError(f"corner must be ROW,COLUMN: {text}")
    return int(parts[0]), int(parts[1])


def choose_corners(image, corners, config):
    selected = []
    drawing = draw_detection(image, corners, config, {"accepted": True, "corner_count": len(corners)}, detailed=True)

    def click(event, x, y, _flags, _parameter):
        if event != cv2.EVENT_LBUTTONDOWN or len(selected) >= 3:
            return
        distance = np.linalg.norm(corners - np.array([x, y]), axis=1)
        index = int(np.argmin(distance))
        if index not in selected:
            selected.append(index)

    window = "select 3 non-collinear corners; Enter=finish, r=reset, q=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, click)
    try:
        while True:
            shown = drawing.copy()
            for order, index in enumerate(selected, 1):
                point = tuple(np.rint(corners[index]).astype(int))
                cv2.circle(shown, point, 10, (0, 0, 255), 3, cv2.LINE_AA)
                cv2.putText(shown, str(order), point, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow(window, shown)
            key = cv2.waitKey(50) & 0xFF
            if key in (ord("q"), 27):
                raise ValueError("corner selection cancelled")
            if key == ord("r"):
                selected.clear()
            if key in (10, 13) and len(selected) == 3:
                return selected
    finally:
        cv2.destroyWindow(window)


def selected_corner_indices(config, specified):
    rows, columns = int(config["board"]["rows"]), int(config["board"]["columns"])
    indices = []
    for row, column in specified:
        if not 0 <= row < rows or not 0 <= column < columns:
            raise ValueError(f"corner ({row},{column}) is outside {rows}x{columns}")
        indices.append(row * columns + column)
    if len(indices) != 3 or len(set(indices)) != 3:
        raise ValueError("select exactly three different corners")
    points = board_points(config)[indices, :2]
    first, second = points[1] - points[0], points[2] - points[0]
    twice_area = abs(float(first[0] * second[1] - first[1] * second[0]))
    if twice_area < float(config["board"]["square_m"]) ** 2:
        raise ValueError("the three validation corners are collinear or too close")
    return indices


def reused_corner_indices(config, selection_path):
    previous = read_json(selection_path)
    if previous.get("kind") != "sp_vision_touch_selection":
        raise ValueError("--reuse-selection must be a touch selection JSON")
    chosen = previous.get("corners", [])
    if len(chosen) != 3 or [item.get("touch_order") for item in chosen] != [1, 2, 3]:
        raise ValueError("previous selection must contain three ordered corners")
    return selected_corner_indices(
        config, [(int(item["row"]), int(item["column"])) for item in chosen]
    )


def select_validation_command(config, frame_dir, intrinsics_path, extrinsics_path, output, specified,
                              reuse_selection=None):
    if specified and reuse_selection:
        raise ValueError("choose either --corner or --reuse-selection")
    intrinsics, extrinsics = read_json(intrinsics_path), read_json(extrinsics_path)
    if not intrinsics.get("passed") or not extrinsics.get("passed"):
        raise ValueError("intrinsics and extrinsics must pass before touch validation")
    check_intrinsics_pattern(config, intrinsics)
    image, frame, corners, metrics = load_view(Path(frame_dir), config)
    if corners is None or not metrics.get("accepted"):
        raise ValueError(f"validation checkerboard detection failed: {metrics}")
    check_frame_sync(frame, config)
    camera = np.asarray(intrinsics["camera_matrix"], dtype=float)
    distortion = np.asarray(intrinsics["distortion"], dtype=float)
    objects = board_points(config)
    target_to_camera, pnp_rms = solve_pnp(corners, camera, distortion, objects)
    threshold = float(config.get("quality", {}).get("max_pnp_rms_px", 1.5))
    if pnp_rms > threshold:
        raise ValueError(f"validation PnP RMS {pnp_rms:.3f} px exceeds {threshold:.3f} px")
    if reuse_selection:
        indices = reused_corner_indices(config, reuse_selection)
    elif specified:
        indices = selected_corner_indices(config, specified)
    else:
        indices = choose_corners(image, corners, config)
        rows_columns = [divmod(index, int(config["board"]["columns"])) for index in indices]
        indices = selected_corner_indices(config, rows_columns)
    kinematics = robot_kinematics(config)
    pitch_to_base = head_matrix(config, kinematics, frame["head_q2"])
    camera_to_pitch = np.asarray(extrinsics["T_pitch_camera"], dtype=float)
    target_to_base = pitch_to_base @ camera_to_pitch @ target_to_camera
    records = []
    marked = draw_detection(image, corners, config, metrics, detailed=True)
    columns = int(config["board"]["columns"])
    for order, index in enumerate(indices, 1):
        row, column = divmod(index, columns)
        board_point = np.r_[objects[index].astype(float), 1.0]
        camera_point = target_to_camera @ board_point
        base_point = target_to_base @ board_point
        records.append(
            {
                "touch_order": order,
                "row": row,
                "column": column,
                "pixel_uv": corners[index],
                "point_camera_m": camera_point[:3],
                "point_base_predicted_m": base_point[:3],
            }
        )
        point = tuple(np.rint(corners[index]).astype(int))
        cv2.circle(marked, point, 12, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.putText(marked, str(order), (point[0] + 10, point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    marked_path = Path(output).with_suffix(".png")
    cv2.imwrite(str(marked_path), marked)
    result = {
        "schema_version": SCHEMA_VERSION,
        "kind": "sp_vision_touch_selection",
        "frame_dir": str(Path(frame_dir).resolve()),
        "head_q2": frame["head_q2"],
        "pnp_rms_px": pnp_rms,
        "T_camera_board": target_to_camera,
        "T_base_board": target_to_base,
        "corners": records,
        "marked_image": str(marked_path.resolve()),
    }
    if reuse_selection:
        result["reused_selection"] = str(Path(reuse_selection).resolve())
    write_json(output, result)
    print(json.dumps({"output": str(output), "marked_image": str(marked_path), "corners": [[item["row"], item["column"]] for item in records]}, indent=2))


def tcp_tip(config, tcp_path):
    if tcp_path:
        result = read_json(tcp_path)
        if not result.get("passed"):
            raise ValueError("TCP pivot result did not pass")
        value = result["tip_in_wrist_m"]
    else:
        value = config.get("validation", {}).get("tip_in_wrist_m")
    tip = np.asarray(value, dtype=float) if value is not None else np.array([])
    if tip.shape != (3,) or not np.isfinite(tip).all():
        raise ValueError("provide a passed --tcp result or validation.tip_in_wrist_m")
    return tip


def validate_command(config, selection_path, side, state_paths, tcp_path, output):
    selection = read_json(selection_path)
    chosen = selection.get("corners", [])
    if len(chosen) != 3 or len(state_paths) != 3:
        raise ValueError("touch validation requires exactly three selected corners and three state files")
    tip = tcp_tip(config, tcp_path)
    kinematics = robot_kinematics(config)
    expected_head = np.asarray(selection["head_q2"], dtype=float)
    records = []
    for chosen_point, state_path in zip(chosen, state_paths):
        state = read_json(state_path)
        head_change = None
        if "head_q2" in state:
            head_change = float(
                np.max(np.abs(np.asarray(state["head_q2"], dtype=float) - expected_head))
            )
        wrist = wrist_matrix(config, kinematics, state["arm_q14"], side)
        touched = wrist @ np.r_[tip, 1.0]
        predicted = np.asarray(chosen_point["point_base_predicted_m"], dtype=float)
        error = float(np.linalg.norm(touched[:3] - predicted))
        records.append(
            {
                "row": chosen_point["row"],
                "column": chosen_point["column"],
                "state": str(state_path),
                "point_base_predicted_m": predicted,
                "point_base_touched_m": touched[:3],
                "error_m": error,
                "head_change_after_image_rad": head_change,
            }
        )
    threshold = float(config.get("quality", {}).get("max_touch_error_m", 0.010))
    passed = bool(max(item["error_m"] for item in records) <= threshold)
    result = {
        "schema_version": SCHEMA_VERSION,
        "kind": "sp_vision_touch_validation",
        "passed": passed,
        "side": side,
        "tip_in_wrist_m": tip,
        "points": records,
        "mean_error_m": float(np.mean([item["error_m"] for item in records])),
        "max_error_m": max(item["error_m"] for item in records),
        "limit_m": threshold,
        "max_head_change_after_image_rad": max(
            [item["head_change_after_image_rad"] for item in records
             if item["head_change_after_image_rad"] is not None]
            or [0.0]
        ),
        "head_motion_note": (
            "Head motion after the validation image is diagnostic only: the prediction uses "
            "the image-synchronized head_q2. The checkerboard must remain fixed."
        ),
        "comparison": "Cartesian points in base_link; joint angles are not compared",
    }
    write_json(output, result)
    print(json.dumps({
        "output": str(output),
        "passed": passed,
        "errors_mm": [1000 * item["error_m"] for item in records],
        "max_error_mm": 1000 * result["max_error_m"],
        "max_head_change_after_image_rad": result["max_head_change_after_image_rad"],
    }, indent=2))
    return passed


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent / "configs" / "head_config.example.json",
        help="head calibration JSON config (defaults to this directory)",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    capture = commands.add_parser("capture", help="capture accepted moving-head checkerboard samples")
    add_board_arguments(capture)
    capture.add_argument("--session", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session")
    capture.add_argument("--count", type=int, default=40, help="views to capture in this run")
    capture.add_argument("--append", action="store_true", help="append views instead of replacing the session's views")

    intrinsics = commands.add_parser("intrinsics", help="fit camera intrinsics")
    add_board_arguments(intrinsics)
    intrinsics.add_argument("--session", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session")
    intrinsics.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session" / "head_intrinsics.json")

    extrinsics = commands.add_parser("extrinsics", help="fit camera-to-pitch transform")
    add_board_arguments(extrinsics)
    extrinsics.add_argument("--session", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session")
    extrinsics.add_argument("--intrinsics", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session" / "head_intrinsics.json")
    extrinsics.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session" / "head_extrinsics.json")

    pivot = commands.add_parser("pivot", help="fit TCP tip from repeated fixed-point touches")
    pivot.add_argument("--side", choices=("left", "right"), required=True)
    pivot.add_argument("--states", required=True, nargs="+", type=Path)
    pivot.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session" / "tcp" / "tcp_pivot.json")

    selection = commands.add_parser("select-validation", help="select three checkerboard touch points")
    add_board_arguments(selection)
    selection.add_argument("--frame", required=True, type=Path)
    selection.add_argument("--intrinsics", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session" / "head_intrinsics.json")
    selection.add_argument("--extrinsics", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_session" / "head_extrinsics.json")
    selection.add_argument("--corner", action="append", type=parse_corner, help="ROW,COLUMN; repeat exactly 3 times")
    selection.add_argument("--reuse-selection", type=Path, help="reuse ordered corners from an earlier touch selection")
    selection.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_validation" / "head_selection.json")

    validate = commands.add_parser("validate", help="compare predicted corners with touched TCP points")
    validate.add_argument("--selection", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_validation" / "head_selection.json")
    validate.add_argument("--side", choices=("left", "right"), required=True)
    validate.add_argument("--states", required=True, nargs=3, type=Path)
    validate.add_argument("--tcp", type=Path, help="passed pivot JSON; otherwise use config value")
    validate.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "data" / "head_camera_validation" / "head_validation.json")
    return parser


def add_board_arguments(parser):
    parser.add_argument(
        "--pattern",
        help="inner corners as COLSxROWS; overrides board values in JSON, for example 7x10",
    )
    parser.add_argument(
        "--square-m",
        type=float,
        help="measured checkerboard square side in metres; overrides JSON",
    )


def main(argv=None):
    args = build_parser().parse_args(argv)
    config = load_config(args.config)
    apply_board_arguments(config, args)
    if args.command == "capture":
        if args.count <= 0:
            raise ValueError("--count must be positive")
        capture_command(config, args.session, args.count, append=args.append)
        return 0
    if args.command == "intrinsics":
        return 0 if intrinsics_command(config, args.session, args.output) else 1
    if args.command == "extrinsics":
        return 0 if extrinsics_command(config, args.session, args.intrinsics, args.output) else 1
    if args.command == "pivot":
        return 0 if pivot_command(config, args.side, args.states, args.output) else 1
    if args.command == "select-validation":
        select_validation_command(
            config, args.frame, args.intrinsics, args.extrinsics, args.output, args.corner,
            reuse_selection=args.reuse_selection
        )
        return 0
    if args.command == "validate":
        return 0 if validate_command(
            config, args.selection, args.side, args.states, args.tcp, args.output
        ) else 1
    raise AssertionError(args.command)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError, cv2.error) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
