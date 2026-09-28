#!/usr/bin/env python3
"""Right wrist camera calibration, with ROS 2 acquisition and offline solves."""

from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import calibration as common


DEFAULT_SESSION = Path(__file__).resolve().parent / "data" / "wrist_camera_session"


def load_config(path: Path) -> dict:
    config = common.load_config(path)
    robot = config["robot"]
    if robot["mount_link"] != "wrist_roll_R_Link":
        raise ValueError("the right camera's fixed mount must be wrist_roll_R_Link")
    source = robot["ros2_joint_names"]["left"] + robot["ros2_joint_names"]["right"]
    target = robot["arm_joint_names"]["left"] + robot["arm_joint_names"]["right"]
    if len(source) != 14 or len(set(source)) != 14 or len(target) != 14 or len(set(target)) != 14:
        raise ValueError("ROS 2 and URDF arm joint lists must each contain 14 unique names")
    return config


def arm_state(config: dict, joints: dict) -> list[float]:
    names = config["robot"]["ros2_joint_names"]
    source = names["left"] + names["right"]
    missing = [name for name in source if name not in joints]
    if missing:
        raise ValueError(f"ROS 2 joint state lacks {missing}")
    values = np.asarray([joints[name] for name in source], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("ROS 2 arm state contains non-finite positions")
    return values.tolist()


def capture_pair(config: dict) -> tuple[np.ndarray, dict]:
    settings = config["capture"]
    topic = settings["color_topic"]
    if topic != "/camera/right/color/image_resized/compressed":
        raise ValueError("this right wrist workflow requires the right resized color topic")
    timeout = float(settings.get("timeout_s", 8))
    if not 0 < timeout <= 60:
        raise ValueError("capture.timeout_s must be in (0, 60]")
    remote = f"""source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID={int(settings.get('ros_domain_id', 0))}
python3 - <<'PY'
import base64
import json
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState

rclpy.init()
node = rclpy.create_node('sp_vision_wrist_pair')
images, states = [], []
def stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
def image_cb(message):
    images.append((stamp(message), message.header.frame_id, bytes(message.data)))
    del images[:-20]
def state_cb(message):
    states.append((stamp(message), dict(zip(message.name, message.position))))
    del states[:-100]
image_sub = node.create_subscription(CompressedImage, {json.dumps(topic)}, image_cb, qos_profile_sensor_data)
state_sub = node.create_subscription(JointState, {json.dumps(settings.get('joint_topic', '/joint_states'))}, state_cb, qos_profile_sensor_data)
deadline = time.monotonic() + {timeout!r}
limit_ns = {int(settings.get('max_state_skew_ms', 100) * 1000000)!r}
selected = None
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if images and states:
        image = images[-1]
        state = min(states, key=lambda item: abs(item[0] - image[0]))
        if abs(state[0] - image[0]) <= limit_ns and abs(time.time_ns() - image[0]) <= 1000000000:
            selected = (image, state)
            break
if selected is None:
    print('no fresh right wrist image and synchronized joint state', file=sys.stderr)
    sys.exit(2)
image, state = selected
print(json.dumps({{'image_b64': base64.b64encode(image[2]).decode('ascii'),
                  'image_stamp_ns': image[0], 'joint_stamp_ns': state[0],
                  'frame_id': image[1], 'joints': state[1]}}))
node.destroy_node()
rclpy.shutdown()
PY
"""
    result = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", settings["host"], "bash -s"],
        input=remote.encode(), capture_output=True, timeout=timeout + 10, check=False,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors="replace").strip() or
                           f"remote capture exited {result.returncode}")
    payload = json.loads(result.stdout)
    image = cv2.imdecode(np.frombuffer(base64.b64decode(payload.pop("image_b64")), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("right wrist JPEG could not be decoded")
    return image, payload


def check_frame(config: dict, frame: dict) -> list[float]:
    sync = frame.get("sensor_sync", {})
    image_ns, joint_ns = int(sync["image_stamp_ns"]), int(sync["joint_stamp_ns"])
    limit = float(config["capture"].get("max_state_skew_ms", 100))
    if abs(image_ns - joint_ns) / 1e6 > limit:
        raise ValueError("image and joint state exceed the synchronization limit")
    if frame.get("camera_frame_id") != "right_color_optical_frame":
        raise ValueError("unexpected camera frame ID")
    joints = sync.get("joints", {})
    arm = arm_state(config, joints)
    if not np.allclose(arm, frame["arm_q14"], atol=1e-9, rtol=0):
        raise ValueError("saved arm_q14 does not match the named ROS 2 joints")
    return arm


def mount_matrix(config: dict, model, arm_q14) -> np.ndarray:
    positions = common.arm_positions(config, arm_q14)
    return model.matrix(config["robot"]["base_link"], config["robot"]["mount_link"], positions)


def nominal_matrix(config: dict, model) -> np.ndarray:
    return model.matrix(config["robot"]["mount_link"], config["robot"]["camera_frame"], {})


def check_model_consistency(config: dict, model) -> dict:
    robot = config["robot"]
    scene = common.MjcfKinematics(common.config_path(config, robot["model_xml"]))
    urdf = nominal_matrix(config, model)
    mjcf = scene.matrix(robot["mount_link"], robot["camera_frame"], {})
    translation = float(np.linalg.norm(urdf[:3, 3] - mjcf[:3, 3]))
    rotation = math.degrees(common.rotation_angle(urdf[:3, :3].T @ mjcf[:3, :3]))
    if translation > 1e-5 or rotation > .01:
        raise ValueError(f"URDF and MJCF right camera chains differ: {translation} m, {rotation} deg")
    return {"passed": True, "translation_m": translation, "rotation_deg": rotation}


def capture_command(config: dict, session: Path, count: int, append: bool) -> None:
    if count < 1:
        raise ValueError("--count must be positive")
    session = Path(session)
    session.mkdir(parents=True, exist_ok=True)
    existing = common.enumerate_views(session) if append else []
    index = max([int(path.name.rsplit("-", 1)[1]) for path in existing] or [0]) + 1
    with tempfile.TemporaryDirectory(prefix=f".{session.name}-capture-", dir=session.parent) as temporary:
        staged = Path(temporary) / "new"
        staged.mkdir()
        print("Keys: s=save, f=flip checkerboard order, r/space=recapture, q=quit")
        saved = 0
        try:
            while saved < count:
                image, sample = capture_pair(config)
                frame = {
                    "schema_version": 1,
                    "camera_id": "right_wrist_color",
                    "camera_frame_id": sample["frame_id"],
                    "timestamp_s": time.time(),
                    "arm_q14": arm_state(config, sample["joints"]),
                    "sensor_sync": sample,
                    "pattern": dict(config["board"]),
                    "corner_order_reversed_180": False,
                }
                flip = False
                while True:
                    corners, metrics = common.detect_board(image, config, flip)
                    frame["corner_order_reversed_180"] = flip
                    frame["detection"] = metrics
                    marked = common.draw_detection(image, corners, config, metrics)
                    cv2.imshow("sp_vision right wrist calibration", marked)
                    key = cv2.waitKey(0) & 0xFF
                    if key in (ord("q"), 27):
                        return
                    if key == ord("f") and corners is not None:
                        flip = not flip
                        continue
                    if key == ord("s"):
                        if not metrics.get("accepted"):
                            print(f"rejected: {metrics}")
                            continue
                        path = (session if append else staged) / f"view-{index:03d}"
                        path.mkdir()
                        if not cv2.imwrite(str(path / "color.png"), image):
                            raise OSError(f"cannot save {path / 'color.png'}")
                        if not cv2.imwrite(str(path / "corners.png"), marked):
                            raise OSError(f"cannot save {path / 'corners.png'}")
                        common.write_json(path / "frame.json", frame)
                        print(f"saved {path}")
                        index += 1
                        saved += 1
                    break
        finally:
            cv2.destroyAllWindows()
        if not append:
            common._replace_capture_views(session, staged)
            print("Replaced previous view-* images; rerun selection or camera fits that used them.")


def vendor_arm_state(config: dict) -> list[float]:
    """Read the same controller arm_q14 used by the head touch validation."""
    try:
        from tron2_deployment.config import load_profile
        from tron2_deployment.robot import WebsocketRobot
    except ImportError as error:
        raise RuntimeError(
            "controller state probing requires the optional tron2_deployment "
            "adapter; offline wrist calibration does not."
        ) from error

    profile = load_profile(common.config_path(config, config["capture"]["state_profile"]))
    adapter = WebsocketRobot(profile)
    try:
        return adapter.read_state()["arm_q14"]
    finally:
        adapter.close()


def probe_command(config: dict, output: Path) -> None:
    before = None
    vendor_error = None
    try:
        before = vendor_arm_state(config)
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        vendor_error = str(error)
    image, sample = capture_pair(config)
    after = None
    if before is not None:
        try:
            after = vendor_arm_state(config)
        except (OSError, RuntimeError, ValueError, KeyError) as error:
            vendor_error = str(error)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    image_path = output.with_suffix(".png")
    if not cv2.imwrite(str(image_path), image):
        raise OSError(f"cannot save {image_path}")
    arm = arm_state(config, sample["joints"])
    mapping_check = {"available": before is not None and after is not None}
    if mapping_check["available"]:
        before_values, after_values, ros_values = map(np.asarray, (before, after, arm))
        stationary = float(np.max(np.abs(after_values - before_values)))
        difference = float(max(np.max(np.abs(ros_values - before_values)),
                               np.max(np.abs(ros_values - after_values))))
        mapping_check.update({"passed": stationary <= .005 and difference <= .005,
                              "vendor_motion_rad": stationary,
                              "max_ros2_vendor_difference_rad": difference,
                              "comparison": "ROS 2 named joints versus the controller arm_q14 used by head touch validation"})
    else:
        mapping_check["reason"] = vendor_error or "controller state was unavailable"
    report = {"schema_version": 1, "camera_frame_id": sample["frame_id"],
              "image_stamp_ns": sample["image_stamp_ns"], "joint_stamp_ns": sample["joint_stamp_ns"],
              "state_skew_ms": abs(sample["image_stamp_ns"] - sample["joint_stamp_ns"]) / 1e6,
              "ros2_named_joints": sample["joints"], "arm_q14": arm,
              "mapping_check": mapping_check,
              "image": str(image_path.resolve())}
    common.write_json(output, report)
    print(json.dumps({"output": str(output), "image": str(image_path),
                      "state_skew_ms": report["state_skew_ms"], "mapping_check": mapping_check}, indent=2))


def pose_records(config, session, names, intrinsics, model):
    camera = np.asarray(intrinsics["camera_matrix"], dtype=float)
    distortion = np.asarray(intrinsics["distortion"], dtype=float)
    objects = common.board_points(config)
    records, rejected = [], []
    for name in names:
        try:
            image, frame, corners, metrics = common.load_view(session / name, config)
            arm = check_frame(config, frame)
            if corners is None or not metrics.get("accepted"):
                raise ValueError("checkerboard detection failed")
            board_to_camera, rms = common.solve_pnp(corners, camera, distortion, objects)
            if rms > float(config["quality"]["max_pnp_rms_px"]):
                raise ValueError(f"PnP RMS {rms:.3f} px exceeds limit")
            records.append({"name": name, "A": mount_matrix(config, model, arm),
                            "Y": board_to_camera, "pnp_rms_px": rms, "arm_q14": arm})
        except (KeyError, ValueError) as error:
            rejected.append({"view": name, "reason": str(error)})
    return records, rejected


def intrinsics_command(config, session, output):
    views = common.enumerate_views(session)
    if not views:
        raise ValueError("wrist camera session has no saved views")
    for path in views:
        frame = common.read_json(path / "frame.json")
        if frame.get("camera_frame_id") != "right_color_optical_frame":
            raise ValueError(f"{path} is not a right wrist color frame")
    passed = common.intrinsics_command(config, session, output)
    result = common.read_json(output)
    result["camera_frame"] = config["robot"]["camera_frame"]
    result["color_topic"] = config["capture"]["color_topic"]
    common.write_json(output, result)
    return passed


def extrinsics_command(config, session, intrinsics_path, output):
    intrinsics = common.read_json(intrinsics_path)
    if not intrinsics.get("passed"):
        raise ValueError("intrinsics quality gates did not pass")
    if intrinsics.get("camera_frame") != config["robot"]["camera_frame"] or intrinsics.get("color_topic") != config["capture"]["color_topic"]:
        raise ValueError("intrinsics result does not belong to this wrist camera topic")
    common.check_intrinsics_pattern(config, intrinsics)
    model = common.UrdfKinematics(common.config_path(config, config["robot"]["urdf"]))
    model_consistency = check_model_consistency(config, model)
    training, rejected = pose_records(config, session, intrinsics["training_views"], intrinsics, model)
    minimum = int(config["quality"].get("min_extrinsic_views", 12))
    if len(training) < minimum:
        raise ValueError(f"only {len(training)} valid training views, require {minimum}: {rejected}")
    rotations = [item["A"][:3, :3] for item in training]
    spread = max(math.degrees(common.rotation_angle(rotations[0].T @ rotation)) for rotation in rotations[1:])
    if spread < float(config["quality"].get("min_mount_rotation_span_deg", 20)):
        raise ValueError(f"insufficient wrist orientation spread: {spread:.1f} deg")
    rotation_vectors = np.asarray([Rotation.from_matrix(rotations[0].T @ rotation).as_rotvec()
                                   for rotation in rotations[1:]])
    secondary = float(np.linalg.svd(rotation_vectors, compute_uv=False)[1])
    if secondary < float(config["quality"].get("min_secondary_rotation_rad", .12)):
        raise ValueError("wrist rotations are nearly about one axis; vary at least two joint axes")
    A, Y = [item["A"] for item in training], [item["Y"] for item in training]
    weight = float(config["quality"].get("rotation_weight_m_per_rad", 0.1))
    X, Z, solver = common.solve_handeye(A, Y, weight)
    residuals = common.board_residuals(A, X, Y, Z)
    limit_m = float(config["quality"]["max_board_residual_m"])
    limit_rad = math.radians(float(config["quality"]["max_board_residual_deg"]))
    kept = [i for i, item in enumerate(residuals) if item["translation_m"] <= limit_m and item["rotation_rad"] <= limit_rad]
    outliers = [{"view": training[i]["name"], **item} for i, item in enumerate(residuals) if i not in kept]
    if len(kept) < minimum:
        raise ValueError("too many inconsistent wrist hand-eye samples")
    if len(kept) < len(training):
        training = [training[i] for i in kept]
        A, Y = [item["A"] for item in training], [item["Y"] for item in training]
        X, Z, solver = common.solve_handeye(A, Y, weight)
        residuals = common.board_residuals(A, X, Y, Z)
    holdout, holdout_rejected = pose_records(config, session, intrinsics.get("holdout_views", []), intrinsics, model)
    holdout_errors = common.board_residuals([item["A"] for item in holdout], X, [item["Y"] for item in holdout], Z) if holdout else []
    train_report = [{"view": item["name"], "pnp_rms_px": item["pnp_rms_px"], **error} for item, error in zip(training, residuals)]
    holdout_report = [{"view": item["name"], "pnp_rms_px": item["pnp_rms_px"], **error} for item, error in zip(holdout, holdout_errors)]
    passed = bool(solver["optimizer_success"] and holdout_report and all(
        item["translation_m"] <= limit_m and item["rotation_rad"] <= limit_rad
        for item in train_report + holdout_report))
    nominal = nominal_matrix(config, model)
    result = {"schema_version": 1, "kind": "sp_vision_right_wrist_extrinsics", "passed": passed,
              "mount_link": config["robot"]["mount_link"], "camera_frame": config["robot"]["camera_frame"],
              "T_wrist_roll_camera": X, "T_base_board": Z, "nominal_T_wrist_roll_camera": nominal,
              "calibrated_from_nominal": {"translation_m": float(np.linalg.norm(X[:3, 3] - nominal[:3, 3])),
                                          "rotation_deg": math.degrees(common.rotation_angle(nominal[:3, :3].T @ X[:3, :3]))},
              "training": train_report, "holdout": holdout_report,
              "rejected": rejected + holdout_rejected, "outliers": outliers,
              "mount_rotation_span_deg": spread, "secondary_rotation_rad": secondary,
              "solver": solver}
    result["model_consistency"] = model_consistency
    common.write_json(output, result)
    print(json.dumps({"output": str(output), "passed": passed, "training": len(train_report),
                      "holdout": len(holdout_report),
                      "max_training_mm": 1000 * max(item["translation_m"] for item in train_report),
                      "max_holdout_mm": 1000 * max([item["translation_m"] for item in holdout_report] or [float("inf")]),
                      "mount_rotation_span_deg": spread}, indent=2))
    return passed


def select_validation_command(config, frame_dir, intrinsics_path, extrinsics_path, output, specified, reuse):
    intrinsics, extrinsics = common.read_json(intrinsics_path), common.read_json(extrinsics_path)
    if not intrinsics.get("passed") or not extrinsics.get("passed"):
        raise ValueError("intrinsics and extrinsics must pass before validation")
    if intrinsics.get("camera_frame") != config["robot"]["camera_frame"] or intrinsics.get("color_topic") != config["capture"]["color_topic"]:
        raise ValueError("intrinsics result does not belong to this wrist camera topic")
    if extrinsics.get("kind") != "sp_vision_right_wrist_extrinsics" or extrinsics.get("mount_link") != config["robot"]["mount_link"]:
        raise ValueError("extrinsics result does not belong to this wrist camera")
    common.check_intrinsics_pattern(config, intrinsics)
    image, frame, corners, metrics = common.load_view(frame_dir, config)
    if [image.shape[1], image.shape[0]] != intrinsics["image_size"]:
        raise ValueError("validation image size differs from wrist intrinsics")
    arm = check_frame(config, frame)
    if corners is None or not metrics.get("accepted"):
        raise ValueError("validation checkerboard detection failed")
    board_to_camera, rms = common.solve_pnp(corners, np.asarray(intrinsics["camera_matrix"], float),
                                            np.asarray(intrinsics["distortion"], float), common.board_points(config))
    if rms > float(config["quality"]["max_pnp_rms_px"]):
        raise ValueError("validation PnP RMS exceeds limit")
    if specified and reuse:
        raise ValueError("choose --corner or --reuse-selection")
    if reuse:
        indices = common.reused_corner_indices(config, reuse)
    elif specified:
        indices = common.selected_corner_indices(config, specified)
    else:
        indices = common.choose_corners(image, corners, config)
    model = common.UrdfKinematics(common.config_path(config, config["robot"]["urdf"]))
    base_board = mount_matrix(config, model, arm) @ np.asarray(extrinsics["T_wrist_roll_camera"]) @ board_to_camera
    marked = common.draw_detection(image, corners, config, metrics, detailed=True)
    columns = int(config["board"]["columns"])
    points = []
    for order, index in enumerate(indices, 1):
        point = base_board @ np.r_[common.board_points(config)[index], 1]
        row, column = divmod(index, columns)
        points.append({"touch_order": order, "row": row, "column": column,
                       "pixel_uv": corners[index], "point_base_predicted_m": point[:3]})
        uv = tuple(np.rint(corners[index]).astype(int))
        cv2.circle(marked, uv, 12, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.putText(marked, str(order), (uv[0] + 10, uv[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)
    marked_path = Path(output).with_suffix(".png")
    marked_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(marked_path), marked):
        raise OSError(f"cannot save {marked_path}")
    common.write_json(output, {"schema_version": 1, "kind": "sp_vision_wrist_touch_selection",
                               "frame_dir": str(Path(frame_dir).resolve()), "arm_q14": arm,
                               "T_camera_board": board_to_camera, "T_base_board": base_board,
                               "pnp_rms_px": rms, "corners": points, "marked_image": str(marked_path.resolve())})
    print(json.dumps({"output": str(output), "corners": [[p["row"], p["column"]] for p in points]}, indent=2))


def validate_command(config, selection_path, side, state_paths, tcp_path, output):
    selection = common.read_json(selection_path)
    if selection.get("kind") != "sp_vision_wrist_touch_selection":
        raise ValueError("selection was not made from a wrist camera image")
    points = selection.get("corners", [])
    if len(points) != 3 or len(state_paths) != 3:
        raise ValueError("three selected corners and three touch states are required")
    if tcp_path:
        pivot = common.read_json(tcp_path)
        if pivot.get("kind") != "sp_vision_tcp_pivot" or pivot.get("side") != side:
            raise ValueError("TCP pivot result must match the selected touch arm")
    tip = common.tcp_tip(config, tcp_path)
    model = common.UrdfKinematics(common.config_path(config, config["robot"]["urdf"]))
    results = []
    for point, path in zip(points, state_paths):
        state = common.read_json(path)
        touched = common.wrist_matrix(config, model, state["arm_q14"], side) @ np.r_[tip, 1]
        predicted = np.asarray(point["point_base_predicted_m"], float)
        error = float(np.linalg.norm(touched[:3] - predicted))
        results.append({"row": point["row"], "column": point["column"], "state": str(path),
                        "point_base_predicted_m": predicted, "point_base_touched_m": touched[:3], "error_m": error})
    limit = float(config["quality"].get("max_touch_error_m", .01))
    passed = max(item["error_m"] for item in results) <= limit
    common.write_json(output, {"schema_version": 1, "kind": "sp_vision_wrist_touch_validation",
                               "passed": passed, "side": side, "tip_in_wrist_m": tip, "points": results,
                               "max_error_m": max(item["error_m"] for item in results), "limit_m": limit})
    print(json.dumps({"output": str(output), "passed": passed,
                      "errors_mm": [1000 * item["error_m"] for item in results]}, indent=2))
    return passed


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--config", type=Path, default=Path(__file__).resolve().parent / "configs" / "wrist_config.example.json")
    commands = result.add_subparsers(dest="command", required=True)
    probe = commands.add_parser("probe", help="save one camera frame and named ROS 2 joint state")
    probe.add_argument("--output", type=Path, default=DEFAULT_SESSION / "state-probe.json")
    capture = commands.add_parser("capture")
    common.add_board_arguments(capture)
    capture.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    capture.add_argument("--count", type=int, default=40)
    capture.add_argument("--append", action="store_true")
    intrinsic = commands.add_parser("intrinsics")
    common.add_board_arguments(intrinsic)
    intrinsic.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    intrinsic.add_argument("--output", type=Path, default=DEFAULT_SESSION / "wrist_intrinsics.json")
    extrinsic = commands.add_parser("extrinsics")
    common.add_board_arguments(extrinsic)
    extrinsic.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    extrinsic.add_argument("--intrinsics", type=Path, default=DEFAULT_SESSION / "wrist_intrinsics.json")
    extrinsic.add_argument("--output", type=Path, default=DEFAULT_SESSION / "wrist_extrinsics.json")
    pivot = commands.add_parser("pivot", help="fit a right-arm TCP tip from fixed-point touch states")
    pivot.add_argument("--states", required=True, nargs="+", type=Path)
    pivot.add_argument("--output", type=Path, default=DEFAULT_SESSION / "tcp" / "wrist_tcp_pivot.json")
    selection = commands.add_parser("select-validation")
    common.add_board_arguments(selection)
    selection.add_argument("--frame", required=True, type=Path)
    selection.add_argument("--intrinsics", type=Path, default=DEFAULT_SESSION / "wrist_intrinsics.json")
    selection.add_argument("--extrinsics", type=Path, default=DEFAULT_SESSION / "wrist_extrinsics.json")
    selection.add_argument("--corner", action="append", type=common.parse_corner)
    selection.add_argument("--reuse-selection", type=Path)
    selection.add_argument("--output", type=Path, default=DEFAULT_SESSION / "wrist_selection.json")
    validation = commands.add_parser("validate")
    validation.add_argument("--selection", type=Path, default=DEFAULT_SESSION / "wrist_selection.json")
    validation.add_argument("--side", choices=("left", "right"), default="right")
    validation.add_argument("--states", required=True, type=Path, nargs=3)
    validation.add_argument("--tcp", type=Path)
    validation.add_argument("--output", type=Path, default=DEFAULT_SESSION / "wrist_validation.json")
    return result


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    config = load_config(args.config)
    common.apply_board_arguments(config, args)
    if args.command == "probe":
        probe_command(config, args.output)
    elif args.command == "capture":
        capture_command(config, args.session, args.count, args.append)
    elif args.command == "intrinsics":
        return 0 if intrinsics_command(config, args.session, args.output) else 1
    elif args.command == "extrinsics":
        return 0 if extrinsics_command(config, args.session, args.intrinsics, args.output) else 1
    elif args.command == "pivot":
        return 0 if common.pivot_command(config, "right", args.states, args.output) else 1
    elif args.command == "select-validation":
        select_validation_command(config, args.frame, args.intrinsics, args.extrinsics,
                                  args.output, args.corner, args.reuse_selection)
    elif args.command == "validate":
        return 0 if validate_command(config, args.selection, args.side, args.states,
                                     args.tcp, args.output) else 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
