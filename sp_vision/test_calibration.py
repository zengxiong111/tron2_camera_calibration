from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
import calibration as subject


def _pose(rotvec, translation):
    return subject.transform(Rotation.from_rotvec(rotvec).as_matrix(), translation)


def test_sector_detector_finds_seven_by_ten_inner_corners():
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    square = 44
    columns, rows = 8, 11  # squares; therefore 7 x 10 internal corners
    board = np.full((rows * square, columns * square), 255, dtype=np.uint8)
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2 == 0:
                cv2.rectangle(
                    board,
                    (column * square, row * square),
                    ((column + 1) * square, (row + 1) * square),
                    0,
                    -1,
                )
    canvas = np.full((720, 960), 255, dtype=np.uint8)
    source = np.float32([[0, 0], [board.shape[1] - 1, 0], [board.shape[1] - 1, board.shape[0] - 1], [0, board.shape[0] - 1]])
    destination = np.float32([[220, 100], [735, 145], [690, 650], [175, 590]])
    homography = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(board, homography, (canvas.shape[1], canvas.shape[0]), borderValue=255)
    mask = cv2.warpPerspective(np.full_like(board, 255), homography, (canvas.shape[1], canvas.shape[0]))
    canvas[mask > 0] = warped[mask > 0]
    image = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    corners, metrics = subject.detect_board(image, config)
    assert corners.shape == (70, 2)
    assert metrics["accepted"]
    assert metrics["topology_ok"]


def test_handeye_recovers_camera_to_pitch_transform():
    rng = np.random.default_rng(7)
    expected_x = _pose([0.08, -0.15, 0.04], [0.035, -0.020, 0.065])
    expected_z = _pose([0.20, 0.10, -0.10], [0.70, 0.05, 0.90])
    pitch_to_base, target_to_camera = [], []
    for _ in range(24):
        current = _pose(rng.normal(0, 0.4, 3), rng.normal(0, 0.1, 3))
        pitch_to_base.append(current)
        target_to_camera.append(subject.invert(expected_x) @ subject.invert(current) @ expected_z)
    solved_x, solved_z, result = subject.solve_handeye(pitch_to_base, target_to_camera)
    assert result["optimizer_success"]
    assert np.linalg.norm(solved_x[:3, 3] - expected_x[:3, 3]) < 1e-8
    assert subject.rotation_angle(solved_x[:3, :3].T @ expected_x[:3, :3]) < 1e-8
    assert np.linalg.norm(solved_z[:3, 3] - expected_z[:3, 3]) < 1e-8


def test_urdf_head_fk_uses_configured_pitch_yaw_order():
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    model = subject.robot_kinematics(config)
    actual = subject.head_matrix(config, model, [0.0, 0.0])
    expected_translation = np.array([0.04656 + 0.051, 0.00009 + 0.03, 0.26115 + 0.008 + 0.097])
    assert np.allclose(actual[:3, 3], expected_translation)
    assert np.allclose(actual[:3, :3], np.eye(3))


def test_yaw_moves_pitch_origin_but_pitch_does_not_move_its_own_origin():
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    model = subject.robot_kinematics(config)
    neutral = subject.head_matrix(config, model, [0.0, 0.0])
    pitch_only = subject.head_matrix(config, model, [0.45, 0.0])
    yaw_only = subject.head_matrix(config, model, [0.0, 0.45])
    assert np.allclose(neutral[:3, 3], pitch_only[:3, 3])
    assert not np.allclose(neutral[:3, 3], yaw_only[:3, 3])
    assert not np.allclose(neutral[:3, :3], pitch_only[:3, :3])


def test_assembly_urdf_and_scene_xml_agree_on_head_camera_chain():
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    model = subject.robot_kinematics(config)
    result = subject.check_model_consistency(
        config, model, [[0.0, 0.0], [0.35, -0.4], [-0.25, 0.5]]
    )
    assert result["passed"]


def test_corner_selection_rejects_collinear_points():
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    try:
        subject.selected_corner_indices(config, [(0, 0), (0, 3), (0, 6)])
    except ValueError as error:
        assert "collinear" in str(error)
    else:
        raise AssertionError("collinear points should be rejected")


def test_command_line_board_arguments_override_json_defaults():
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    parser = subject.build_parser()
    args = parser.parse_args([
        "--config", "unused.json", "capture",
        "--pattern", "9x6", "--square-m", "0.025",
        "--session", "unused", "--count", "1",
    ])
    subject.apply_board_arguments(config, args)
    assert config["board"] == {"columns": 9, "rows": 6, "square_m": 0.025}


def test_extrinsics_reject_checkerboard_arguments_different_from_intrinsics():
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    intrinsics = {"pattern": {"columns": 7, "rows": 9, "square_m": 0.021}}
    try:
        subject.check_intrinsics_pattern(config, intrinsics)
    except ValueError as error:
        assert "do not match" in str(error)
    else:
        raise AssertionError("mismatched checkerboard arguments should be rejected")


def test_reuse_selection_preserves_touch_order(tmp_path):
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    previous = tmp_path / "selection.json"
    subject.write_json(previous, {
        "kind": "sp_vision_touch_selection",
        "corners": [
            {"touch_order": 1, "row": 0, "column": 6},
            {"touch_order": 2, "row": 0, "column": 0},
            {"touch_order": 3, "row": 9, "column": 0},
        ],
    })
    assert subject.reused_corner_indices(config, previous) == [6, 0, 63]


def test_capture_replaces_complete_session_but_preserves_it_on_quit(tmp_path, monkeypatch):
    config = subject.load_config(Path(__file__).parent / "configs" / "head_config.example.json")
    session = tmp_path / "session"
    for name in ("view-001", "view-030"):
        directory = session / name
        directory.mkdir(parents=True)
        (directory / "color.png").write_bytes(b"old")
    (session / "intrinsics.json").write_text("old result")

    class FakeCamera:
        def capture(self, include_joint_state):
            assert include_joint_state
            return np.zeros((8, 8, 3), dtype=np.uint8), None, {"head_pitch_yaw": [0, 0]}

        def close(self):
            pass

    monkeypatch.setattr(subject, "open_camera", lambda _config: (FakeCamera(), {"identity": "mock"}))
    monkeypatch.setattr(subject, "detect_board", lambda image, config, flip: (
        np.zeros((70, 2), dtype=np.float32), {"accepted": True, "board_area_ratio": 0.1}))
    monkeypatch.setattr(subject, "draw_detection", lambda image, *args: image)
    monkeypatch.setattr(cv2, "imshow", lambda *args: None)
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda: None)

    def save_image(path, _image):
        Path(path).write_bytes(b"new")
        return True

    monkeypatch.setattr(cv2, "imwrite", save_image)

    keys = iter(map(ord, "sq"))
    monkeypatch.setattr(cv2, "waitKey", lambda _delay: next(keys))
    subject.capture_command(config, session, 2)
    assert sorted(path.name for path in subject.enumerate_views(session)) == ["view-001", "view-030"]
    assert (session / "view-001" / "color.png").read_bytes() == b"old"

    keys = iter(map(ord, "ss"))
    subject.capture_command(config, session, 2)
    assert sorted(path.name for path in subject.enumerate_views(session)) == ["view-001", "view-002"]
    assert (session / "view-001" / "color.png").read_bytes() == b"new"
    assert (session / "intrinsics.json").read_text() == "old result"

    keys = iter(map(ord, "s"))
    subject.capture_command(config, session, 1, append=True)
    assert sorted(path.name for path in subject.enumerate_views(session)) == [
        "view-001", "view-002", "view-003"]
