# Right wrist camera calibration

[简体中文](wrist_calib.zh-CN.md)

After running `scripts/install.sh` as described in the [README](../README.md) — that is the step that produces `.venv/bin/python` — run `source .venv/bin/activate` from the repository root, then `cd sp_vision` and run the script examples below; `python` in these commands is that environment's interpreter. Configuration paths resolve relative to their JSON file; data and results resolve relative to the current working directory. The commands read sensors or solve offline and do not move the robot.

This is a separate, read-only workflow for the right wrist color camera. Run commands from this directory. Images and results go under `data/wrist_camera_session/`, which is ignored by Git. The bundled model assets keep offline calibration independent from the parent repository. The code never moves the robot.

## Published calibration results

The latest local head (2026-09-23) and right wrist (2026-10-05) intrinsic/extrinsic JSON files are published in [calibration_results](../calibration_results/README.md), with source hashes and frame/units guidance. Both fits passed their internal checks, but recorded independent touch validation failed the 10 mm limit (head 14.42 mm; right wrist 12.26 mm). See the result guide before reuse; publication does not establish hardware acceptance.

## Frames and geometry

In `configs/assembly.urdf`, the camera bracket is fixed to `wrist_roll_R_Link`. `wrist_roll_R_Joint` is movable between that link and `wrist_pitch_R_Link`: its local X axis is orthogonal to the pitch joint's local Y axis. The constant to solve is therefore **`T_wrist_roll_camera`**, from `right_wrist_camera_color_optical_frame` into `wrist_roll_R_Link`. The URDF value is a nominal starting reference, not a measured calibration. At any measured arm state:

```text
T_base_camera(q) = T_base_wrist_roll(q) · T_wrist_roll_camera
T_base_board = T_base_camera(q) · T_camera_board
T_wrist_pitch_camera(q_roll)
  = T_wrist_pitch_wrist_roll(q_roll) · T_wrist_roll_camera
```

Seven measured right arm angles enter `T_base_wrist_roll`. Head angles and left arm angles are not part of that FK chain. The capture still saves all 14 arm angles, by name, for audit and for later touch validation.

Calibrate intrinsics for the actual image size delivered by the selected color topic. A resized stream needs intrinsics fitted at that resolution; source-stream `CameraInfo` cannot be reused without accounting for the resize. Keep image size fixed throughout calibration.

## Prepare and verify state mapping

```bash
cp configs/wrist_config.example.json configs/wrist_config.json
```

Set `capture.host`, `ros_setup`, `ros_domain_id`, `color_topic` and `joint_topic` for your ROS 2 installation. The example host and topics are installation-specific defaults, not discovered devices. The script connects by SSH and accepts an image only when its timestamp is within 100 ms of a fresh joint sample. The ROS 2 first four joints use `abad`, `hip`, `yaw`, `knee` names, while the URDF uses `proximal_pitch`, `proximal_roll`, `proximal_yaw`, `elbow`. The configured name list maps them into the **same 14-value order used by the controller `arm_q14` in head touch validation**. The names differ; the vector order does not. Capture saves both the named ROS 2 values and that ordered vector. Physical FK accuracy still needs the held-out board and independent touch checks.

```bash
python calibration_wrist.py --config configs/wrist_config.json probe
```

For the optional controller comparison and the `state` command, copy `configs/robot_profile.example.json` to `configs/robot_profile.json`, fill `robot.host` and `robot.port`, and set `capture.state_profile` in `wrist_config.json` to `robot_profile.json`. Both require a compatible external `tron2_env` runtime.

`probe` saves a synchronized image and named joint JSON under `data/wrist_camera_session/` without requiring a checkerboard. It also reads the same controller `arm_q14` used by head validation before and after the image, then reports `mapping_check.passed` only if the arm stayed still and each mapped ROS 2 value agrees within 0.005 rad. A failed or unavailable controller comparison does not mean image capture failed. The image/joint `state_skew_ms` is a separate check with a 100 ms limit.

Rigidly fix a flat 7×10 inner-corner checkerboard with measured 0.021 m squares. Keep it fixed through the image set and touch check. Move the arm only through the robot's existing reviewed interface, then let it settle. Vary position and orientation so the board covers the image and the wrist rotates about at least two distinct axes. Keep the full board visible; do not change the camera's resize setting during calibration.

## Capture, fit and check

```bash
python calibration_wrist.py --config configs/wrist_config.json \
  capture --session data/wrist_camera_session --count 40

python calibration_wrist.py --config configs/wrist_config.json \
  intrinsics --session data/wrist_camera_session

python calibration_wrist.py --config configs/wrist_config.json \
  extrinsics --session data/wrist_camera_session
```

In the capture window, `s` saves an accepted image, `f` reverses checkerboard corner order by 180°, `r` or Space reacquires, and `q` quits. Without `--append`, a complete capture replaces the session's previous `view-*` images; quitting early preserves them. Use `--append` to add views. The intrinsic fit reuses the head workflow's checkerboard detection, distortion fit, quality limits and holdout split. The extrinsic fit reuses its PnP and hand-eye solver, but uses synchronized `T_base_wrist_roll(q)` for every view. It checks the nominal URDF/MJCF camera chain and evaluates held-out views against the board pose fitted from training views. Require `passed: true`; also inspect `training`, `holdout`, rejected views, and deviation from the nominal mount.

### `wrist_intrinsics.json` fields

The wrist intrinsics are written by the same code as the head intrinsics, so the fields are identical; see the `head_intrinsics.json` table in the [head guide](head_calib.md) for their meanings: `schema_version`, `kind`, `passed`, `image_size`, `pattern`, `camera_matrix` (3×3 `K`), `distortion` (`[k1, k2, p1, p2, k3]`), `rms_px`, `intrinsic_std` (`[fx, fy, cx, cy]` standard deviations), `training_views`, `training_view_rms_px`, `holdout_views`, `rejected_views`, `radial_monotonicity`, `detector`. The wrist result adds two fields so another camera's result cannot be reused here:

| Field | Meaning |
| --- | --- |
| `camera_frame` | Camera frame the result belongs to; must equal `robot.camera_frame` (`right_wrist_camera_color_optical_frame`). |
| `color_topic` | Colour topic used during calibration; must equal `capture.color_topic`. |

The extrinsics solve and `select-validation` cross-check these two fields and fail loudly on a mismatch instead of silently reusing a result.

### `wrist_extrinsics.json` fields

| Field | Type | Meaning |
| --- | --- | --- |
| `schema_version` | int | Structure version of this result file. |
| `kind` | str | Fixed tag `sp_vision_right_wrist_extrinsics`. |
| `passed` | bool | Whether both training and holdout satisfy the translation/rotation residual limits. |
| `mount_link` | str | Extrinsics parent frame; must be `wrist_roll_R_Link`. |
| `camera_frame` | str | The calibrated wrist camera frame. |
| `T_wrist_roll_camera` | 4×4 | **The calibration result**: `wrist_roll_R_Link` ← wrist colour camera. Use this at runtime. |
| `nominal_T_wrist_roll_camera` | 4×4 | The URDF nominal mount, for comparison only. |
| `T_base_board` | 4×4 | The fixed board's pose in the base frame (a jointly solved quantity); comparing it across runs reveals whether the board moved. |
| `calibrated_from_nominal` | object | `translation_m` and `rotation_deg` — note this is **degrees**, unlike the head extrinsics which use radians. |
| `training` | list | Per-view `{view, pnp_rms_px, translation_m, rotation_rad}`. |
| `holdout` | list | Held-out views, same shape as `training`, not used for the solve. |
| `rejected` | list | Views dropped by detection, synchronization or PnP limits, as `{view, reason}`. |
| `outliers` | list | Robustly discarded inconsistent views, as `{view, translation_m, rotation_rad}`. |
| `mount_rotation_span_deg` | float | Wrist orientation span in degrees, showing whether the pose excitation was sufficient. |
| `secondary_rotation_rad` | float | Second singular value of the rotation-axis distribution; too small means every pose rotates about nearly one axis. |
| `solver` | object | Same as the head: `initial_method`, `initial_score`, `optimizer_success`, `optimizer_cost`, `method_failures`. |
| `model_consistency` | object | URDF/MJCF consistency of this wrist camera chain: `passed`, `translation_m`, `rotation_deg`. |

## Getting a better calibration

Board and image-quality requirements match the head workflow (physical printed board, caliper-measured square side, rigidly fixed and never handheld, mostly 20°–45° tilted poses, coverage of the centre and edges, no moiré or blur). Wrist-specific points:

- Prefer 20 or more training poses and rotate the wrist about **at least two non-parallel** axes; the default orientation span requirement is ≥ 20° (`min_mount_rotation_span_deg`).
- Insufficient excitation fails early (`insufficient wrist orientation spread`, `nearly about one axis`). Add poses instead of loosening the limits.
- Keep the resize setting constant: intrinsics are resolution-specific, so a resized image needs its own calibration and cannot reuse the raw stream's `CameraInfo`.
- Confirm the intrinsics `passed: true` with `radial_monotonicity.minimum_radial_derivative > 0` before solving extrinsics.
- Read `calibrated_from_nominal`: a deviation far beyond assembly tolerance points at the solve or the data. Check corner order and synchronization rather than accepting it.
- Moving the board, changing the resize, or refitting the wrist camera invalidates both the intrinsics and the extrinsics.
- `passed: true` only means the offline gates were met; the independent touch validation is still the final check.

## Calibrate the right-arm touch TCP

To recalibrate the TCP, keep one rigid tip against one fixed point. A dexterous fingertip is usable only if every finger joint remains at the same pose; the state files do not record finger joints. After each pose settles, save measured state to `data/wrist_camera_session/tcp/pose-01.json` through `pose-04.json` with the `state` command, which only reads controller feedback and sends no motion:

```bash
python calibration_wrist.py --config configs/wrist_config.json \
  state --output data/wrist_camera_session/tcp/pose-01.json
python calibration_wrist.py --config configs/wrist_config.json \
  state --output data/wrist_camera_session/tcp/pose-02.json
python calibration_wrist.py --config configs/wrist_config.json \
  state --output data/wrist_camera_session/tcp/pose-03.json
python calibration_wrist.py --config configs/wrist_config.json \
  state --output data/wrist_camera_session/tcp/pose-04.json
```

Each state JSON contains `arm_q14`: fourteen finite measured joint angles in radians, in the configured left-arm order followed by the right-arm order, plus a synchronized `head_q2` and `timestamp_s`.

The wrist command reuses the head workflow's fixed-point solver and returns the tip position in `wrist_roll_R_Link`:

```bash
python calibration_wrist.py --config configs/wrist_config.json \
  pivot --states data/wrist_camera_session/tcp/pose-{01,02,03,04}.json \
  --output data/wrist_camera_session/tcp/wrist_tcp_pivot.json
```

Require `passed: true` and inspect `max_residual_m`, wrist orientation spread, and `condition_number`. Four-pose fitting determines tip **position**, not tool orientation. You may reuse a passed head-workflow TCP result only when it belongs to the same right wrist frame, physical probe and unchanged finger posture. Pass that file explicitly with `--tcp`; no prior calibration results are distributed in this repository.

## Independent touch validation

After fitting extrinsics, keep the checkerboard fixed in the base frame and capture a **new** wrist image that was not used for fitting. The right arm may move after this image; predictions use the joint state synchronized to that image. Select three spread-out, non-collinear physical inner corners that the tip can touch:

```bash
python calibration_wrist.py --config configs/wrist_config.json \
  capture --session data/wrist_camera_validation --count 1

python calibration_wrist.py --config configs/wrist_config.json \
  select-validation --frame data/wrist_camera_validation/view-001 \
  --output data/wrist_camera_validation/wrist_selection.json
```

Each completed `capture --count 1` replaces the previous validation image with a new `view-001`; quitting with `q` preserves the old image. After recapturing, rerun `select-validation` so `wrist_selection.json` and `wrist_selection.png` match the new image. Previous touch states are usable only if the board and physical corners stayed fixed.

Inspect the numbers in `data/wrist_camera_validation/wrist_selection.png` against the physical corners. Keep the board fixed and use the same tip and finger pose as in TCP fitting. Touch corners 1, 2, and 3 in order and save measured state to `data/wrist_camera_validation/state-01.json` through `state-03.json` with the `state` command after each pose settles:

```bash
python calibration_wrist.py --config configs/wrist_config.json \
  state --output data/wrist_camera_validation/state-01.json
python calibration_wrist.py --config configs/wrist_config.json \
  state --output data/wrist_camera_validation/state-02.json
python calibration_wrist.py --config configs/wrist_config.json \
  state --output data/wrist_camera_validation/state-03.json
```

Then compare the predictions with those states:

```bash
python calibration_wrist.py --config configs/wrist_config.json \
  validate --selection data/wrist_camera_validation/wrist_selection.json \
  --side right --tcp data/wrist_camera_session/tcp/wrist_tcp_pivot.json \
  --states data/wrist_camera_validation/state-{01,02,03}.json \
  --output data/wrist_camera_validation/wrist_validation.json
```

Validation compares camera-predicted corners with touched tip positions in `base_Link`; this report's `passed` is the independent check. Neither TCP fit residuals nor held-out extrinsic consistency replace it.

The `translation_m` values in the extrinsic JSON's `training` and `holdout` lists are in meters. The terminal summary converts their maxima to `max_training_mm` and `max_holdout_mm`, as the head command does. If touch errors are close to checkerboard corner spacing, check that numbers 1, 2, and 3 in `wrist_selection.png` match the physical touch order. Only after confirming the physical corners, rerun `select-validation --corner row,column` in that order and validate again. Inferring corners solely from the touch states does not make an independent validation pass.
