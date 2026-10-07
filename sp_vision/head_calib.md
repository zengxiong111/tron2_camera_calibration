# Head camera calibration

[简体中文](head_calib.zh-CN.md)

After running `scripts/install.sh` as described in the [README](../README.md) — that is the step that produces `.venv/bin/python` — run `source .venv/bin/activate` from the repository root, then `cd sp_vision` and run the script examples below; `python` in these commands is that environment's interpreter. Configuration paths resolve relative to their JSON file; data and results resolve relative to the current working directory. The commands read sensors or solve offline and do not move the robot.

## ROS 2 one-frame camera diagnostic

The `sp-vision-capture` diagnostic reads one color image from a ROS 2 Foxy host over SSH. It is separate from the main head calibration, whose default RGB-D backend subscribes locally through ROS 1 Noetic. The right wrist calibration also uses its own ROS 2 image and joint-state acquisition path. Do not use this single-image diagnostic as a head calibration frame because it does not save depth or synchronized head state. To check a ROS 2 stream:

```bash
python capture_ros2_image.py
```

The script subscribes to `sensor_msgs/msg/CompressedImage` with the sensor-data QoS profile over SSH and saves `data/right-camera-latest.jpg`. Use `--camera top` for the corresponding head color topic, `/camera/top/color/image_raw/compressed`. Both paths only read images; the snapshot does not include depth or joint state and is not a calibrated observation.

The head workflow implements:

1. capture about 40 fixed-checkerboard images while moving head yaw and pitch;
2. solve color-camera intrinsics;
3. solve `T_pitch_camera`, from the color optical frame to `head_pitch_Link`;
4. validate three checkerboard corners by touching them with a calibrated arm tip.

The program does not command the head, arms, hands, or grippers. Perform motion manually through the robot's existing reviewed interface. Inputs and outputs use JSON, not YAML.

## Model snapshots and frame convention

The bundled model snapshots define the kinematic chains:

- `configs/assembly.urdf` supplies kinematics;
- `configs/scene.xml` cross-checks the same head_camera chain;
- `head_camera_color_optical_frame` is the calibrated OpenCV color frame (+x right, +y down, +z forward);
- `head_pitch_Link` is the camera's nearest moving pitch frame.

The old `d435_Link` attached directly to `base_Link` is ignored. The nominal transform in the final URDF is reported only for comparison and does not constrain the calibration.

`T_A_B` maps coordinates from B into A. The head chain is:

```text
T_base_pitch(q_yaw, q_pitch)
  = T_base_headbase
  · T_headbase_yaw(q_yaw)
  · T_yaw_pitch_origin
  · R_pitch(q_pitch)
```

The yaw-to-pitch offset is `[0.051, 0.03, 0.097] m`. Yaw therefore moves the pitch origin; pitch rotates about its own origin and does not change yaw or move that origin. Captures store `[pitch, yaw]`, but the implementation maps values by joint name and follows the URDF yaw→pitch hierarchy.

Every extrinsic solve checks the relevant transforms from `configs/assembly.urdf` against `configs/scene.xml` at three head poses and stops if they disagree.

## Checkerboard and dependencies

The example config specifies 7×10 **inner corners** and `0.021 m` squares, meaning 8×11 printed squares. Use a flat physical board with a white outer border.

Board geometry can be supplied directly as `--pattern COLSxROWS --square-m METRES`. Command-line values override JSON, and the program rejects disagreement between intrinsic, extrinsic, and validation stages.

```bash
cp configs/head_config.example.json configs/head_config.json
cp configs/robot_profile.example.json configs/robot_profile.json
```

If the profile selects the WebSocket bridge backend instead of ROS Noetic, install its optional Python dependency from this directory with `python -m pip install -e '..[live]'`.

Keep the local config beside the example so `assembly.urdf` and `scene.xml` resolve correctly. For live capture, set `capture.profile` in `head_config.json` to `robot_profile.json` and fill its camera intrinsics, distortion, depth intrinsics, depth-to-color transform and selected ROS/bridge connection settings. Example placeholders and null fields are not a runnable camera profile. Offline solving of previously recorded views does not need a live camera profile. The XML is used for kinematic cross-checks; referenced meshes are not included. Before recording controller state, also fill `robot.host` and `robot.port` (default `5000`) in that profile and point `capture.state_profile` at it; the `state` command needs a compatible external `tron2_env` runtime and is not required for offline solving.

## 1. Capture 40 views

Rigidly fix the board where the camera and arm can both reach it. Do not move it during this dataset.

```bash
python calibration.py \
  --config configs/head_config.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session data/head_camera_session --count 40
```

By default, `--count` is the number of views in a fresh capture. After all views are saved, the new `view-*` directories replace the previous ones in this session and numbering restarts at `view-001`. Quitting early leaves the previous set intact. To deliberately continue an existing dataset, pass `--append --count N`; for example, add ten views to a 30-view session with `--append --count 10`. Re-run intrinsics and extrinsics after either kind of capture because existing JSON results still describe the previous images.

Keys are:

- `s`: save an accepted frame;
- `r`, Space, or another ordinary key: discard and reacquire without saving;
- `f`: reverse corner order by 180° so `(0,0)` remains the same physical corner;
- `q` or Esc: quit.

Detection uses `findChessboardCornersSB` with `NORMALIZE_IMAGE`, `EXHAUSTIVE`, and `ACCURACY`. It must find all 70 subpixel corners and pass topology, coverage, border, and synchronization checks. Colored row overlays are saved as `corners.png` for review.

Move through combinations spanning positive and negative yaw and pitch, and wait for the head to stop before each save. Cover distinct board positions, tilts, and apparent sizes; extra nearly identical frames do little to improve the fit. With the example settings and 40 accepted views, the last six frames are held out and the preceding 34 are fitted, subject to quality rejection. Collect distinct poses for the holdout frames as well.

## 2. Solve intrinsics

```bash
python calibration.py \
  --config configs/head_config.json intrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session data/head_camera_session \
  --output data/head_camera_session/head_intrinsics.json
```

Require `passed: true`. Review per-view RMS, the saved diagnostic overlays, and the radial-monotonicity result. Do not continue if focal length or principal point is implausibly different from factory values. Use a physical board; a checkerboard shown on a laptop introduces moiré and is unsuitable for final calibration.

### `head_intrinsics.json` fields (the wrist intrinsics share this schema)

| Field | Type | Meaning |
| --- | --- | --- |
| `schema_version` | int | Structure version of this result file, currently `1`; unrelated to the config file's top-level `schema_version`. |
| `kind` | str | Result tag; intrinsics always use `sp_vision_intrinsics`. |
| `passed` | bool | Whether every quality gate was met. Do not solve extrinsics from a result with `false`. |
| `image_size` | `[width, height]` | Pixel size of the fitted images. The intrinsics are valid only for this resolution. |
| `pattern` | object | Board geometry: `columns`/`rows` are the **inner-corner** counts, `square_m` is the measured square side in metres. |
| `camera_matrix` | 3×3 | Intrinsic matrix `K`, row order `[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`, in pixels. |
| `distortion` | length 5 | OpenCV five-parameter distortion `[k1, k2, p1, p2, k3]`: `k1`/`k2`/`k3` radial, `p1`/`p2` tangential. |
| `rms_px` | float | Overall reprojection RMS of all training views, in pixels. |
| `intrinsic_std` | length 4 | `[fx, fy, cx, cy]` standard deviations from `calibrateCameraExtended`; a large entry means that degree of freedom is poorly excited. |
| `training_views` | list[str] | The `view-*` directories that actually contributed after quality rejection. |
| `training_view_rms_px` | map | Per-view reprojection RMS in pixels, for spotting a single outlier view. |
| `holdout_views` | list[str] | Views held out of the fit. |
| `rejected_views` | list | Rejected views as `{view, reason}` plus either `rms_px` (per-view reprojection too high) or `metrics` (detection quality failed; keys below). |
| `radial_monotonicity` | object | `max_normalized_radius` (largest normalized radius in frame) and `minimum_radial_derivative`, which **must be greater than 0**. |
| `detector` | str | Detector and flags actually used, for example `cv2.findChessboardCornersSB(NORMALIZE_IMAGE\|EXHAUSTIVE\|ACCURACY)`. |

Keys inside `rejected_views[].metrics`: `accepted`, `detected`, `topology_ok`, `board_area_ratio`, `border_margin_px`, `laplacian_variance`, `corner_count`, `flip_180`, and `reason` when detection failed outright.

## 3. Solve camera-to-pitch extrinsics

```bash
python calibration.py \
  --config configs/head_config.json extrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session data/head_camera_session \
  --intrinsics data/head_camera_session/head_intrinsics.json \
  --output data/head_camera_session/head_extrinsics.json
```

For every frame, IPPE PnP provides `T_camera_board`, and final-URDF FK provides `T_base_pitch`. The fixed board imposes:

```text
T_base_pitch(i) · T_pitch_camera · T_camera_board(i)
  = T_base_board
```

Require `passed: true`. Use `T_pitch_camera` as the measured result. `nominal_T_pitch_camera` is only the final-URDF reference; `training`, `holdout`, and `model_consistency` provide the essential checks.

### `head_extrinsics.json` fields

| Field | Type | Meaning |
| --- | --- | --- |
| `schema_version` | int | Structure version of this result file. |
| `kind` | str | Fixed tag `sp_vision_pitch_camera_extrinsics`. |
| `passed` | bool | Whether both training and holdout satisfy the translation/rotation residual limits. |
| `frame_convention` | str | States that `T_A_B` maps coordinates from frame B into frame A. |
| `T_pitch_camera` | 4×4 | **The calibration result**: `head_pitch_Link` ← colour optical camera. Use this at runtime. |
| `nominal_T_pitch_camera` | 4×4 | The same transform from the URDF, for comparison only; never a solver constraint. |
| `calibrated_from_nominal` | object | Difference from nominal: `translation_m`, `rotation_rad`, and `delta_transform` (`nominal⁻¹ · T_pitch_camera`). |
| `T_base_board` | 4×4 | The fixed board's pose in `base_Link` (a jointly solved quantity). Comparing it across runs reveals whether the board moved. |
| `head_joint_names` | list | Head joint names and their order. |
| `pitch_link` | str | The pitch link used as the extrinsics parent frame. |
| `training` | list | Per-view `{view, pnp_rms_px, translation_m, rotation_rad}`; the last two are `T_base_board` consistency residuals. |
| `holdout` | list | Held-out views, same shape as `training`, not used for the solve. |
| `rejected` | list | Views dropped in training or holdout by detection, synchronization or PnP limits, as `{view, reason}`. |
| `outliers` | list | Robustly discarded inconsistent training views, as `{view, translation_m, rotation_rad}`. |
| `head_span_rad` | length 2 | Peak-to-peak range of each head joint over the training set (ordered by `head_joint_names`), showing whether excitation was sufficient. |
| `solver` | object | `initial_method` (the chosen of five OpenCV hand-eye initializations), `initial_score`, `optimizer_success`, `optimizer_cost`, `method_failures`. |
| `model_consistency` | object | `assembly.urdf` versus `scene.xml` chain check: `passed`, `tolerance`, `maximum_error`, and `samples[]` with `head_q2`, `pitch_translation_m`, `pitch_rotation_rad`, `camera_translation_m`, `camera_rotation_rad`. A disagreement aborts the run instead of writing a result. |
| `quality_limits` | object | The `max_board_residual_m` and `max_board_residual_deg` limits used for this result. |

At runtime:

```text
T_base_camera(q_yaw, q_pitch)
  = T_base_pitch(q_yaw, q_pitch) · T_pitch_camera
```

Task yaw and pitch therefore need not match a calibration pose; use the live joint angles.

## 4. Calibrate the touch tip

Use a sharp point rigidly fixed relative to the wrist. With a dexterous hand, prefer a rigid probe. A fingertip is valid only if every finger joint remains fixed throughout pivot fitting and validation, because state JSON does not contain finger joints.

Touch one fixed point at four clearly different right-wrist orientations. After each pose settles, save measured state to `data/tcp/pose-01.json` through `pose-04.json` with the `state` command, which only reads controller feedback and sends no motion:

```bash
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-01.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-02.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-03.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-04.json
```

Each state JSON contains `arm_q14` (fourteen finite measured joint angles in radians, ordered by the configured left-arm joint list followed by the right-arm list), a synchronized `head_q2`, and `timestamp_s`. Keep the same right-arm probe for the pivot and validation examples below.

```bash
python calibration.py \
  --config configs/head_config.json pivot --side right \
  --states data/tcp/pose-{01,02,03,04}.json \
  --output data/head_camera_session/tcp/head_tcp_pivot.json
```

Require `passed: true`. This fits touch-point position, not tool orientation.

## 5. Touch three validation corners

Without moving the board, capture one new image after calibration:

```bash
python calibration.py \
  --config configs/head_config.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session data/touch-validation --count 1
```

Select three spread-out, non-collinear corners. Omit `--corner` for click-and-snap selection.

```bash
python calibration.py \
  --config configs/head_config.json select-validation \
  --pattern 7x10 --square-m 0.021 \
  --frame data/touch-validation/view-001 \
  --intrinsics data/head_camera_session/head_intrinsics.json \
  --extrinsics data/head_camera_session/head_extrinsics.json \
  --corner 0,0 --corner 0,6 --corner 9,3 \
  --output data/head_camera_validation/head_selection.json
```

Inspect `head_selection.png`. Keep the board fixed, touch labeled points 1, 2, and 3 in order, and save measured joint state in the same order to `data/touch-validation/state-01.json` through `state-03.json` with the `state` command. The head may move after imaging: prediction uses the image-synchronized `head_q2`, while later head motion is recorded only as a diagnostic and does not affect the error gate.

```bash
python calibration.py \
  --config configs/head_config.json state \
  --output data/touch-validation/state-01.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/touch-validation/state-02.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/touch-validation/state-03.json
```

```bash
python calibration.py \
  --config configs/head_config.json validate \
  --selection data/head_camera_validation/head_selection.json \
  --side right --tcp data/head_camera_session/tcp/head_tcp_pivot.json \
  --states data/touch-validation/state-{01,02,03}.json \
  --output data/head_camera_validation/head_validation.json
```

The comparison is:

```text
p_base_camera
  = T_base_pitch(head_q2) · T_pitch_camera · T_camera_board · p_board_corner

p_base_touch
  = T_base_wrist(arm_q14) · p_wrist_tip
```

The metric is 3D Cartesian distance in `base_Link`, not joint-angle difference. The default maximum is 10 mm; replace it with a limit justified by probe repeatability, board mounting, and task clearance. The program sends no motion commands—use supervision and low speed, and do not push the board.

## Recheck previous touch points after moving the head

After accepting the right-arm TCP pivot, the first checkerboard touch selection and three touch states, keep the checkerboard fixed in the same base-frame pose and keep the same rigid tip installation. The head camera may move. Capture one new synchronized color image, reuse the same three physical corners in the original touch order, and compare the newly predicted base-frame points with the saved arm states. This is an offline check; it does not move the robot or require new touches.

```bash
python calibration.py \
  --config configs/head_config.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session data/touch-recheck --count 1

python calibration.py \
  --config configs/head_config.json select-validation \
  --pattern 7x10 --square-m 0.021 \
  --frame data/touch-recheck/view-001 \
  --intrinsics data/head_camera_session/head_intrinsics.json \
  --extrinsics data/head_camera_session/head_extrinsics.json \
  --reuse-selection data/head_camera_validation/head_selection.json \
  --output data/head_camera_validation/head_recheck_selection.json

python calibration.py \
  --config configs/head_config.json validate \
  --selection data/head_camera_validation/head_recheck_selection.json \
  --side right --tcp data/head_camera_session/tcp/head_tcp_pivot.json \
  --states data/touch-validation/state-{01,02,03}.json \
  --output data/head_camera_validation/head_recheck_validation.json
```

Inspect `head_recheck_selection.png` and confirm that the labels match the same physical corners. The new image supplies one board PnP pose; the old states supply the three base-frame touch points. If the board moved after the first touches, or the tip installation changed, the comparison is invalid. The report gives per-point 3D distances and exits with status 1 when the configured limit is exceeded.

## Offline check

```bash
python -m pytest -q test_calibration.py
```

This checks 7×10 SB detection, hand-eye transform direction, final-model FK, yaw/pitch origin behavior, and URDF/XML agreement without connecting to hardware.
## Getting a better calibration

The board itself:

- Use a flat **physical printed** board with no warped corners; a checkerboard shown on a display is unsuitable for a final calibration because moiré and the pixel grid corrupt the corners.
- Measure the square side with calipers before filling in `square_m`; averaging over several squares is more stable than measuring one.
- Confirm the inner-corner counts match the JSON (7×10 here) and leave at least about one square of white margin around the printed pattern.
- Rigidly fix the board (clamp or stand). It must not move during the whole intrinsics-plus-extrinsics dataset, and it must not be held by hand.

Intrinsics:

- Prefer 20 or more training views (default gates are 12 training plus 6 holdout); quality rejection reduces the usable count, so capture extra.
- Cover the frame: put the board near the centre, the corners and the edges, vary `board_area_ratio` between roughly 0.3 and 0.05, and vary the working distance.
- Favour **tilted** poses: views around 20°–45° off the optical axis carry more information than head-on ones.
- Read `intrinsic_std` first: one large entry means that direction is under-excited; add poses rather than loosening limits.
- Always confirm `radial_monotonicity.minimum_radial_derivative > 0`; otherwise the distortion model is non-monotonic in frame and the intrinsics are unusable.

Extrinsics (head):

- With the board fixed, give both head yaw and pitch a clear range (default per-joint peak-to-peak ≥ 0.20 rad ≈ 11.5°), covering positive and negative directions.
- Wait for the head to stop before saving; each frame stores the image-synchronized `head_q2`, and fallback or out-of-tolerance frames are rejected.
- An empty `outliers` list is ideal. When outliers appear, check corner order (whether `f` should flip 180°) and image/joint synchronization before accepting the result.
- Note `T_base_board`: if it changes noticeably in a later run, the board moved.

General:

- Keep image size, focus and resize settings constant across the dataset; mixing resolutions aborts the solve immediately.
- Do not make holdout views nearly identical to training views, or the holdout check proves nothing.
- Moving the board, the camera or its mount, or any collision invalidates the previous intrinsics, extrinsics and TCP; recapture and re-solve.
- `passed: true` only means the offline gates were met, not that the physical setup is accepted; independent touch validation is still required.
## Common failures

- `expected 70 inner corners`: check the inner-corner count and avoid blur, glare, occlusion, and display moiré.
- `(0,0)` jumps to the opposite corner: press `f` during capture before saving.
- `joint/image header skew exceeded`: wait for the head to settle and reacquire.
- `insufficient head excitation`: add combined poses spanning both yaw and pitch directions.
- URDF/XML consistency failure: fix the final model rather than compensating with an old transform.
- Large touch error with low reprojection error: inspect TCP calibration, finger posture, board motion, touch order, and image/state synchronization.
