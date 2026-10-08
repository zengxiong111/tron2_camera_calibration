# Published head and right wrist camera parameters

[简体中文](README.zh-CN.md)

Uploaded on 2026-10-08 from `/home/dc/tron2_deployment_on_dc/sp_vision/data`. These are the latest available results for each camera in that local folder, not a new calibration run. All four JSON files are copied byte for byte. [manifest.json](manifest.json) records source paths, source modification times, SHA-256 hashes and the recorded independent-validation summaries. The source checkout commit identifies the code checkout only; the source result files are Git-ignored.

## Files and provenance

Dates and times below are source file modification times in Asia/Shanghai (UTC+08:00), not embedded solve timestamps. The head results are from the actual `head-camera-session` directory (hyphens), even though current guide examples use `head_camera_session` (underscores). The older wrist `intrinsics.json` and `extrinsics.json` from 2026-09-23 are superseded by the `wrist_*` results from 2026-10-05. Only the right wrist has results in this upload.

| Result | Published file | Local source relative to deployment root | Source last modified |
| --- | --- | --- | --- |
| Head intrinsics | [head_intrinsics.json](head/2026-09-23/head_intrinsics.json) | `sp_vision/data/head-camera-session/intrinsics.json` | 2026-09-23 20:15:24 |
| Head extrinsics | [head_extrinsics.json](head/2026-09-23/head_extrinsics.json) | `sp_vision/data/head-camera-session/extrinsics.json` | 2026-09-23 20:16:38 |
| Right wrist intrinsics | [wrist_intrinsics.json](right_wrist/2026-10-05/wrist_intrinsics.json) | `sp_vision/data/wrist_camera_session/wrist_intrinsics.json` | 2026-10-05 19:42:24 |
| Right wrist extrinsics | [wrist_extrinsics.json](right_wrist/2026-10-05/wrist_extrinsics.json) | `sp_vision/data/wrist_camera_session/wrist_extrinsics.json` | 2026-10-05 20:23:37 |

## Recorded quality and acceptance

| Camera | Image size | Intrinsic RMS (px) | Intrinsic/extrinsic `passed` | Independent maximum touch error | Touch limit | Independent `passed` |
| --- | --- | --- | --- | --- | --- | --- |
| Head | 640×480 | 0.072941 | true / true | 14.421725 mm | 10 mm | false |
| Right wrist | 640×480 resized | 0.211810 | true / true | 12.260710 mm | 10 mm | false |

These are recorded local results checked during publication, not fresh hardware measurements. A fit's `passed: true` does not establish independent physical accuracy. Both recorded touch checks failed the 10 mm limit; these parameters remain calibration evidence and are not accepted for tasks requiring ≤10 mm accuracy. Source report paths and hashes are in the manifest; images, raw joint-state samples, TCP results, robot profiles and credentials are not published.

## Units and frames

`camera_matrix` contains pixel intrinsics; `distortion` is the OpenCV five-coefficient vector `[k1, k2, p1, p2, k3]`. Use the matching calibrated 640×480 color image geometry; do not apply these intrinsics unchanged to a different resolution, crop, or depth stream. The wrist resized color stream is `/camera/right/color/image_resized/compressed`, whereas its source CameraInfo has a different resolution.

`T_A_B` transforms coordinates from B into A. Homogeneous transforms are 4×4 and translation is in meters. Optical axes are +x right, +y down, +z forward. The head transform is `T_pitch_camera`, from the head color optical frame into `head_pitch_Link`. The right wrist transform is `T_wrist_roll_camera`, from `right_wrist_camera_color_optical_frame` into `wrist_roll_R_Link`. The capture reports the ROS frame `right_color_optical_frame`; do not assume an installed TF alias exists without checking it. The head result does not embed a camera frame ID; its interpretation follows the head calibration workflow.

```text
p_A = T_A_B · p_B
T_base_head_camera(q) = T_base_pitch(q) · T_pitch_camera
T_base_right_wrist_camera(q) = T_base_wrist_roll(q) · T_wrist_roll_camera
```

Use image-synchronized joint angles with the matching kinematic model. `T_base_board` is the fixed calibration board pose for its own session, not a general object pose. `nominal_T_*` is a model reference, not the measured camera extrinsic. The source deployment head guide (`sp_vision/head_calib.md` in the source checkout) records a subsequent D455 model update; the September head JSON retains its earlier nominal transform and consistency check. Its `model_consistency` pass therefore does not verify compatibility with the current model. Rerun the solve and independent validation against the intended model before acceptance.

For tabletop work, the [fixed tabletop frame contract](../TABLETOP_FRAME_CONTRACT.md) ([中文](../TABLETOP_FRAME_CONTRACT.zh-CN.md)) requires the physical-base origin to be 0.45 m above the table. This publication neither changes mounting geometry nor verifies that historical captures satisfy that contract.

## Load the published files

Run from the calibration repository root; Python's standard library is sufficient to inspect the files. Loading JSON does not connect to hardware or issue commands.

```python
import json
from pathlib import Path

root = Path("calibration_results")
head_intrinsics = json.loads((root / "head/2026-09-23/head_intrinsics.json").read_text())
head_extrinsics = json.loads((root / "head/2026-09-23/head_extrinsics.json").read_text())
wrist_intrinsics = json.loads((root / "right_wrist/2026-10-05/wrist_intrinsics.json").read_text())
wrist_extrinsics = json.loads((root / "right_wrist/2026-10-05/wrist_extrinsics.json").read_text())
K_head = head_intrinsics["camera_matrix"]
D_head = head_intrinsics["distortion"]
T_pitch_camera = head_extrinsics["T_pitch_camera"]
K_wrist = wrist_intrinsics["camera_matrix"]
D_wrist = wrist_intrinsics["distortion"]
T_wrist_roll_camera = wrist_extrinsics["T_wrist_roll_camera"]
```

Keep the original JSON schema and numeric precision when consuming these results. Full field descriptions and recalibration instructions are in the [head guide](../sp_vision/head_calib.md) and [wrist guide](../sp_vision/wrist_calib.md).
