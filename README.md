# TRON2 camera calibration

[简体中文](README.zh-CN.md)

Capture and calibrate TRON2 head and right-wrist color cameras, solve hand-eye extrinsics, and validate with independent touch points. Offline solving needs no ROS. Capture and state commands read sensors without sending motion commands.

## Install

Use Python 3.10+ with `venv`. On Ubuntu, install `libgl1 libglib2.0-0` if OpenCV shared libraries are missing.

```bash
git clone https://github.com/zengxiong111/tron2_camera_calibration.git
cd tron2_camera_calibration
TRON2_PYTHON=python3 bash scripts/install.sh
source .venv/bin/activate
```

The installer creates `.venv`, installs dependencies, and checks all three CLI entrypoints. It checks an existing environment without rebuilding it. Override `TRON2_PYTHON` for another interpreter or `TRON2_PIP_INDEX_URL` for another package index. ROS and `tron2_env` require separate installation.

## Workflow

1. Copy the required `*.example.json` files in `sp_vision/configs/` to local JSON files and fill in connection settings.
2. Print and measure the board: defaults are **7×10 inner corners, 0.021 m squares**, or 8×11 printed squares.
3. Fix the board, capture about 40 distinct poses, then solve intrinsics and extrinsics.
4. Calibrate a rigid probe TCP and independently validate three board corners.

Follow the [head guide](sp_vision/head_calib.md) or [wrist guide](sp_vision/wrist_calib.md). Script examples run from `sp_vision/`; config paths resolve relative to the JSON file, data paths relative to the working directory. See `sp-vision-head --help`, `sp-vision-wrist --help`, and `sp-vision-capture --help` for options.

| Function | Additional requirements |
|---|---|
| Offline intrinsics, extrinsics, TCP and validation | Installed Python environment |
| Head capture | Local ROS 1 Noetic or WebSocket bridge; color and synchronized head state only, without depth or existing intrinsics |
| Wrist capture | ROS 2 Foxy on the camera host, image and joint-state topics, and SSH public-key login; set `capture.ssh_identity_file` to select a private key |
| `state` / wrist `probe` controller comparison | Compatible external `tron2_env` and configured `robot.host` / `robot.port` |
| Single-frame diagnostic | ROS 2 host and SSH; the snapshot lacks synchronized joint state and cannot serve as a calibration observation |

## Preparation references

- [Calibration pattern generator](https://calib.io/zh/pages/camera-calibration-pattern-generator): create a printable board, then measure its square size and update the config.
- [TRON2 SDK development guide](https://www.limxdynamics.com/zh/documents/847884267345285120): development connections and SSH login password. Authorize your public key before using the wrist script's non-interactive login.
- [TRON2 user manual](https://www.limxdynamics.com/zh/documents/844648486841487360): network setup and Wi-Fi password.
- [Official TRON2 robot description](https://github.com/limxdynamics/tron2-robot-description): URDF/xacro and model resources including dual-arm/head variants. Select the actual hardware variant. Bundled solver snapshots do not automatically track upstream changes.

## Results and frames

[Published head and wrist results](calibration_results/README.md) passed internal fit checks, but independent touch errors were **14.42 mm** for the head and **12.26 mm** for the wrist, exceeding the 10 mm limit. Re-solve and validate with the intended model before reuse.

`T_A_B` maps B into A; translations use metres and joint angles use radians. Tabletop work follows the [fixed 45 cm mounting constraint](TABLETOP_FRAME_CONTRACT.md). Bundled models contain kinematics without meshes. An intrinsic/extrinsic `passed: true` does not establish independent touch acceptance.

Keep local configuration, images and generated reports in ignored locations such as `data/`. Do not commit passwords, private keys or capture datasets.

## Development and license

```bash
python -m pip install -e '.[test]'
python -m pytest -q
```

[Changelog](CHANGELOG.md) · [Contributors and sources](CONTRIBUTORS.md) · [MIT license](LICENSE)
