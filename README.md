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
2. Generate, download and print your own board using the [pattern generator](https://calib.io/zh/pages/camera-calibration-pattern-generator). Count its inner corners and **measure the printed square size**, then update the config. The example has 7×10 inner corners, or 8×11 printed squares.
3. Fix the board, capture about 40 distinct poses, then solve intrinsics and extrinsics.
4. Calibrate a rigid probe TCP and independently validate three board corners.

Run script examples from `sp_vision/`; config paths resolve relative to the JSON file, data paths relative to the working directory. See `sp-vision-head --help`, `sp-vision-wrist --help`, and `sp-vision-capture --help` for options.

### Measure the printed board

Use calipers or a ruler on the final printed, mounted board. Measure across several consecutive squares horizontally and vertically, then divide each length by the number of squares. Repeat at different locations to check print scaling and equal horizontal/vertical dimensions. The required value is the **side length of one square**, excluding the paper and border.

Divide the measured millimetres by 1000 and set `board.square_m` in local `head_config.json` / `wrist_config.json`. Set `board.columns` and `board.rows` to inner-corner counts. The head example's `0.021` and wrist example's `0.015` are examples, not measurements of your board. Remake and remeasure a warped or unevenly scaled board. Capture, intrinsics, extrinsics and validation must use the same measured geometry; re-solve and revalidate after changing it.

### Capture and solve

```bash
cd sp_vision
cp configs/head_config.example.json configs/head_config.json
cp configs/wrist_config.example.json configs/wrist_config.json
cp configs/robot_profile.example.json configs/robot_profile.json
```

Edit the local configs first: enter measured board geometry, addresses and topics. Point `capture.profile` (head) and `capture.state_profile` to `robot_profile.json`. Select ROS or the bridge for head capture. Set up SSH public-key login for the wrist, optionally selecting `capture.ssh_identity_file`.

```bash
python calibration.py --config configs/head_config.json capture --session data/head --count 40
python calibration.py --config configs/head_config.json intrinsics --session data/head --output data/head/head_intrinsics.json
python calibration.py --config configs/head_config.json extrinsics --session data/head --intrinsics data/head/head_intrinsics.json --output data/head/head_extrinsics.json

python calibration_wrist.py --config configs/wrist_config.json capture --session data/wrist --count 40
python calibration_wrist.py --config configs/wrist_config.json intrinsics --session data/wrist --output data/wrist/wrist_intrinsics.json
python calibration_wrist.py --config configs/wrist_config.json extrinsics --session data/wrist --intrinsics data/wrist/wrist_intrinsics.json --output data/wrist/wrist_extrinsics.json
```

During capture, `s` saves, `f` reverses corner order, `r`/Space reacquires and `q` quits. Keep the board fixed, vary head/wrist poses and wait for motion to settle. A completed capture replaces previous views; use `--append` to add views. Require `passed: true`, then record probe poses with `state`, fit TCP with `pivot`, choose three non-collinear corners with `select-validation`, touch and record them in order, and run `validate`. See each subcommand's `--help` for arguments.

| Function | Additional requirements |
|---|---|
| Offline intrinsics, extrinsics, TCP and validation | Installed Python environment |
| Head capture | Local ROS 1 Noetic or WebSocket bridge; color and synchronized head state only, without depth or existing intrinsics |
| Wrist capture | ROS 2 Foxy on the camera host, image and joint-state topics, and SSH public-key login; set `capture.ssh_identity_file` to select a private key |
| `state` / wrist `probe` controller comparison | Compatible external `tron2_env` and configured `robot.host` / `robot.port` |
| Single-frame diagnostic | ROS 2 host and SSH; the snapshot lacks synchronized joint state and cannot serve as a calibration observation |

## Preparation references

- [Calibration pattern generator](https://calib.io/zh/pages/camera-calibration-pattern-generator): generate, download and print your own board, then measure its square size and update the config.
- [TRON2 SDK development guide](https://www.limxdynamics.com/zh/documents/847884267345285120): development connections and SSH login password. Authorize your public key before using the wrist script's non-interactive login.
- [TRON2 user manual](https://www.limxdynamics.com/zh/documents/844648486841487360): network setup and Wi-Fi password.
- [Official TRON2 robot description](https://github.com/limxdynamics/tron2-robot-description): URDF/xacro and model resources including dual-arm/head variants. Select the actual hardware variant. Bundled solver snapshots do not automatically track upstream changes.

## Results and frames

[Published head and wrist results](calibration_results/README.md) passed internal fit checks, but independent touch errors were **14.42 mm** for the head and **12.26 mm** for the wrist, exceeding the 10 mm limit. Re-solve and validate with the intended model before reuse.

`T_A_B` maps B into A; translations use metres and joint angles use radians. For tabletop work, the physical-base origin is fixed **0.45 m above the tabletop**: `z_physical_base_world - z_tabletop_world = 0.45 m`. Resolve the physical-base frame rather than substituting the URDF root or wrist origin. Bundled models contain kinematics without meshes. An intrinsic/extrinsic `passed: true` does not establish independent touch acceptance.

Keep local configuration, images and generated reports in ignored locations such as `data/`. Do not commit passwords, private keys or capture datasets.

## Development and license

```bash
python -m pip install -e '.[test]'
python -m pytest -q
```

[MIT license](LICENSE), retaining Jerry Lo’s original copyright notice. Thanks to [AcSg999](https://github.com/AcSg999) and [Shukashuki](https://github.com/Shukashuki).

Sources: tools and kinematic snapshots from [tron2_deployment_on_dc](https://github.com/AcSg999/tron2_deployment_on_dc), `sp_vision/` at `dbd6c2f2d0718f38c86fac34dc86788ad43c60b5`; bridge adapter derived from [dexpipe](https://github.com/Shukashuki/dexpipe), `hardware/tron2_rgbd.py` at `1e4a810de477bda7f621db553ecd496bb0884f2f`; read-only controller adapter references [tron2_env](https://github.com/limxdynamics/tron2_env) at `03e39d3f7d66dcb096b41f03612fdab2f8c10ea5`.
