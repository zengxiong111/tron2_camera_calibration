# TRON2 Camera Calibration

Fixed tabletop height: [frame contract](TABLETOP_FRAME_CONTRACT.md) · [中文](TABLETOP_FRAME_CONTRACT.zh-CN.md)


[简体中文](README.zh-CN.md)

Standalone Python tools for offline calibration of the TRON2 moving head color camera and right wrist color camera. The package contains the calibration code and kinematic snapshots needed by the solvers; it does not contain a robot controller, ROS installation, camera recordings, or local robot profiles.

## Install

Python 3.10 or newer is required. Clone the repository and run the installer once from its root; **this is the step that produces `.venv/bin/python`** together with the `sp-vision-head`, `sp-vision-wrist` and `sp-vision-capture` runtime units, which every command in this README and in the calibration guides uses:

```bash
git clone https://github.com/zengxiong111/tron2_camera_calibration.git
cd tron2_camera_calibration
bash scripts/install.sh
```

`scripts/install.sh` runs, in order: `python3.10 -I -m venv .venv` to create the virtual environment, pip upgrade through `.venv/bin/python`, installation of this package with the `test` and `live` extras (`'.[test,live]'`), `pip check`, then a `--help` check of all three `sp-vision-*` entrypoints. To build the environment by hand instead:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install '.[test,live]'
```

Limits of the installer:

- A Python 3.10-or-newer executable must be on `PATH`. Ubuntu 20.04's system Python 3.8 is not usable, and `/usr/bin/python3` must not be repointed; select another interpreter with `TRON2_PYTHON=/path/to/python3.10 bash scripts/install.sh`.
- That interpreter needs the `venv`/`ensurepip` modules (on Debian/Ubuntu usually the `python3.10-venv` package).
- OpenCV needs system shared libraries: on Ubuntu run `sudo apt install -y libgl1 libglib2.0-0` first, or `import cv2` fails on a missing `libGL.so.1`.
- A package index must be reachable; the default is `https://pypi.org/simple`, and `TRON2_PIP_INDEX_URL` selects another one. That choice only affects the installer and its build subprocesses and does not rewrite the global pip configuration.
- When `.venv` already exists the installer **only validates its interpreter, it never rebuilds it**; an environment older than 3.10 makes it fail outright (keep it and use a separate checkout). Delete `.venv` manually to force a rebuild.
- The installer **does not install ROS or the optional `tron2_env` runtime** (see below).

For development and tests, install the editable copy into the same environment:

```bash
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -m pytest -q
```

A bare `python` in the commands below means that virtual environment's interpreter: run `source .venv/bin/activate` once and keep using `python`, or call `.venv/bin/python` explicitly from the repository root (for example `.venv/bin/python sp_vision/calibration.py ...`). `.venv` is Git-ignored and is not part of the wheel, so create it once per checkout. Keep it separate from any environment that provides `opencv-python` or a headless OpenCV wheel, because this project uses `opencv-contrib-python` as its only `cv2` provider.

The same base dependencies are listed in `requirements.txt` for environments that use requirements files.

Offline calibration needs nothing beyond this virtual environment: NumPy, SciPy and OpenCV's contrib modules are enough to solve the head and wrist intrinsics/extrinsics, the TCP pivot and the touch validation. Head-camera live capture through its bridge backend additionally needs the `websockets` client (already installed by `'.[live]'`). The configured head-camera ROS backend uses ROS 1 Noetic locally and needs the system `rospy`, `message_filters` and `sensor_msgs` packages, plus a reachable ROS master. The separate right-wrist capture workflow and `sp-vision-capture` utility use ROS 2 Foxy on the configured camera host and require `rclpy` and `sensor_msgs` there; neither ROS generation is installed by this Python package or by the installer. The optional controller-state commands (`state` to record a pose, `probe` for the wrist mapping comparison) use the separately provided `tron2_env` runtime (and its transport dependencies) and are not needed for offline calibration.

## Commands

The install provides three command-line entrypoints. Pass `--help` to inspect their subcommands and options:

```bash
sp-vision-head --help
sp-vision-wrist --help
sp-vision-capture --help
```

The original scripts remain runnable from a source checkout:

```bash
python sp_vision/calibration.py --help
python sp_vision/calibration_wrist.py --help
python sp_vision/capture_ros2_image.py --help
```

Packaged example configurations and model snapshots are under `sp_vision/configs/`. Copy the relevant `*.example.json` to a local JSON file before adding machine-specific camera host, topic or profile settings. Local profiles and calibration sessions are intentionally excluded from Git and wheel distributions; store recordings, fitted results and reports under a writable working directory such as `data/`.

## Documentation

| Document | Purpose |
|---|---|
| [Head calibration](sp_vision/head_calib.md) | Prepare configuration, capture views, solve intrinsics/extrinsics and validate touch points |
| [Right wrist calibration](sp_vision/wrist_calib.md) | Verify state mapping, capture wrist views and validate the camera-to-wrist transform |
| [Contributors](CONTRIBUTORS.md) | Project contributors and copyright attribution |
| [Changelog](CHANGELOG.md) | Version history and compatibility notes |
| [Source manifest](SOURCE_MANIFEST.json) | Source commits, original and edited file hashes, model asset scope |

Calibration guides run script examples from `sp_vision/`. Touch validation reads state JSON produced by the read-only `state` command, which samples controller feedback without sending motion commands. No calibration datasets or previous fit results are distributed here.

## Workflows and evidence limits

- **Offline calibration** reads saved images, joint-state records, JSON settings and the bundled kinematic models. It performs camera intrinsics/extrinsics, fixed-point TCP fitting and geometric checks without opening a camera, contacting ROS, or commanding a robot.
- **Live capture** is separate from solving. The head workflow defaults to its ROS 1 Noetic RGB-D path and can use the existing WebSocket RGB-D bridge backend; the independent right wrist workflow reads the right color image and synchronized joint state through ROS 2 on the camera host. `sp-vision-capture` is a ROS 2 single-image diagnostic. These interfaces must not be confused across ROS generations. None of them sends motion commands.
- **Model snapshots are kinematic-only.** The bundled URDF/MJCF snapshots reference mesh files that are not distributed here. The solvers use joint transforms and do not load visual or collision geometry, so these snapshots support kinematic FK and model-chain consistency checks only. They cannot support mesh rendering, collision checking, physics simulation or geometric robot-clearance claims.
- **Passing a solver is not physical validation.** Offline fit and held-out checks do not prove live camera synchronization, correct physical mounting, successful robot touch validation, or Sim2Real performance. Follow the head and wrist guides for independent touch checks and interpret each report's `passed` field in its stated scope.

See [head calibration](sp_vision/head_calib.md) and [right wrist calibration](sp_vision/wrist_calib.md) for data collection and validation steps.

## License

Contributors: [AcSg999](https://github.com/AcSg999) and [Shukashuki](https://github.com/Shukashuki). See [CONTRIBUTORS.md](CONTRIBUTORS.md) for source attribution.

MIT. The original copyright notice is retained in [LICENSE](LICENSE).
