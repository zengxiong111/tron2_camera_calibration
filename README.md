# TRON2 Camera Calibration

[简体中文](README.zh-CN.md)

Standalone Python tools for offline calibration of the TRON2 moving head color camera and right wrist color camera. The package contains the calibration code and kinematic snapshots needed by the solvers; it does not contain a robot controller, ROS installation, camera recordings, or local robot profiles.

## Install

Python 3.10 or newer is required. Install from a checkout:

```bash
python -m pip install .
```

For development and tests:

```bash
python -m pip install -e '.[test]'
python -m pytest -q
```

The same base dependencies are listed in `requirements.txt` for environments that use requirements files. `pip install -e .` is the recommended project installation command.

The offline solvers need NumPy, SciPy and OpenCV's contrib modules. Head-camera live capture through its bridge backend additionally needs `pip install '.[live]'` (the `websockets` client). The configured head-camera ROS backend uses ROS 1 Noetic locally and needs the system `rospy`, `message_filters` and `sensor_msgs` packages, plus a reachable ROS master. The separate right-wrist capture workflow and `sp-vision-capture` utility use ROS 2 Foxy on the configured camera host and require `rclpy` and `sensor_msgs` there; neither ROS generation is installed by this Python package. The optional controller-state probe uses the separately provided `tron2_env` runtime (and its transport dependencies) and is not needed for offline calibration.

## Commands

The install provides three no-argument command entrypoints; each opens its own help when passed `--help`:

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

## Workflows and evidence limits

- **Offline calibration** reads saved images, joint-state records, JSON settings and the bundled kinematic models. It performs camera intrinsics/extrinsics, fixed-point TCP fitting and geometric checks without opening a camera, contacting ROS, or commanding a robot.
- **Live capture** is separate from solving. The head workflow defaults to its ROS 1 Noetic RGB-D path and can use the existing WebSocket RGB-D bridge backend; the independent right wrist workflow reads the right color image and synchronized joint state through ROS 2 on the camera host. `sp-vision-capture` is a ROS 2 single-image diagnostic. These interfaces must not be confused across ROS generations. None of them sends motion commands.
- **Model snapshots are kinematic-only.** The bundled URDF/MJCF snapshots reference mesh files that are not distributed here. The solvers use joint transforms and do not load visual or collision geometry, so these snapshots support kinematic FK and model-chain consistency checks only. They cannot support mesh rendering, collision checking, physics simulation or geometric robot-clearance claims.
- **Passing a solver is not physical validation.** Offline fit and held-out checks do not prove live camera synchronization, correct physical mounting, successful robot touch validation, or Sim2Real performance. Follow the head and wrist guides for independent touch checks and interpret each report's `passed` field in its stated scope.

See [head calibration](sp_vision/head_calib.md) and [right wrist calibration](sp_vision/wrist_calib.md) for data collection and validation steps.

## License

Contributors: [AcSg999](https://github.com/AcSg999) and [Shukashuki](https://github.com/Shukashuki). See [CONTRIBUTORS.md](CONTRIBUTORS.md) for source attribution.


MIT. The original copyright notice is retained in [LICENSE](LICENSE).
