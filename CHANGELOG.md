# Changelog

This file records version-level changes. See [简体中文](CHANGELOG.zh-CN.md).

## [Unreleased]

### Added
- Publish the latest local head (2026-09-23) and right wrist (2026-10-05) intrinsics/extrinsics under `calibration_results/`, with byte-preserving JSON, a source/hash manifest, bilingual usage guidance and explicit failed independent touch-validation status.
- Add `scripts/install.sh`, adapted from the deployment installer: it creates `.venv/bin/python` and the three `sp-vision-*` entrypoints, installs `'.[test,live]'`, runs `pip check` and validates each entrypoint with `--help`.

### Fixed
- Restore the runnable `state` subcommand for `sp-vision-head` and `sp-vision-wrist`. It records measured controller `arm_q14`/`head_q2` feedback to JSON for TCP pivot fitting and touch validation, replacing guide text that deferred to an unavailable external state recorder. It reads feedback only and sends no motion command.
- Document `capture.state_profile` in `head_config.example.json` and in the head/wrist guides.

### Documentation
- Make `scripts/install.sh` the documented installation step: it produces `.venv/bin/python`, which the README and both calibration guides call, and offline calibration needs only that environment. The README also records the installer's limits (Python 3.10+ with `venv`/`ensurepip`, OpenCV shared libraries, reachable index, existing-`.venv` handling, and that it installs neither ROS nor `tron2_env`).
- Document every field of the `head_intrinsics.json`, `wrist_intrinsics.json`, `head_extrinsics.json` and `wrist_extrinsics.json` results in the calibration guides, and add a "getting a better calibration" section covering board choice, pose coverage, excitation limits and the triggers that invalidate a solve.

### Notes & Caveats
- The `state` command needs the external `tron2_env` runtime, like the wrist `probe` comparison; offline solving still does not.

## [0.1.0] - 2026-09-28

### Features
- Package the offline TRON2 head and right wrist camera calibration tools as `tron2-camera-calibration`.
- Include example JSON configuration and URDF/MJCF kinematic snapshots in built distributions.
- Add command-line entrypoints and document optional live capture requirements.

### Design Rationale
- Keep offline solving installable without ROS or controller software. Hardware-facing adapters and live acquisition remain optional and lazy-loaded.
- Bundle only kinematic model files needed for solver FK; external mesh assets are not required by the offline solver.

### Notes & Caveats
- Live ROS acquisition requires separately installed ROS runtime and a reachable configured camera host. The vendor controller-state probe additionally depends on the external `tron2_env` runtime.
- Included URDF/MJCF snapshots do not include their referenced meshes and must not be used for rendering, collision or physics claims.
- Do not treat an offline pass as independent physical touch validation or Sim2Real success.
