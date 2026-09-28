# Changelog

This file records version-level changes. See [简体中文](CHANGELOG.zh-CN.md).

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
