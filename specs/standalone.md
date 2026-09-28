# Standalone camera calibration extraction

Extract `sp_vision` from `AcSg999/tron2_deployment_on_dc` at commit `dbd6c2f2d0718f38c86fac34dc86788ad43c60b5` into `tron2_camera_calibration`.

- Preserve existing offline calibration algorithms, tests, config schema and URDF/XML kinematic snapshots.
- Provide an installable `sp_vision` Python package, dependencies, CLI commands, bilingual documentation and CI.
- Remove runtime imports from `tron2_deployment`; migrate the minimum camera capture and read-only controller feedback adapters. Optional ROS/vendor transport runtimes remain explicitly documented.
- Keep construction/import free of hardware connections and do not expose new motion commands.
- Configure remote host, ROS setup path and ROS domain; preserve existing capture synchronization and quality gates.
- Include MIT license and source provenance. Exclude local profiles, captured data, generated results, logs and credentials.
- Verify all existing offline tests and focused adapter tests; verify a built wheel outside the source checkout, including config/model assets and CLI help. Do not claim live hardware verification.
- Credit `AcSg999` and `Shukashuki` in contributor documentation; invite both as repository collaborators after publication. Preserve original copyright attribution.
- Publish the validated result as `zengxiong111/tron2_camera_calibration`, Public, when the authorized account is available.

This extraction starts new history with a pinned source import commit; it does not preserve the full parent repository's historical commits. The original repository is unchanged.
