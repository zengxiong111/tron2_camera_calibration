#!/usr/bin/env python3
"""Build and verify the hardware-free wheel in a fresh isolated environment."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(args: list[str], *, cwd: Path, env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, cwd=cwd, env=env, text=True, check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def main() -> int:
    uv = shutil.which("uv")
    if not uv:
        raise SystemExit("uv is required; install uv and add it to PATH")

    clean_env = os.environ.copy()
    clean_env.pop("PYTHONPATH", None)
    clean_env.pop("PYTHONHOME", None)

    with tempfile.TemporaryDirectory(prefix="sp-vision-wheel-check-") as tmp:
        tmpdir = Path(tmp)
        wheelhouse, venv, work = tmpdir / "wheels", tmpdir / ".venv", tmpdir / "outside-checkout"
        wheelhouse.mkdir()
        work.mkdir()
        built = run([uv, "build", "--wheel", "--out-dir", str(wheelhouse)], cwd=ROOT, env=clean_env)
        wheels = sorted(wheelhouse.glob("*.whl"))
        if len(wheels) != 1:
            raise RuntimeError(f"expected one wheel, found {wheels}\n{built.stdout}")
        run([uv, "venv", "--python", sys.executable, str(venv)], cwd=work, env=clean_env)
        python = venv / "bin" / "python"
        run([uv, "pip", "install", "--python", str(python), f"{wheels[0]}[test,live]"], cwd=work, env=clean_env)
        bin_dir = venv / "bin"

        for command in ("sp-vision-head", "sp-vision-wrist", "sp-vision-capture"):
            result = run([str(bin_dir / command), "--help"], cwd=work, env=clean_env)
            if not result.stdout.strip():
                raise RuntimeError(f"{command} --help returned no output")
            print(f"PASS {command} --help")

        probe = r'''import importlib.abc, importlib.machinery, importlib.resources, pathlib, sys
source = pathlib.Path(sys.argv[1]).resolve()
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "tron2_deployment" or fullname.startswith("tron2_deployment."):
            raise RuntimeError(f"deployment checkout import blocked: {fullname}")
        if fullname == "sp_vision" or fullname.startswith("sp_vision."):
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
            if spec and spec.origin and pathlib.Path(spec.origin).resolve().is_relative_to(source):
                raise RuntimeError(f"source-tree import blocked: {fullname} -> {spec.origin}")
        return None
sys.meta_path.insert(0, Guard())
import sp_vision
import sp_vision.calibration as head
import sp_vision.calibration_wrist as wrist
import sp_vision.capture_ros2_image
package = importlib.resources.files("sp_vision")
for name in ("head_config.example.json", "wrist_config.example.json", "assembly.urdf", "scene.xml"):
    path = package.joinpath("configs", name)
    if not path.is_file():
        raise RuntimeError(f"missing installed config/model resource: {path}")
    print(f"PASS installed model/config resource {name}: {path}")
for module in (head, wrist, sp_vision.capture_ros2_image):
    location = pathlib.Path(module.__file__).resolve()
    if location.is_relative_to(source):
        raise RuntimeError(f"module escaped wheel install: {module.__name__} -> {location}")
    print(f"PASS wheel-only import {module.__name__}: {location}")
head_config = head.load_config(package.joinpath("configs", "head_config.example.json"))
head_model = head.UrdfKinematics(head.config_path(head_config, head_config["robot"]["urdf"]))
head_result = head.check_model_consistency(head_config, head_model, [[0.0, 0.0], [0.2, -0.1], [-0.2, 0.1]])
wrist_config = wrist.load_config(package.joinpath("configs", "wrist_config.example.json"))
wrist_model = wrist.common.UrdfKinematics(wrist.common.config_path(wrist_config, wrist_config["robot"]["urdf"]))
wrist_result = wrist.check_model_consistency(wrist_config, wrist_model)
if not head_result["passed"] or not wrist_result["passed"]:
    raise RuntimeError("installed head/wrist model consistency check failed")
print(f"PASS installed head URDF/MJCF kinematics consistency: max_error={head_result['maximum_error']:.3g}")
print(f"PASS installed wrist URDF/MJCF kinematics consistency: translation={wrist_result['translation_m']:.3g}m rotation={wrist_result['rotation_deg']:.3g}deg")
rigid, bridge_capture, bridge_config, ros_capture = head._import_capture_dependencies()
print(f"PASS installed adapter imports: {bridge_capture.__module__}, {rigid.__module__}, {ros_capture.__module__}")
'''
        print(run([str(python), "-c", probe, str(ROOT)], cwd=work, env=clean_env).stdout, end="")
        # Tests are copied outside the checkout and redirected to the installed
        # package so their legacy top-level import names cannot load source files.
        testdir = work / "tests"
        test_package_dir = testdir / "sp_vision"
        configs = test_package_dir / "configs"
        configs.mkdir(parents=True)
        site_probe = "import importlib.resources; print(importlib.resources.files('sp_vision'))"
        resource_root = Path(run([str(python), "-c", site_probe], cwd=work, env=clean_env).stdout.strip())
        for name in ("head_config.example.json", "wrist_config.example.json", "robot_profile.example.json",
                     "assembly.urdf", "scene.xml"):
            shutil.copy2(resource_root / "configs" / name, configs / name)
        test_files = sorted((ROOT / "sp_vision").glob("test_*.py"))
        for source_test in test_files:
            contents = source_test.read_text(encoding="utf-8")
            contents = contents.replace("import calibration as subject", "import sp_vision.calibration as subject")
            contents = contents.replace("import calibration_wrist as wrist", "import sp_vision.calibration_wrist as wrist")
            (test_package_dir / source_test.name).write_text(contents, encoding="utf-8")
        result = run([str(python), "-m", "pytest", "-q", "-p", "no:cacheprovider", str(test_package_dir)],
                     cwd=work, env=clean_env)
        print(result.stdout.strip())
        if "passed" not in result.stdout:
            raise RuntimeError("pytest produced no passing-test summary")

    print("PASS isolated wheel verification; live camera/ROS capture remains untested")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
