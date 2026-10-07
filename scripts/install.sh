#!/usr/bin/env bash
# Create the project virtual environment and install the calibration tools.
#
# This is the step that produces .venv/bin/python together with the
# .venv/bin/sp-vision-head, .venv/bin/sp-vision-wrist and
# .venv/bin/sp-vision-capture runtime units. Every command in README.md and in
# sp_vision/*_calib*.md then runs with that interpreter.
#
# Requirements: a Python 3.10-or-newer executable (default "python3.10",
# override with TRON2_PYTHON), its venv/ensurepip modules, OpenCV's shared
# libraries (on Ubuntu: apt install -y libgl1 libglib2.0-0) and access to a
# package index. It does not install ROS or the optional tron2_env runtime.
set -euo pipefail
cd "$(dirname "$0")/.."
tron2_python="${TRON2_PYTHON:-python3.10}"
if ! command -v "$tron2_python" >/dev/null 2>&1; then
    echo "Python 3.10 or newer is required. Set TRON2_PYTHON to an installed Python executable; see README.md." >&2
    exit 1
fi
"$tron2_python" -I -c 'import sys; sys.exit(0 if sys.version_info[:2] >= (3, 10) else "Use Python 3.10 or newer for the calibration tools; keep the Ubuntu system Python unchanged.")'
if [[ -e .venv ]]; then
    .venv/bin/python -I -c 'import sys; sys.exit(0 if sys.version_info[:2] >= (3, 10) else "Existing .venv uses a Python older than 3.10; preserve it and create a separate checkout/environment.")'
else
    "$tron2_python" -I -m venv .venv
fi
# Keep this index choice local to the installer and its build subprocesses.
export PIP_INDEX_URL="${TRON2_PIP_INDEX_URL:-https://pypi.org/simple}"
.venv/bin/python -I -m pip install --index-url "$PIP_INDEX_URL" --upgrade pip
.venv/bin/python -I -m pip install --index-url "$PIP_INDEX_URL" '.[test,live]'
.venv/bin/python -I -m pip check
.venv/bin/sp-vision-head --help >/dev/null
.venv/bin/sp-vision-wrist --help >/dev/null
.venv/bin/sp-vision-capture --help >/dev/null
echo "Installed. Run the calibration guides with this environment, for example:"
echo "  .venv/bin/python sp_vision/calibration.py --help"
