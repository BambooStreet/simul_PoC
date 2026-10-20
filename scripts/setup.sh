#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python -c 'import mujoco, numpy, PIL' || python -m pip install --target .deps -r requirements.txt
python -m pip install --target .deps --no-deps dexmate-urdf==0.8.4 pypdf==6.20.1
PYTHONPATH="$PWD/.deps${PYTHONPATH:+:$PYTHONPATH}" python scripts/prepare_robot.py
