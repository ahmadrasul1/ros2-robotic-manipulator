#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export PYTHONDONTWRITEBYTECODE=1

WP3="src/rascl_wp3_ss26_group11"
SIM_TRAJECTORY="$WP3/trajectories/task1/task1_full_simulation_ik.csv"
HW_TRAJECTORY="$WP3/trajectories/task1/task1_full_hardware.csv"
LIMITS="$WP3/config/robot_limits.yaml"
HOMING="$WP3/config/homing.yaml"

echo "[1/8] Parsing Python source"
python3 - <<'PY'
import ast
from pathlib import Path

files = sorted(Path("src").rglob("*.py"))
for path in files:
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
print(f"Parsed {len(files)} Python files")
PY

echo "[2/8] Parsing YAML and XML/xacro"
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

for path in sorted(Path("src").rglob("*.yaml")):
    with path.open("r", encoding="utf-8") as handle:
        yaml.safe_load(handle)
for pattern in ("*.xml", "*.xacro"):
    for path in sorted(Path("src").rglob(pattern)):
        ET.parse(path)
print("YAML and XML syntax passed")
PY

echo "[3/8] Running unit tests"
PYTHONPATH="$WP3" python3 -m unittest discover -s "$WP3/test" -v

echo "[4/8] Validating simulation and hardware Task 1 trajectories"
python3 "$WP3/scripts/validate_trajectories.py" \
  --limits "$LIMITS" \
  --homing "$HOMING" \
  --verbose \
  "$SIM_TRAJECTORY" "$HW_TRAJECTORY"

echo "[5/8] Running cross-file safety audit"
python3 "$WP3/scripts/audit_project.py" --root "$ROOT"

echo "[6/8] Auditing physical kinematics and Cartesian paths"
python3 "$WP3/scripts/audit_task1_kinematics.py" --root "$ROOT"

echo "[7/8] Checking shell syntax"
bash -n rosws.sh

if command -v xacro >/dev/null 2>&1; then
  echo "[8/8] Expanding xacro"
  xacro src/rascl_description/urdf/rascl.urdf.xacro >/tmp/rascl_verified.urdf
else
  echo "[8/8] xacro not installed here; expansion will be checked by the ROS build container"
fi

if [[ "${RUN_COLCON:-false}" == "true" ]]; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "RUN_COLCON=true, but colcon is unavailable" >&2
    exit 1
  fi
  echo "[optional] Clean ROS 2 build"
  rm -rf build install log
  colcon build --symlink-install
fi

echo "All available project checks passed."
