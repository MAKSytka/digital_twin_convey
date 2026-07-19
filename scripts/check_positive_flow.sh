#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

if [[ ! -f install/setup.bash ]]; then
  echo "Workspace is not built. Run ./scripts/build.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

message="$(timeout 10 ros2 topic echo /singulator/matrix/command --once)"
printf '%s\n' "$message"

if printf '%s\n' "$message" | awk '
  /^target_speed_mps:/ {inside=1; next}
  inside && /^---/ {inside=0}
  inside && /^- / {
    value=$2 + 0.0
    if (value <= 0.0) bad=1
  }
  END {exit bad ? 1 : 0}
'; then
  echo "PASS: all observed matrix commands are strictly positive."
else
  echo "FAIL: a zero or negative matrix command was observed." >&2
  exit 2
fi

echo
echo "Infeed command:"
timeout 5 ros2 topic echo /singulator/infeed/cmd_vel --once || true

echo
echo "Roller commands:"
timeout 5 ros2 topic echo /singulator/throat/left/cmd_vel --once || true
timeout 5 ros2 topic echo /singulator/throat/right/cmd_vel --once || true

echo
echo "Friction configuration:"
grep -nE '^MU = 0\.8$|^MU2 = 0\.8$' \
  src/singulator_gazebo/scripts/generate_matrix_14x4_stream_v2.py
