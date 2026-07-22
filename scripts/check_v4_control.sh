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
    count++
    if (value < 0.999 || value > 3.001) bad=1
  }
  END {
    if (count != 56) exit 2
    exit bad ? 1 : 0
  }
'; then
  echo "PASS: 56 matrix commands are within 1.00..3.00 m/s."
else
  status=$?
  if [[ $status -eq 2 ]]; then
    echo "FAIL: MatrixCommand does not contain exactly 56 speeds." >&2
  else
    echo "FAIL: a command outside 1.00..3.00 m/s was observed." >&2
  fi
  exit "$status"
fi

echo
echo "Roller command (expected 2.50 m/s):"
timeout 5 ros2 topic echo /singulator/throat/left/cmd_vel --once || true

echo
echo "V4 controller parameters:"
ros2 param get /singulation_controller minimum_speed_mps
ros2 param get /singulation_controller maximum_acceleration_mps2
ros2 param get /singulation_controller roller_speed_mps
ros2 param get /singulation_controller capture_rows
ros2 param get /singulation_controller stabilisation_rows
