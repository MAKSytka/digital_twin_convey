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

echo "Controller parameters:"
for parameter in \
  minimum_speed_mps \
  maximum_speed_mps \
  entry_boost_rows \
  entry_boost_speed_mps \
  cluster_dx_enter_m \
  finalizer_rows \
  lag_guard_horizon_s; do
  ros2 param get /singulation_controller "$parameter" || true
done

echo
echo "One matrix command:"
message="$(timeout 10 ros2 topic echo /singulator/matrix/command --once)"
printf '%s\n' "$message"

if printf '%s\n' "$message" | awk '
  /^target_speed_mps:/ {inside=1; next}
  inside && /^---/ {inside=0}
  inside && /^- / {
    value=$2 + 0.0
    if (value < 0.099 || value > 3.001) bad=1
  }
  END {exit bad ? 1 : 0}
'; then
  echo "PASS: observed commands are within 0.10..3.00 m/s."
else
  echo "FAIL: command outside 0.10..3.00 m/s." >&2
  exit 2
fi

echo
echo "Vision frequencies:"
ros2 topic hz /singulator/boxes --window 20 || true
