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
values="$(printf '%s\n' "$message" | awk '
  /^target_speed_mps:/ {inside=1; next}
  inside && /^---/ {inside=0}
  inside && /^- / {print $2}
')"

count="$(printf '%s\n' "$values" | sed '/^$/d' | wc -l)"
if [[ "$count" -ne 56 ]]; then
  echo "FAIL: expected 56 matrix speeds, found $count" >&2
  exit 2
fi

if ! printf '%s\n' "$values" | awk '
  NF {
    value=$1+0.0
    if (value < 1.0-1e-6 || value > 3.0+1e-6) bad=1
  }
  END {exit bad ? 1 : 0}
'; then
  echo "FAIL: matrix command outside 1.00..3.00 m/s." >&2
  exit 3
fi

controller="src/singulator_control/singulator_control/singulation_controller.py"
model="src/singulator_description/models/roller_throat/model.sdf"
world="src/singulator_gazebo/worlds/matrix_14x4_stream_v2.sdf"

grep -q 'Resolution is a one-way state transition' "$controller"
grep -q 'control_v6:' "$controller"
grep -q 'exit_transfer_collision' "$model"
grep -q 'undertray_collision' "$model"
grep -q 'outfeed_conveyor' "$world"

roller_count="$(grep -c '<collision name="roller_' "$model")"
if [[ "$roller_count" -ne 42 ]]; then
  echo "FAIL: expected 42 roller collisions, found $roller_count" >&2
  exit 4
fi

echo "PASS: 56 commands are within 1.00..3.00 m/s."
echo "PASS: rank latch, conflict hysteresis and dense roller throat are installed."
