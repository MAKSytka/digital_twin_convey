#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

message="$(timeout 10 ros2 topic echo /singulator/matrix/command --once)"
printf '%s\n' "$message"

count="$(printf '%s\n' "$message" | awk '
  /^target_speed_mps:/ {inside=1; next}
  inside && /^---/ {inside=0}
  inside && /^- / {count++}
  END {print count + 0}
')"

if [[ "$count" -ne 56 ]]; then
  echo "FAIL: expected 56 matrix commands, got $count." >&2
  exit 2
fi

if printf '%s\n' "$message" | awk '
  /^target_speed_mps:/ {inside=1; next}
  inside && /^---/ {inside=0}
  inside && /^- / {
    value=$2 + 0.0
    if (value < 0.999 || value > 3.001) bad=1
  }
  END {exit bad ? 1 : 0}
'; then
  echo "PASS: 56 matrix commands are within 1.00..3.00 m/s."
else
  echo "FAIL: matrix command outside 1.00..3.00 m/s." >&2
  exit 3
fi

echo
echo "Outfeed command:"
outfeed="$(timeout 5 ros2 topic echo /singulator/outfeed/cmd_vel --once || true)"
printf '%s\n' "$outfeed"
if ! printf '%s\n' "$outfeed" | awk '
  /^data:/ {value=$2 + 0.0; seen=1}
  END {exit (seen && value >= 2.45 && value <= 2.55) ? 0 : 1}
'; then
  echo "WARN: expected downstream outfeed near 2.50 m/s." >&2
fi

echo
echo "Static V5 checks:"
controller="src/singulator_control/singulator_control/singulation_controller.py"
launch_file="src/singulator_bringup/launch/matrix_stream_roller.launch.py"
generator="src/singulator_gazebo/scripts/generate_matrix_14x4_stream_v2.py"
world="src/singulator_gazebo/worlds/matrix_14x4_stream_v2.sdf"

if grep -Eq 'capture_rows|stabilisation_rows|rank_feedforward_weight|capture_speed_mps' \
  "$controller" "$launch_file"; then
  echo "FAIL: removed V4 zonal/rank parameters are still present." >&2
  exit 4
fi

grep -n 'uniform_rows=true' "$controller"
grep -n 'discharge_speed_mps' "$controller" "$launch_file"
grep -nE '^MAX_ACCELERATION = 6\.0$|^MAX_JERK = 30\.0$|^OUTFEED_LENGTH = 1\.60$' \
  "$generator"
grep -n 'model name="outfeed_conveyor"' "$world"
grep -n '<max_acceleration>6.0</max_acceleration>' "$world" | head -1
grep -n '<max_jerk>30.0</max_jerk>' "$world" | head -1

echo "PASS: V5 uniform controller and downstream outfeed are installed."
