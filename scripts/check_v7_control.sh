#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

CONTROLLER="src/singulator_control/singulator_control/singulation_controller.py"
LOGIC="src/singulator_control/singulator_control/global_queue_logic.py"
LAUNCH="src/singulator_bringup/launch/matrix_stream_roller.launch.py"

python3 -m py_compile "$CONTROLLER" "$LOGIC" "$LAUNCH"
python3 tools/test_v7_logic.py

grep -q 'control_v7:' "$CONTROLLER"
grep -q 'global_order' "$CONTROLLER"
grep -q 'build_pairwise_speed_profile' "$CONTROLLER"
grep -q 'merged_track_timeout_s' "$CONTROLLER"
grep -q 'cell_r17_c03' 'src/singulator_gazebo/worlds/matrix_14x4_stream_v2.sdf'
grep -q '/singulator/cell/r17_c03/cmd_vel' \
  'src/singulator_bringup/config/bridge_rows_14_17.yaml'
grep -q '/singulator/outfeed/cmd_vel' \
  'src/singulator_bringup/config/bridge_aux.yaml'
grep -q '/singulator/throat/left/cmd_vel' \
  'src/singulator_bringup/config/bridge_throat.yaml'
grep -q '"4.590"' "$LAUNCH"
grep -q 'SetLaunchConfiguration("matrix_rows", "18")' "$LAUNCH"
grep -q 'ParameterValue(matrix_rows, value_type=int)' \
  'src/singulator_bringup/launch/matrix_stream.launch.py'

if grep -qE 'next_available_exit_s|slot_time_by_id|_assign_exit_slots' "$CONTROLLER"; then
  echo "FAIL: obsolete exit-slot scheduler symbols are still present." >&2
  exit 2
fi

if grep -qE 'cell_owner_by_index|cell_owner_hold_s|cell_owner_switch_margin' "$CONTROLLER"; then
  echo "FAIL: obsolete shared-cell owner selection is still present." >&2
  exit 3
fi

echo "PASS: V7 static structure and pure control tests are valid."

if [[ ! -f install/setup.bash ]]; then
  echo "INFO: workspace is not built; ROS topic range check skipped."
  exit 0
fi

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

if ! timeout 2 ros2 topic list 2>/dev/null | grep -qx '/singulator/matrix/command'; then
  echo "INFO: /singulator/matrix/command is not active; runtime check skipped."
  exit 0
fi

message="$(timeout 10 ros2 topic echo /singulator/matrix/command --once)"
printf '%s\n' "$message"

count="$(printf '%s\n' "$message" | awk '
  /^target_speed_mps:/ {inside=1; next}
  inside && /^---/ {inside=0}
  inside && /^- / {count++}
  END {print count+0}
')"

if [[ "$count" -ne 72 ]]; then
  echo "FAIL: expected 72 speed commands, got $count." >&2
  exit 4
fi

if ! printf '%s\n' "$message" | awk '
  /^target_speed_mps:/ {inside=1; next}
  inside && /^---/ {inside=0}
  inside && /^- / {
    value=$2 + 0.0
    if (value < 0.999 || value > 3.001) bad=1
  }
  END {exit bad ? 1 : 0}
'; then
  echo "FAIL: a matrix speed is outside 1.00..3.00 m/s." >&2
  exit 5
fi

echo "PASS: 72 live matrix commands are within 1.00..3.00 m/s."
