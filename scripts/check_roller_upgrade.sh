#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ! -f install/setup.bash ]]; then
  echo "Workspace is not built. Run ./scripts/build.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

echo "Matrix command publishers:"
ros2 topic info /singulator/matrix/command --verbose || true

echo
echo "Matrix command range:"
timeout 8 ros2 topic echo /singulator/matrix/command --once || true

echo
echo "Roller throat commands:"
timeout 5 ros2 topic echo /singulator/throat/left/cmd_vel --once || true
timeout 5 ros2 topic echo /singulator/throat/right/cmd_vel --once || true

echo
echo "Gazebo throat topics:"
gz topic -l | grep '/singulator/throat/' || true

echo
echo "Active controller nodes:"
ros2 node list | grep -E 'singulation_controller|roller_throat_controller|uniform_matrix_controller|matrix_command_fanout' || true
