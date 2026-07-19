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

echo "Vision observations:"
timeout 8 ros2 topic hz /singulator/boxes --window 10 || true

echo
echo "Matrix command rate:"
timeout 8 ros2 topic hz /singulator/matrix/command --window 20 || true

echo
echo "One matrix command:"
timeout 8 ros2 topic echo /singulator/matrix/command --once || true

echo
echo "Publishers of /singulator/matrix/command:"
ros2 topic info /singulator/matrix/command --verbose || true
