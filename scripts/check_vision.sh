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

echo "Camera rate:"
timeout 8 ros2 topic hz /singulator/camera/image_raw --window 10 || true

echo
echo "One observation array:"
timeout 8 ros2 topic echo /singulator/boxes --once

echo
echo "Published vision topics:"
ros2 topic list | grep -E '^/singulator/(camera|boxes|perception)' || true
