#!/usr/bin/env bash
set -Eeo pipefail

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

echo "=== Publisher count (must be 1 for /singulator/matrix/command) ==="
ros2 topic info /singulator/matrix/command -v

echo
echo "=== Infeed command ==="
timeout 5 ros2 topic echo /singulator/infeed/cmd_vel --once

echo
echo "=== Outfeed command ==="
timeout 5 ros2 topic echo /singulator/outfeed/cmd_vel --once

echo
echo "=== One matrix cell command ==="
timeout 5 ros2 topic echo /singulator/cell/r00_c00/cmd_vel --once

echo
echo "=== Full matrix command (56 values, all must be 2.0) ==="
timeout 5 ros2 topic echo /singulator/matrix/command --once
