#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u

echo "=== Nodes ==="
ros2 node list | sort

echo
echo "=== Core topics ==="
for topic in \
  /clock \
  /singulator/matrix/command \
  /singulator/matrix/state \
  /singulator/infeed/cmd_vel \
  /singulator/outfeed/cmd_vel \
  /singulator/cell/r00_c00/cmd_vel \
  /singulator/cell/r13_c03/cmd_vel; do
  ros2 topic info "$topic" || true
  echo
done
