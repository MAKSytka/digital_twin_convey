#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
set -u

python3 tools/validate_kty_mechatronics.py

rm -rf \
  build/kty_station_sim \
  install/kty_station_sim \
  build/singulator_interfaces \
  install/singulator_interfaces

colcon build \
  --symlink-install \
  --packages-select singulator_interfaces kty_station_sim

set +u
source install/setup.bash
set -u

for executable in \
  mechatronics_cycle \
  fill_estimator \
  depth_perception \
  contour_recorder \
  vision_dashboard; do
  if ! ros2 pkg executables kty_station_sim | awk '{print $2}' | grep -Fxq "$executable"; then
    echo "ERROR: missing installed executable: $executable" >&2
    exit 1
  fi
done

echo "KTY mechatronics build: OK"
