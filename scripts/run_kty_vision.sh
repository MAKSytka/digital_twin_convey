#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

for executable in flow_cycle depth_perception contour_recorder vision_dashboard; do
  if ! ros2 pkg executables kty_station_sim | awk '{print $2}' | grep -Fxq "$executable"; then
    echo "ERROR: missing kty_station_sim executable: $executable" >&2
    echo "Run: bash ./scripts/build_kty_vision.sh" >&2
    exit 1
  fi
done

exec ros2 launch kty_station_sim kty_vision.launch.py "$@"
