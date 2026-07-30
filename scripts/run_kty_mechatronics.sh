#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

for executable in \
  mechatronics_cycle_v3 \
  fill_estimator_v2 \
  depth_perception_3d_v2 \
  contour_recorder_3d \
  vision_dashboard_3d; do
  if ! ros2 pkg executables kty_station_sim | awk '{print $2}' | grep -Fxq "$executable"; then
    echo "ERROR: missing kty_station_sim executable: $executable" >&2
    echo "Run: bash ./scripts/build_kty_perception_3d.sh" >&2
    exit 1
  fi
done

plugin_prefix="$(ros2 pkg prefix kty_conveyor_surface 2>/dev/null || true)"
if [[ -z "$plugin_prefix" || ! -f "$plugin_prefix/lib/libKtyConveyorSurfaceSystem.so" ]]; then
  echo "ERROR: missing kty_conveyor_surface Gazebo plugin" >&2
  echo "Run: bash ./scripts/build_kty_perception_3d.sh" >&2
  exit 1
fi

exec ros2 launch kty_station_sim kty_mechatronics_surface.launch.py "$@"
