#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
set -u

python3 tools/validate_kty_classical_3d.py
python3 tools/validate_kty_contact_surface.py

rm -rf \
  build/singulator_interfaces install/singulator_interfaces \
  build/kty_conveyor_surface install/kty_conveyor_surface \
  build/kty_station_sim install/kty_station_sim

colcon build \
  --symlink-install \
  --packages-select \
  singulator_interfaces \
  kty_conveyor_surface \
  kty_station_sim

set +u
source install/setup.bash
set -u

for exe in \
  depth_perception_3d_v2 \
  contour_recorder_3d \
  vision_dashboard_3d \
  mechatronics_cycle_v3 \
  fill_estimator_v2; do
  ros2 pkg executables kty_station_sim | awk '{print $2}' | grep -Fxq "$exe" || {
    echo "ERROR missing $exe" >&2
    exit 1
  }
done

plugin_prefix="$(ros2 pkg prefix kty_conveyor_surface)"
test -f "$plugin_prefix/lib/libKtyConveyorSurfaceSystem.so" || {
  echo "ERROR missing KtyConveyorSurfaceSystem plugin" >&2
  exit 1
}

ros2 interface show singulator_interfaces/msg/KtyGraspCandidate >/dev/null
ros2 interface show singulator_interfaces/msg/KtyProductContour >/dev/null

echo "KTY contact-surface runtime build: OK"
