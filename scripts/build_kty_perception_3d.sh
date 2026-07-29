#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
set -u
python3 tools/validate_kty_classical_3d.py
rm -rf build/singulator_interfaces install/singulator_interfaces build/kty_station_sim install/kty_station_sim
colcon build --symlink-install --packages-select singulator_interfaces kty_station_sim
set +u
source install/setup.bash
set -u
for exe in depth_perception_3d contour_recorder_3d vision_dashboard_3d mechatronics_cycle fill_estimator; do
  ros2 pkg executables kty_station_sim | awk '{print $2}' | grep -Fxq "$exe" || { echo "ERROR missing $exe" >&2; exit 1; }
done
ros2 interface show singulator_interfaces/msg/KtyGraspCandidate >/dev/null
ros2 interface show singulator_interfaces/msg/KtyProductContour >/dev/null
echo "KTY classical 3-D perception build: OK"
