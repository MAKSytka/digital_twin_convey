#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

# Gazebo by itself only renders the passive station.  The cycle starts in the
# ROS controller, so fail loudly instead of opening a world which looks healthy
# (and whose simulation clock advances) while none of the KTY nodes are usable.
required_executables=(
  station_controller
  product_spawner
  depth_perception
  safety_monitor
  metrics_node
)
installed_executables="$(ros2 pkg executables kty_station_sim 2>/dev/null || true)"
for executable in "${required_executables[@]}"; do
  if ! grep -Eq "^kty_station_sim[[:space:]]+${executable}$" <<<"$installed_executables"; then
    echo "ERROR: kty_station_sim/${executable} is not installed." >&2
    echo "Run ./scripts/build.sh, source install/setup.bash, then retry." >&2
    exit 2
  fi
done

ros2 launch kty_station_sim kty_station.launch.py "$@"
