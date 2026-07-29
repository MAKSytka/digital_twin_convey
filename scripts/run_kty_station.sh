#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

required_executables=(
  station_controller
  vibration_driver
  product_spawner
  depth_perception
  safety_monitor
  metrics_node
  registry_json_mirror
)
installed_executables="$(ros2 pkg executables kty_station_sim 2>/dev/null || true)"
for executable in "${required_executables[@]}"; do
  if ! grep -Eq "^kty_station_sim[[:space:]]+${executable}$" <<<"$installed_executables"; then
    echo "ERROR: kty_station_sim/${executable} is not installed." >&2
    echo "Run: bash ./scripts/build_kty_station.sh" >&2
    exit 2
  fi
done

required_interfaces=(
  KtyProductContour
  KtyProductContourArray
  KtyGroundTruth
  KtyGroundTruthArray
  KtyStationState
  KtyFault
)
for interface in "${required_interfaces[@]}"; do
  if ! ros2 interface show "singulator_interfaces/msg/${interface}" >/dev/null 2>&1; then
    echo "ERROR: singulator_interfaces/msg/${interface} is not installed." >&2
    echo "Run: bash ./scripts/build_kty_station.sh" >&2
    echo "Then source install/setup.bash in every diagnostic terminal." >&2
    exit 2
  fi
done

echo "[kty-startup] interfaces and executables are available"
echo "[kty-startup] kty package: $(ros2 pkg prefix kty_station_sim)"
echo "[kty-startup] interface package: $(ros2 pkg prefix singulator_interfaces)"

exec ros2 launch kty_station_sim kty_station.launch.py "$@"
