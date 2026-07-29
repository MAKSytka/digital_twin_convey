#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
set -u

# These two packages must be rebuilt together. Otherwise the Python nodes can
# be new while the shell still sees an old rosidl type-support installation.
rm -rf \
  build/singulator_interfaces \
  install/singulator_interfaces \
  build/kty_station_sim \
  install/kty_station_sim

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -y

colcon build \
  --symlink-install \
  --packages-select singulator_interfaces kty_station_sim \
  --event-handlers console_direct+

set +u
source install/setup.bash
set -u

interfaces=(
  KtyProductContour
  KtyProductContourArray
  KtyGroundTruth
  KtyGroundTruthArray
  KtyStationState
  KtyFault
)

for interface in "${interfaces[@]}"; do
  if ! ros2 interface show "singulator_interfaces/msg/${interface}" >/dev/null; then
    echo "ERROR: generated interface ${interface} is unavailable after build." >&2
    exit 3
  fi
done

python3 tools/validate_kty_station.py

echo
echo "KTY station build complete."
echo "In every new terminal run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${PROJECT_ROOT}/install/setup.bash"
