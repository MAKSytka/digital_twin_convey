#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
set -u

rm -rf build/kty_station_sim install/kty_station_sim

python3 tools/validate_kty_flow.py

colcon build \
  --symlink-install \
  --packages-up-to kty_station_sim

set +u
source install/setup.bash
set -u

if ! ros2 pkg executables kty_station_sim | grep -Eq '(^|[[:space:]])flow_cycle$'; then
  echo "ERROR: flow_cycle executable was not installed." >&2
  exit 1
fi

echo "KTY flow build: OK"
