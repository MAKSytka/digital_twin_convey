#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

if ! ros2 pkg executables kty_station_sim | grep -Eq '(^|[[:space:]])flow_cycle$'; then
  echo "ERROR: kty_station_sim/flow_cycle is not installed." >&2
  echo "Run: bash ./scripts/build_kty_flow.sh" >&2
  exit 1
fi

exec ros2 launch kty_station_sim kty_flow.launch.py "$@"
