#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
set -u

rm -rf build/kty_station_sim install/kty_station_sim

colcon build \
  --symlink-install \
  --packages-select kty_station_sim

set +u
source install/setup.bash
set -u

if ! ros2 pkg executables kty_station_sim | grep -Eq '^kty_station_sim[[:space:]]+smoke_heartbeat$'; then
  echo "ERROR: smoke_heartbeat was not installed." >&2
  exit 2
fi

WORLD="$(ros2 pkg prefix --share kty_station_sim)/worlds/kty_station_smoke.sdf"
if [[ ! -f "$WORLD" ]]; then
  echo "ERROR: smoke world was not installed: $WORLD" >&2
  exit 2
fi

echo "KTY smoke build: OK"
echo "Package: $(ros2 pkg prefix kty_station_sim)"
echo "World:   $WORLD"
