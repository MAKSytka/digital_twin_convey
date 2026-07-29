#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

if ! ros2 pkg executables kty_station_sim | grep -Eq '^kty_station_sim[[:space:]]+smoke_heartbeat$'; then
  echo "ERROR: kty_station_sim/smoke_heartbeat is not installed." >&2
  echo "Run: bash ./scripts/build_kty_smoke.sh" >&2
  exit 2
fi

WORLD="$(ros2 pkg prefix --share kty_station_sim)/worlds/kty_station_smoke.sdf"
if [[ ! -f "$WORLD" ]]; then
  echo "ERROR: smoke world is missing from the install space: $WORLD" >&2
  echo "Run: bash ./scripts/build_kty_smoke.sh" >&2
  exit 2
fi

echo "[kty-smoke] launching isolated world"
echo "[kty-smoke] expected model: kty_smoke_container"
echo "[kty-smoke] heartbeat: /kty/smoke/heartbeat"

exec ros2 launch kty_station_sim kty_smoke.launch.py "$@"
