#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
# Runtime v14 adds the canonical SceneBroadcaster dynamic-pose bridge.
exec ros2 launch kty_station_sim kty_mechatronics_v14.launch.py "$@"
