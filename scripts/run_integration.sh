#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

if [[ ! -f install/setup.bash ]]; then
  echo "Workspace is not built. Run ./scripts/build.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

exec ros2 launch singulator_bringup matrix_stream.launch.py \
  start_spawner:=false \
  start_demo_controller:=false \
  start_cleanup:=true \
  infeed_speed_mps:="${INFEED_SPEED_MPS:-2.0}" \
  outfeed_speed_mps:="${OUTFEED_SPEED_MPS:-2.0}"
