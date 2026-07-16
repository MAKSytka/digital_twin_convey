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

CONVEYOR_SPEED_MPS="${CONVEYOR_SPEED_MPS:-2.0}"

echo "Starting all conveyors at ${CONVEYOR_SPEED_MPS} m/s"

exec ros2 launch singulator_bringup matrix_stream.launch.py \
  start_spawner:=true \
  start_demo_controller:=true \
  start_cleanup:=true \
  infeed_speed_mps:="${CONVEYOR_SPEED_MPS}" \
  outfeed_speed_mps:="${CONVEYOR_SPEED_MPS}" \
  demo_speed_mps:="${CONVEYOR_SPEED_MPS}" \
  target_rate_boxes_per_sec:="${TARGET_RATE_BOXES_PER_SEC:-4.0}" \
  seed:="${SEED:-42}"
