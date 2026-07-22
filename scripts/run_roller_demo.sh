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

exec ros2 launch singulator_bringup matrix_stream_roller.launch.py \
  start_perception:=true \
  start_spawner:=true \
  start_demo_controller:=false \
  start_singulation_controller:=false \
  start_cleanup:=true \
  infeed_speed_mps:="${INFEED_SPEED_MPS:-2.00}" \
  outfeed_speed_mps:="${OUTFEED_SPEED_MPS:-2.50}" \
  target_rate_boxes_per_sec:="${TARGET_RATE_BOXES_PER_SEC:-4.0}" \
  seed:="${SEED:-42}"
