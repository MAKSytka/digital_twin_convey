#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

./scripts/stop_demo.sh 2>/dev/null || true

patterns=(
  "matrix_stream_roller.launch.py"
  "roller_throat_controller"
  "roller_throat_spawner"
  "singulator_throat_bridge"
  "singulation_controller"
  "vision_stream_node"
  "image_bridge"
)

for pattern in "${patterns[@]}"; do
  pkill -f "$pattern" 2>/dev/null || true
done

echo "Roller singulator simulation processes were stopped."
