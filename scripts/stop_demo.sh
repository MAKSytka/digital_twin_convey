#!/usr/bin/env bash
set -Eeuo pipefail

patterns=(
  "matrix_stream.launch.py"
  "gz sim"
  "parameter_bridge"
  "image_bridge"
  "matrix_command_fanout"
  "aux_conveyor_controller"
  "singulation_row_spawner"
  "uniform_matrix_controller"
  "singulation_controller"
  "vision_stream_node"
  "cleanup_passed_boxes"
)

for pattern in "${patterns[@]}"; do
  pkill -f "$pattern" 2>/dev/null || true
 done

echo "Singulator simulation processes were stopped."
