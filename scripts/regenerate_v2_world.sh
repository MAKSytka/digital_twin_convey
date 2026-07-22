#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

python3 src/singulator_gazebo/scripts/generate_matrix_14x4_stream_v2.py

grep -nE \
  '^MAX_ACCELERATION = 6\.0$|^MAX_JERK = 30\.0$' \
  src/singulator_gazebo/scripts/generate_matrix_14x4_stream_v2.py

grep -nE \
  '<min_acceleration>-6\.0</min_acceleration>|<max_acceleration>6\.0</max_acceleration>|<min_jerk>-30\.0</min_jerk>|<max_jerk>30\.0</max_jerk>' \
  src/singulator_gazebo/worlds/matrix_14x4_stream_v2.sdf | head -n 8

echo "Regenerated matrix_14x4_stream_v2.sdf with ±6 m/s² and ±30 m/s³ limits."
