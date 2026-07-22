#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

python3 src/singulator_description/scripts/generate_roller_throat_model.py

MODEL="src/singulator_description/models/roller_throat/model.sdf"

grep -q 'exit_transfer_collision' "$MODEL"
grep -q 'undertray_collision' "$MODEL"

count="$(grep -c '<collision name="roller_' "$MODEL")"
if [[ "$count" -ne 42 ]]; then
  echo "FAIL: expected 42 roller collisions, found $count" >&2
  exit 2
fi

echo "PASS: dense roller throat regenerated (21 rollers per side)."
