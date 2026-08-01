#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

printf 'Release checks\n'
printf 'branch=%s\n' "$(git branch --show-current)"
printf 'commit=%s\n' "$(git rev-parse --short HEAD)"
printf 'generated=%s\n\n' "$(date --iso-8601=seconds)"

run() {
  printf '\n>>> %q' "$1"
  shift
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

run release python3 tools/validate_release.py
run project python3 tools/validate_project.py
run separator python3 tools/validate_separator_demo.py
run kty-v18 python3 tools/validate_kty_runtime_v18.py
run v7-logic python3 tools/test_v7_logic.py
run v7-structure bash scripts/check_v7_control.sh

printf '\nAll static release checks passed.\n'
printf 'Runtime Gazebo checks are still required on the target Ubuntu workstation.\n'
