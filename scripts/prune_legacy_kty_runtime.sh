#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MODE="check"
case "${1:-}" in
  ""|--check|--dry-run)
    MODE="check"
    ;;
  --apply)
    MODE="apply"
    ;;
  *)
    echo "Usage: $0 [--check|--apply]" >&2
    exit 2
    ;;
esac

# This list contains superseded KTY implementations, wrappers, diagnostics,
# documents and workflows. Missing paths are allowed because some of them may
# already have been removed in a previous cleanup commit.
LEGACY_PATHS=(
  docs/KTY_RUNTIME_V7.md
  docs/KTY_MECHATRONICS_STAGE4.md
  docs/KTY_VISION_STAGE3.md
  docs/KTY_CLASSICAL_3D_STAGE6.md
  docs/KTY_CONTACT_SURFACE_RUNTIME.md

  src/kty_station_sim/kty_station_sim/mechatronics_cycle.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v2.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v3.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v10.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v11.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v12.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v13.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v14.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v15.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v16.py
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v17.py

  src/kty_station_sim/kty_station_sim/world_patch_v2.py
  src/kty_station_sim/kty_station_sim/world_patch_v3.py

  src/kty_station_sim/kty_station_sim/fill_estimator.py
  src/kty_station_sim/kty_station_sim/depth_perception.py
  src/kty_station_sim/kty_station_sim/depth_perception_3d.py
  src/kty_station_sim/kty_station_sim/contour_recorder.py
  src/kty_station_sim/kty_station_sim/vision_dashboard.py

  src/kty_station_sim/launch/kty_mechatronics.launch.py
  src/kty_station_sim/launch/kty_mechatronics_v2.launch.py
  src/kty_station_sim/launch/kty_mechatronics_v7.launch.py
  src/kty_station_sim/launch/kty_mechatronics_surface.launch.py
  src/kty_station_sim/launch/kty_mechatronics_v13.launch.py
  src/kty_station_sim/launch/kty_mechatronics_v14.launch.py
  src/kty_station_sim/launch/kty_mechatronics_v15.launch.py
  src/kty_station_sim/launch/kty_mechatronics_v16.launch.py
  src/kty_station_sim/launch/kty_vision.launch.py

  scripts/build_kty_mechatronics.sh
  scripts/run_kty_mechatronics.sh
  scripts/stop_kty_mechatronics.sh
  scripts/build_kty_vision.sh
  scripts/run_kty_vision.sh
  scripts/stop_kty_vision.sh
  scripts/check_kty_mechatronics.sh
  scripts/check_kty_vision.sh
  scripts/check_kty_vibration.sh
  scripts/check_kty_runtime_v7.sh
  scripts/check_kty_cycle_v12.sh
  scripts/check_kty_cycle_v13.sh
  scripts/check_kty_startup_v14.sh
  scripts/check_kty_startup_v15.sh
  scripts/check_kty_runtime_v16.sh

  tools/validate_kty_flow.py
  tools/validate_kty_mechatronics.py
  tools/validate_kty_vision.py
  tools/validate_kty_runtime_v7.py
  tools/validate_kty_runtime_v13.py
  tools/validate_kty_runtime_v14.py
  tools/validate_kty_runtime_v15.py
  tools/validate_kty_runtime_v16.py
  tools/validate_kty_runtime_v17.py

  .github/workflows/kty-runtime-v7-static.yml
  .github/workflows/kty-runtime-v13-static.yml
  .github/workflows/kty-runtime-v15-static.yml
  .github/workflows/kty-runtime-v16-static.yml
  .github/workflows/kty-runtime-v17-static.yml
)

# These files must exist after the migration. The pruning script does not
# synthesize them: it verifies that consolidation has already happened.
REQUIRED_CURRENT_PATHS=(
  docs/TROUBLESHOOTING.md
  docs/KTY_RUNTIME_COMMANDS.md
  docs/KTY_RUNTIME_V18_HANDOFF.md
  src/kty_station_sim/kty_station_sim/mechatronics_cycle_v18.py
  src/kty_station_sim/kty_station_sim/world_patch_v4.py
  src/kty_station_sim/kty_station_sim/fill_estimator_v2.py
  src/kty_station_sim/kty_station_sim/depth_perception_3d_v2.py
  src/kty_station_sim/kty_station_sim/contour_recorder_3d.py
  src/kty_station_sim/kty_station_sim/vision_dashboard_3d.py
  tools/validate_kty_runtime_v18.py
  scripts/build_kty_perception_3d.sh
  scripts/run_kty_perception_3d.sh
  scripts/check_kty_runtime_v18.sh
)

for path in "${REQUIRED_CURRENT_PATHS[@]}"; do
  if [[ ! -f "$path" ]]; then
    echo "ERROR: required current file is missing: $path" >&2
    exit 1
  fi
done

printf 'Legacy KTY cleanup scenario\n'
printf 'mode=%s\n' "$MODE"
printf 'branch=%s\n\n' "$(git branch --show-current)"

tracked=()
for path in "${LEGACY_PATHS[@]}"; do
  if git ls-files --error-unmatch "$path" >/dev/null 2>&1; then
    tracked+=("$path")
  fi
done

printf 'Tracked legacy candidates: %d\n' "${#tracked[@]}"
printf '%s\n' "${tracked[@]:-}"

# Search references by basename in surviving tracked files. References inside
# another legacy candidate, this script and the cleanup plan are ignored.
declare -A legacy_map=()
for path in "${LEGACY_PATHS[@]}"; do
  legacy_map["$path"]=1
done

references_file="$(mktemp)"
trap 'rm -f "$references_file"' EXIT

for target in "${tracked[@]}"; do
  name="$(basename "$target")"
  while IFS= read -r line; do
    source_path="${line%%:*}"
    [[ "$source_path" == "$target" ]] && continue
    [[ -n "${legacy_map[$source_path]:-}" ]] && continue
    [[ "$source_path" == "scripts/prune_legacy_kty_runtime.sh" ]] && continue
    [[ "$source_path" == "docs/LEGACY_RUNTIME_CLEANUP.md" ]] && continue
    printf '%s -> %s\n' "$source_path" "$name" >> "$references_file"
  done < <(git grep -nF "$name" -- . 2>/dev/null || true)
done

sort -u "$references_file" -o "$references_file"

if [[ -s "$references_file" ]]; then
  echo
  echo "Surviving references that must be migrated before deletion:"
  cat "$references_file"
else
  echo
  echo "No surviving references to legacy basenames were found."
fi

if [[ "$MODE" == "check" ]]; then
  cat <<'EOF'

Dry-run only. No files were deleted.
Before --apply:
  1. consolidate the current KTY launch into one non-versioned release launch;
  2. remove old console_scripts from setup.py;
  3. simplify the KTY build/run/check scripts;
  4. replace historical GitHub Actions with the current release workflow;
  5. rerun this command until the surviving-reference list is empty.
EOF
  exit 0
fi

if [[ -s "$references_file" ]]; then
  echo "ERROR: cleanup aborted because surviving references remain." >&2
  exit 1
fi

branch="$(git branch --show-current)"
if [[ -z "$branch" || "$branch" == "main" || "$branch" == "master" ]]; then
  echo "ERROR: --apply is forbidden on the primary branch." >&2
  exit 1
fi

if [[ -n "$(git status --porcelain)" ]]; then
  echo "ERROR: worktree must be clean before --apply." >&2
  exit 1
fi

backup_dir="${HOME}/git_backups"
mkdir -p "$backup_dir"
bundle="$backup_dir/digital_twin_before_legacy_prune_$(date +%Y%m%d-%H%M%S).bundle"

git bundle create "$bundle" --all
git bundle verify "$bundle"
printf 'Backup bundle: %s\n' "$bundle"

if ((${#tracked[@]} == 0)); then
  echo "Nothing to delete."
  exit 0
fi

git rm -- "${tracked[@]}"

if ! bash scripts/run_release_checks.sh; then
  echo "ERROR: release checks failed; restoring deleted files." >&2
  git restore --staged --worktree -- "${tracked[@]}"
  exit 1
fi

cat <<'EOF'

Legacy files are staged for deletion and release checks passed.
Review the diff, perform a clean ROS build and run all three Gazebo demos before commit:

  git diff --cached --stat
  rm -rf build install log
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  bash scripts/build.sh
  source install/setup.bash
  bash scripts/run_roller_demo.sh
  ros2 launch singulator_bringup infeed_size_separator_demo.launch.py seed:=42
  bash scripts/run_kty_perception_3d.sh

The script intentionally does not commit or push the deletion.
EOF
