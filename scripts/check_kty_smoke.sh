#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

failures=0

check_node() {
  local node="$1"
  if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
    echo "OK node: $node"
  else
    echo "FAIL node: $node" >&2
    failures=$((failures + 1))
  fi
}

check_gz_service() {
  local service="$1"
  if gz service -l 2>/dev/null | grep -Fxq "$service"; then
    echo "OK Gazebo service: $service"
  else
    echo "FAIL Gazebo service: $service" >&2
    failures=$((failures + 1))
  fi
}

check_gz_model() {
  local model="$1"
  local raw_list
  local normalized_list

  raw_list="$(gz model --list 2>/dev/null || true)"
  normalized_list="$(
    printf '%s\n' "$raw_list" |
      sed -E \
        -e 's/^[[:space:]]*-[[:space:]]*//' \
        -e 's/^[[:space:]]+//' \
        -e 's/[[:space:]]+$//'
  )"

  if grep -Fxq "$model" <<<"$normalized_list"; then
    echo "OK model: $model (gz model --list)"
    return 0
  fi

  # Some Gazebo Harmonic CLI builds decorate or omit entries in `gz model
  # --list`. The scene broadcaster pose stream is an independent runtime
  # source of truth for entities loaded into this world.
  if timeout 5 gz topic -e \
      -t /world/kty_station_smoke/pose/info \
      -n 1 2>/dev/null |
      grep -Fq "name: \"${model}\""; then
    echo "OK model: $model (world pose stream)"
    return 0
  fi

  echo "FAIL model: $model" >&2
  echo "Raw output of 'gz model --list':" >&2
  if [[ -n "$raw_list" ]]; then
    printf '%s\n' "$raw_list" >&2
  else
    echo "  <empty>" >&2
  fi
  return 1
}

check_node /kty_smoke_heartbeat

printf '\nHeartbeat sample:\n'
if ! timeout 5 ros2 topic echo /kty/smoke/heartbeat --once; then
  echo "FAIL: no smoke heartbeat received in 5 s" >&2
  failures=$((failures + 1))
fi

printf '\nGazebo services:\n'
if command -v gz >/dev/null 2>&1; then
  check_gz_service /world/kty_station_smoke/control
  check_gz_service /world/kty_station_smoke/create
  check_gz_service /world/kty_station_smoke/remove
  check_gz_service /world/kty_station_smoke/set_pose
  check_gz_service /gui/camera/view_control
else
  echo "FAIL: gz command is unavailable" >&2
  failures=$((failures + 1))
fi

printf '\nGazebo model check:\n'
if ! check_gz_model kty_smoke_container; then
  failures=$((failures + 1))
fi

printf '\nGazebo stats sample:\n'
if ! timeout 5 gz topic -e -t /world/kty_station_smoke/stats -n 1; then
  echo "FAIL: no Gazebo world stats received in 5 s" >&2
  failures=$((failures + 1))
fi

cat <<'EOF'

Manual controls:
  Pause:
    gz service -s /world/kty_station_smoke/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'
  Resume:
    gz service -s /world/kty_station_smoke/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
  Reset:
    gz service -s /world/kty_station_smoke/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'

Expected GUI behavior:
  middle mouse drag          orbit camera
  left mouse drag            pan camera
  wheel / right mouse drag   zoom camera
EOF

if (( failures > 0 )); then
  echo "KTY smoke diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo "KTY smoke diagnostics: OK"
