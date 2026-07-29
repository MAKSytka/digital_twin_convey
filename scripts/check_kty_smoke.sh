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
if gz model --list 2>/dev/null | grep -Fxq 'kty_smoke_container'; then
  echo "OK model: kty_smoke_container"
else
  echo "FAIL model: kty_smoke_container" >&2
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
