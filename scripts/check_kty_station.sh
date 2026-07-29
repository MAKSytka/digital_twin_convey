#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

failures=0

check_interface() {
  local type="$1"
  if ros2 interface show "$type" >/dev/null 2>&1; then
    echo "OK interface: $type"
  else
    echo "FAIL interface: $type" >&2
    failures=$((failures + 1))
  fi
}

check_node() {
  local node="$1"
  if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
    echo "OK node: $node"
  else
    echo "FAIL node: $node" >&2
    failures=$((failures + 1))
  fi
}

check_topic() {
  local topic="$1"
  if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
    echo "OK topic: $topic"
  else
    echo "FAIL topic: $topic" >&2
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

check_interface singulator_interfaces/msg/KtyGroundTruthArray
check_interface singulator_interfaces/msg/KtyStationState

check_node /station_controller
check_node /kty_vibration_driver
check_node /product_spawner
check_node /kty_registry_json_mirror

check_topic /clock
check_topic /kty/station/state
check_topic /kty/product_spawner/enabled
check_topic /kty/ground_truth/registry
check_topic /kty/ground_truth/registry_json
check_topic /kty/platform/cmd_pos_filtered

printf '\nGazebo services:\n'
if command -v gz >/dev/null 2>&1; then
  check_gz_service /world/kty_station/control
  check_gz_service /world/kty_station/create
  check_gz_service /world/kty_station/remove
  check_gz_service /world/kty_station/set_pose
  check_gz_service /gui/camera/view_control
else
  echo "FAIL: gz command is unavailable" >&2
  failures=$((failures + 1))
fi

printf '\nStation state:\n'
timeout 5 ros2 topic echo /kty/station/state --once || {
  echo "FAIL: no station state received in 5 s" >&2
  failures=$((failures + 1))
}

printf '\nProduct feeder command:\n'
timeout 5 ros2 topic echo /kty/product_spawner/enabled --once || {
  echo "FAIL: no feeder-enable command received in 5 s" >&2
  failures=$((failures + 1))
}

printf '\nGround-truth JSON mirror:\n'
timeout 5 ros2 topic echo /kty/ground_truth/registry_json --once || {
  echo "FAIL: no registry JSON received in 5 s" >&2
  failures=$((failures + 1))
}

printf '\nPlatform command rate:\n'
timeout 5 ros2 topic hz /kty/platform/cmd_pos_filtered --window 100 || true

printf '\nGazebo KTY entities:\n'
if command -v gz >/dev/null 2>&1; then
  gz model --list 2>/dev/null | grep -E '(^|/)(kty_[0-9]{6}|kty_product_)' || true
fi

printf '\nManual world-control smoke test commands:\n'
cat <<'EOF'
Pause:
  gz service -s /world/kty_station/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'
Resume:
  gz service -s /world/kty_station/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
Reset:
  gz service -s /world/kty_station/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset { all: true }'
EOF

printf '\nExpected cycle:\n'
echo "WAIT_EMPTY_KTY -> POSITION_KTY -> CLAMP -> LOAD -> VIBRATE"
echo "During LOAD/VIBRATE names matching kty_product_cNNNNNN_pNNNNNN must appear."

if (( failures > 0 )); then
  echo "KTY runtime diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo "KTY runtime diagnostics: OK"
