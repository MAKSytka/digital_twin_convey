#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

failures=0

wait_for_node() {
  local node="$1"
  local deadline=$((SECONDS + 25))
  while (( SECONDS < deadline )); do
    if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
      echo "OK node: $node"
      return 0
    fi
    sleep 1
  done
  echo "FAIL node: $node" >&2
  failures=$((failures + 1))
  return 1
}

wait_for_any_node() {
  local label="$1"
  shift
  local deadline=$((SECONDS + 25))
  while (( SECONDS < deadline )); do
    local nodes
    nodes="$(ros2 node list 2>/dev/null || true)"
    for candidate in "$@"; do
      if printf '%s\n' "$nodes" | grep -Fxq "$candidate"; then
        echo "OK ${label}: $candidate"
        return 0
      fi
    done
    sleep 1
  done
  echo "FAIL ${label}: expected one of $*" >&2
  failures=$((failures + 1))
  return 1
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + 25))
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "OK topic: $topic"
      return 0
    fi
    sleep 1
  done
  echo "FAIL topic: $topic" >&2
  failures=$((failures + 1))
  return 1
}

printf 'ROS nodes:\n'
wait_for_node /kty_mechatronics_cycle || true
wait_for_node /kty_fill_estimator || true
wait_for_any_node perception \
  /kty_depth_perception \
  /kty_classical_3d_perception || true
wait_for_any_node recorder \
  /kty_contour_recorder \
  /kty_contour_recorder_3d || true
wait_for_any_node dashboard \
  /kty_vision_dashboard \
  /kty_vision_dashboard_3d || true
wait_for_node /kty_mechatronics_command_bridge || true

printf '\nROS topics:\n'
for topic in \
  /kty/flow/state \
  /kty/fill/state \
  /kty/mech/heartbeat \
  /kty/mech/infeed_rollers/cmd_vel \
  /kty/mech/active_rollers/cmd_vel \
  /kty/mech/outfeed_rollers/cmd_vel \
  /kty/mech/pusher/cmd_pos \
  /kty/mech/clamps/cmd_pos \
  /kty/mech/gate/cmd_pos \
  /kty/mech/vibration/cmd_pos \
  /kty/mech/locator_stop/cmd_pos \
  /kty/vision/depth_image \
  /kty/vision/dashboard; do
  wait_for_topic "$topic" || true
done

printf '\nGazebo services and joint topic:\n'
for service in \
  /world/kty_mechatronics/control \
  /world/kty_mechatronics/create \
  /world/kty_mechatronics/remove; do
  if gz service -l 2>/dev/null | grep -Fxq "$service"; then
    echo "OK Gazebo service: $service"
  else
    echo "FAIL Gazebo service: $service" >&2
    failures=$((failures + 1))
  fi
done
if gz topic -l 2>/dev/null | grep -Fxq /kty/mech/joint_state; then
  echo "OK Gazebo topic: /kty/mech/joint_state"
else
  echo "FAIL Gazebo topic: /kty/mech/joint_state" >&2
  failures=$((failures + 1))
fi

printf '\nWaiting for one physical changeover:\n'
if ! python3 - <<'PY'
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


REQUIRED = {
    "LOAD",
    "CLOSE_GATE",
    "COMPACT",
    "EJECT_ACTIVE",
    "POSITION_NEXT",
    "VERIFY_READY",
    "OPEN_GATE",
}


class Observer(Node):
    def __init__(self):
        super().__init__("kty_mechatronics_acceptance_observer")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.latest = {}
        self.seen = set()
        self.max_fill = 0.0
        self.max_height = 0.0
        self.two_kty_seen = False
        self.second_load_seen = False
        self.create_subscription(String, "/kty/flow/state", self.on_state, qos)

    def on_state(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.latest = payload
        state = str(payload.get("state", ""))
        self.seen.add(state)
        self.max_fill = max(
            self.max_fill,
            float(payload.get("estimated_fill_ratio", 0.0) or 0.0),
        )
        self.max_height = max(
            self.max_height,
            float(payload.get("maximum_height_m", 0.0) or 0.0),
        )
        active = str(payload.get("active_kty", ""))
        queue = str(payload.get("queue_kty", ""))
        self.two_kty_seen = self.two_kty_seen or bool(
            active and queue and active != queue
        )
        if state == "LOAD" and int(payload.get("cycle_id", 0) or 0) >= 2:
            self.second_load_seen = True
        print(
            "cycle={cycle} state={state} fill={fill:.3f} height={height:.3f} "
            "gate={gate} clamps={clamps} vibration={vibration}".format(
                cycle=payload.get("cycle_id"),
                state=state,
                fill=float(payload.get("estimated_fill_ratio", 0.0) or 0.0),
                height=float(payload.get("maximum_height_m", 0.0) or 0.0),
                gate=payload.get("gate_open"),
                clamps=payload.get("clamps_closed"),
                vibration=payload.get("vibration_mode"),
            ),
            flush=True,
        )


rclpy.init()
node = Observer()
deadline = time.monotonic() + 180.0
success = False
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.latest.get("state") == "ERROR":
            print(f"ERROR state: {node.latest.get('detail')}", file=sys.stderr)
            break
        threshold_seen = node.max_fill >= 0.70 or node.max_height >= 0.280
        if (
            REQUIRED <= node.seen
            and node.two_kty_seen
            and node.second_load_seen
            and threshold_seen
        ):
            success = True
            print("OK: complete physical KTY changeover observed", flush=True)
            break
finally:
    node.destroy_node()
    rclpy.shutdown()

raise SystemExit(0 if success else 1)
PY
then
  echo "FAIL: physical KTY changeover was not observed" >&2
  failures=$((failures + 1))
fi

printf '\nCurrent dynamic KTY models:\n'
model_output="$(gz model --list 2>/dev/null || true)"
printf '%s\n' "$model_output" | grep -E 'kty_mech_container_' || true
kty_count="$(printf '%s\n' "$model_output" | grep -Ec 'kty_mech_container_' || true)"
if (( kty_count >= 2 )); then
  echo "OK: at least two KTY models coexist"
else
  echo "WARNING: fewer than two KTY models visible at this exact sample"
fi

printf '\nFill estimate sample:\n'
if timeout 8 ros2 topic echo /kty/fill/state --once; then
  true
else
  echo "FAIL: no fill estimate" >&2
  failures=$((failures + 1))
fi

printf '\nGazebo real-time factor sample:\n'
stats="$(timeout 4 gz topic -e -t /world/kty_mechatronics/stats -n 1 2>/dev/null || true)"
printf '%s\n' "$stats" | sed -n '1,24p'
python3 - "$stats" <<'PY' || true
import re
import sys

text = sys.argv[1]
match = re.search(r"real_time_factor:\s*([0-9.]+)", text)
if match:
    value = float(match.group(1))
    print(f"Measured RTF: {value:.3f}")
    if value < 0.70:
        print(
            "WARNING: RTF is below 0.70; first test with "
            "show_dashboard:=false before reducing sensor quality."
        )
PY

cat <<'EOF'

Expected repeating state sequence:
  LOAD -> CLOSE_GATE -> COMPACT -> EJECT_ACTIVE
  -> POSITION_NEXT -> VERIFY_READY -> OPEN_GATE -> LOAD

Physical settings:
  weak vibration:     8 Hz, ±0.5 mm
  compact vibration: 18 Hz, ±3 mm, 8 s
  gate threshold:     fill >= 70% OR maximum height >= 280 mm
EOF

if (( failures > 0 )); then
  echo "KTY mechatronics diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo "KTY mechatronics diagnostics: OK"
