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

check_node /kty_flow_cycle

printf '\nGazebo services:\n'
check_gz_service /world/kty_flow/control
check_gz_service /world/kty_flow/create
check_gz_service /world/kty_flow/remove
check_gz_service /world/kty_flow/set_pose
check_gz_service /gui/camera/view_control

printf '\nCurrent state sample:\n'
if ! timeout 5 ros2 topic echo /kty/flow/state --once; then
  echo "FAIL: no /kty/flow/state message in 5 s" >&2
  failures=$((failures + 1))
fi

printf '\nWaiting for one complete deterministic cycle:\n'
if ! python3 - <<'PY'
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Observer(Node):
    def __init__(self):
        super().__init__("kty_flow_acceptance_observer")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.latest = None
        self.create_subscription(String, "/kty/flow/state", self.on_state, qos)

    def on_state(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.latest = payload
        print(
            "state={state} cycle={cycle} spawned={spawned} "
            "inside={inside} removed={removed}".format(
                state=payload.get("state"),
                cycle=payload.get("cycle_id"),
                spawned=payload.get("spawned_products"),
                inside=payload.get("inside_products"),
                removed=payload.get("removed_models"),
            ),
            flush=True,
        )


rclpy.init()
node = Observer()
deadline = time.monotonic() + 70.0
success = False
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        payload = node.latest
        if not payload:
            continue
        if payload.get("state") == "ERROR":
            print(f"ERROR state: {payload.get('detail')}", file=sys.stderr)
            break
        expected = int(payload.get("expected_product_count", 0))
        if (
            payload.get("state") == "COMPLETE"
            and int(payload.get("completed_cycles", 0)) >= 1
            and int(payload.get("spawned_products", -1)) == expected
            and int(payload.get("inside_products", -1)) == expected
            and int(payload.get("removed_models", -1)) == expected + 1
        ):
            success = True
            print("OK: complete cycle observed", flush=True)
            break
finally:
    node.destroy_node()
    rclpy.shutdown()

raise SystemExit(0 if success else 1)
PY
then
  echo "FAIL: complete KTY flow cycle was not observed" >&2
  failures=$((failures + 1))
fi

printf '\nPost-cycle dynamic model check:\n'
model_output="$(gz model --list 2>/dev/null || true)"
if printf '%s\n' "$model_output" | grep -Eq 'kty_flow_container|kty_flow_product_[0-9]+'; then
  echo "FAIL: dynamic KTY flow models remain after COMPLETE" >&2
  printf '%s\n' "$model_output" | grep -E 'kty_flow_container|kty_flow_product_[0-9]+' >&2 || true
  failures=$((failures + 1))
else
  echo "OK: KTY and products were despawned"
fi

cat <<'EOF'

Expected visual cycle:
  SPAWN_KTY -> APPROACH -> LOAD -> SETTLE
  -> OUTFEED -> OUTFEED_HOLD -> DESPAWN -> COMPLETE

Restart one cycle:
  ros2 service call /kty/flow/restart std_srvs/srv/Trigger '{}'
EOF

if (( failures > 0 )); then
  echo "KTY flow diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo "KTY flow diagnostics: OK"
