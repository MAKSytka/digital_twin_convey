#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

failures=0

wait_node() {
  local node="$1"
  local deadline=$((SECONDS + 30))
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

printf 'Contact-surface runtime nodes:\n'
for node in \
  /kty_mechatronics_cycle_v3 \
  /kty_fill_estimator_v2 \
  /kty_classical_3d_perception_v2 \
  /kty_contour_recorder_3d \
  /kty_vision_dashboard_3d \
  /kty_mechatronics_command_bridge; do
  wait_node "$node" || true
done

printf '\nSurface command topics:\n'
for topic in \
  /kty/mech/infeed_surface/cmd_vel \
  /kty/mech/active_surface/cmd_vel \
  /kty/mech/outfeed_surface/cmd_vel; do
  if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
    echo "OK topic: $topic"
  else
    echo "FAIL topic: $topic" >&2
    failures=$((failures + 1))
  fi
done

printf '\nGazebo lifecycle services:\n'
for service in \
  /world/kty_mechatronics_surface/control \
  /world/kty_mechatronics_surface/create \
  /world/kty_mechatronics_surface/remove; do
  if gz service -l 2>/dev/null | grep -Fxq "$service"; then
    echo "OK Gazebo service: $service"
  else
    echo "FAIL Gazebo service: $service" >&2
    failures=$((failures + 1))
  fi
done

printf '\nWaiting for staged release, slide-gate changeover and surface-driven outfeed:\n'
if ! python3 - <<'PY'
import json
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


class Observer(Node):
    def __init__(self):
        super().__init__('kty_contact_surface_acceptance_observer')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.seen = set()
        self.gate_closed_seen = False
        self.gate_open_after_closed = False
        self.second_load = False
        self.outfeed_nonzero = False
        self.release_zero_seen = False
        self.transport_seen = False
        self.error = ''
        self.latest_active = 0.0
        self.latest_outfeed = 0.0
        self.create_subscription(String, '/kty/flow/state', self.on_state, qos)
        self.create_subscription(
            Float64,
            '/kty/mech/active_surface/cmd_vel',
            self.on_active,
            10,
        )
        self.create_subscription(
            Float64,
            '/kty/mech/outfeed_surface/cmd_vel',
            self.on_outfeed,
            10,
        )

    @staticmethod
    def gate_exists():
        result = subprocess.run(
            ['gz', 'model', '--list'],
            capture_output=True,
            text=True,
            timeout=4,
            check=False,
        )
        return 'kty_mech_chute_gate' in result.stdout

    def on_active(self, message):
        self.latest_active = float(message.data)

    def on_outfeed(self, message):
        self.latest_outfeed = float(message.data)
        if abs(self.latest_outfeed) >= 0.05:
            self.outfeed_nonzero = True

    def on_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(data.get('state', ''))
        detail = str(data.get('detail', ''))
        cycle = int(data.get('cycle_id', 0) or 0)
        self.seen.add(state)
        self.transport_seen = self.transport_seen or data.get('transport') == 'flat_contact_surface'
        if state == 'ERROR':
            self.error = str(data.get('detail', 'unknown error'))
        if (
            state == 'EJECT_ACTIVE'
            and 'retract locator' in detail
            and abs(self.latest_active) < 0.01
            and abs(self.latest_outfeed) < 0.01
        ):
            self.release_zero_seen = True
        exists = self.gate_exists()
        if state in {'CLOSE_GATE', 'COMPACT', 'EJECT_ACTIVE', 'POSITION_NEXT', 'VERIFY_READY'} and exists:
            self.gate_closed_seen = True
        if self.gate_closed_seen and state in {'OPEN_GATE', 'LOAD'} and not exists:
            self.gate_open_after_closed = True
        if state == 'LOAD' and cycle >= 2:
            self.second_load = True
        print(
            f"cycle={cycle} state={state} detail={detail!r} gate_model={exists} "
            f"active={self.latest_active:.3f} outfeed={self.latest_outfeed:.3f} "
            f"transport={data.get('transport')} "
            f"fill={float(data.get('estimated_fill_ratio', 0.0) or 0.0):.3f}",
            flush=True,
        )


rclpy.init()
node = Observer()
deadline = time.monotonic() + 300.0
success = False
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.error:
            print(f"ERROR state: {node.error}")
            break
        required = {
            'LOAD', 'CLOSE_GATE', 'COMPACT', 'EJECT_ACTIVE',
            'POSITION_NEXT', 'VERIFY_READY', 'OPEN_GATE'
        }
        if (
            required <= node.seen
            and node.transport_seen
            and node.release_zero_seen
            and node.outfeed_nonzero
            and node.gate_closed_seen
            and node.gate_open_after_closed
            and node.second_load
        ):
            success = True
            print('OK: staged release cleared the locator before flat surfaces moved the loaded KTY')
            break
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if success else 1)
PY
then
  echo 'FAIL: contact-surface changeover was not observed' >&2
  failures=$((failures + 1))
fi

printf '\nCurrent KTY / gate models:\n'
gz model --list 2>/dev/null | grep -E 'kty_mech_container_|kty_mech_chute_gate' || true

printf '\n3-D perception heartbeat:\n'
if timeout 15 ros2 topic echo /kty/perception/contours --once >/tmp/kty_surface_contours.txt; then
  sed -n '1,45p' /tmp/kty_surface_contours.txt
  echo 'OK: 3-D contour frame received'
else
  echo 'FAIL: no 3-D contour frame; inspect /kty/perception/fault' >&2
  timeout 3 ros2 topic echo /kty/perception/fault --once || true
  failures=$((failures + 1))
fi

printf '\nGazebo real-time factor:\n'
stats="$(timeout 5 gz topic -e -t /world/kty_mechatronics_surface/stats -n 1 2>/dev/null || true)"
printf '%s\n' "$stats" | sed -n '1,25p'
python3 - "$stats" <<'PY'
import re
import sys
match = re.search(r'real_time_factor:\s*([0-9.]+)', sys.argv[1])
if match:
    value = float(match.group(1))
    print(f'Measured RTF: {value:.3f}')
    if value < 0.45:
        print('WARNING: RTF remains below 0.45; keep show_dashboard:=false.')
PY

cat <<'EOF'

Expected runtime:
  - generated world contains no roller links or roller joints;
  - locator and clamps release while active/outfeed commands remain zero;
  - after 2.5 s, the three flat plates carry the KTY through force control;
  - /kty/mech/outfeed_surface/cmd_vel becomes positive only after clearance;
  - kty_mech_chute_gate exists while closed and disappears in OPEN_GATE;
  - products remain physical bodies inside the moving KTY;
  - vibration_deck still moves vertically at 8 / 18 Hz.
EOF

if (( failures > 0 )); then
  echo "KTY contact-surface diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo 'KTY contact-surface diagnostics: OK'
