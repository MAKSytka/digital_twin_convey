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

printf '\nWaiting for eject -> confirmed despawn -> next-KTY positioning:\n'
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
        self.locator_removed_seen = False
        self.locator_recreated_seen = False
        self.transport_seen = False
        self.despawn_seen = False
        self.despawn_confirmed_before_position = False
        self.first_kty = ''
        self.error = ''
        self.latest_active = 0.0
        self.latest_outfeed = 0.0
        self.last_nonzero_outfeed = 0.0
        self.create_subscription(String, '/kty/flow/state', self.on_state, qos)
        self.create_subscription(Float64, '/kty/mech/active_surface/cmd_vel', self.on_active, 10)
        self.create_subscription(Float64, '/kty/mech/outfeed_surface/cmd_vel', self.on_outfeed, 10)

    @staticmethod
    def model_exists(name):
        if not name:
            return False
        result = subprocess.run(
            ['gz', 'model', '--list'],
            capture_output=True,
            text=True,
            timeout=4,
            check=False,
        )
        return name in result.stdout

    def on_active(self, message):
        self.latest_active = float(message.data)

    def on_outfeed(self, message):
        self.latest_outfeed = float(message.data)
        if abs(self.latest_outfeed) >= 0.05:
            self.outfeed_nonzero = True
            self.last_nonzero_outfeed = self.latest_outfeed

    def on_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(data.get('state', ''))
        detail = str(data.get('detail', ''))
        cycle = int(data.get('cycle_id', 0) or 0)
        active_kty = str(data.get('active_kty', ''))
        self.seen.add(state)
        if not self.first_kty and active_kty and cycle == 1:
            self.first_kty = active_kty
        self.transport_seen = self.transport_seen or str(data.get('transport', '')).startswith(
            'flat_contact_surface'
        )
        self.last_nonzero_outfeed = max(
            self.last_nonzero_outfeed,
            float(data.get('last_nonzero_outfeed_mps', 0.0) or 0.0),
        )
        if state == 'ERROR':
            self.error = str(data.get('detail', 'unknown error'))
        locator_exists = self.model_exists('kty_mech_runtime_locator')
        if (
            state == 'EJECT_ACTIVE'
            and 'remove locator' in detail
            and abs(self.latest_active) < 0.01
            and abs(self.latest_outfeed) < 0.01
        ):
            self.release_zero_seen = True
        if state == 'EJECT_ACTIVE' and 'locator absent' in detail and not locator_exists:
            self.locator_removed_seen = True
        if state == 'DESPAWN_ACTIVE':
            self.despawn_seen = True
        despawned_cycles = int(data.get('despawned_cycles', 0) or 0)
        last_despawned = str(data.get('last_despawned_kty', ''))
        if state in {'POSITION_NEXT', 'VERIFY_READY', 'OPEN_GATE', 'LOAD'}:
            if (
                despawned_cycles >= 1
                and self.first_kty
                and last_despawned == self.first_kty
                and not self.model_exists(self.first_kty)
            ):
                self.despawn_confirmed_before_position = True
        if state in {'POSITION_NEXT', 'VERIFY_READY'} and locator_exists:
            self.locator_recreated_seen = True
        gate_exists = self.model_exists('kty_mech_chute_gate')
        if state in {
            'CLOSE_GATE', 'COMPACT', 'EJECT_ACTIVE', 'DESPAWN_ACTIVE',
            'POSITION_NEXT', 'VERIFY_READY'
        } and gate_exists:
            self.gate_closed_seen = True
        if self.gate_closed_seen and state in {'OPEN_GATE', 'LOAD'} and not gate_exists:
            self.gate_open_after_closed = True
        if state == 'LOAD' and cycle >= 2:
            self.second_load = True
        print(
            f"cycle={cycle} state={state} detail={detail!r} "
            f"active_kty={active_kty} first_kty={self.first_kty} "
            f"locator={locator_exists} gate={gate_exists} "
            f"active={self.latest_active:.3f} outfeed={self.latest_outfeed:.3f} "
            f"last_nonzero={self.last_nonzero_outfeed:.3f} "
            f"despawned_cycles={despawned_cycles} last_despawned={last_despawned!r} "
            f"transport={data.get('transport')} "
            f"fill={float(data.get('estimated_fill_ratio', 0.0) or 0.0):.3f}",
            flush=True,
        )


rclpy.init()
node = Observer()
deadline = time.monotonic() + 360.0
success = False
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.error:
            print(f"ERROR state: {node.error}; retained outfeed={node.last_nonzero_outfeed:.3f}")
            break
        required = {
            'LOAD', 'CLOSE_GATE', 'COMPACT', 'EJECT_ACTIVE', 'DESPAWN_ACTIVE',
            'POSITION_NEXT', 'VERIFY_READY', 'OPEN_GATE'
        }
        if (
            required <= node.seen
            and node.transport_seen
            and node.release_zero_seen
            and node.locator_removed_seen
            and node.despawn_seen
            and node.despawn_confirmed_before_position
            and node.locator_recreated_seen
            and node.outfeed_nonzero
            and node.gate_closed_seen
            and node.gate_open_after_closed
            and node.second_load
        ):
            success = True
            print('OK: first loaded KTY was absent before the second KTY was positioned')
            break
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if success else 1)
PY
then
  echo 'FAIL: runtime-v11 despawn-first changeover was not observed' >&2
  failures=$((failures + 1))
fi

printf '\nCurrent KTY / gate / locator models:\n'
gz model --list 2>/dev/null | grep -E 'kty_mech_container_|kty_mech_chute_gate|kty_mech_runtime_locator' || true

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

Expected runtime v11:
  - generated world contains no roller links and no joint-driven locator;
  - kty_mech_runtime_locator is deleted before active/outfeed becomes nonzero;
  - the loaded KTY enters DESPAWN_ACTIVE after clearing the active zone;
  - carried products and the first KTY are confirmed absent before POSITION_NEXT;
  - the runtime locator is recreated only after the old KTY is absent;
  - kty_mech_chute_gate remains closed during changeover and opens for cycle 2.
EOF

if (( failures > 0 )); then
  echo "KTY contact-surface diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo 'KTY runtime-v11 despawn-first diagnostics: OK'
