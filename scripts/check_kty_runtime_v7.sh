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

printf 'Runtime-v7 nodes:\n'
for node in \
  /kty_mechatronics_cycle_v2 \
  /kty_fill_estimator_v2 \
  /kty_classical_3d_perception_v2 \
  /kty_contour_recorder_3d \
  /kty_vision_dashboard_3d \
  /kty_mechatronics_command_bridge; do
  wait_node "$node" || true
done

printf '\nCore samples:\n'
timeout 8 ros2 topic echo /kty/fill/state --once || {
  echo 'FAIL: no corrected fill estimate' >&2
  failures=$((failures + 1))
}
timeout 8 ros2 topic echo /kty/flow/state --once || {
  echo 'FAIL: no mechatronics state' >&2
  failures=$((failures + 1))
}

printf '\nWaiting for slide-gate changeover and outfeed progress:\n'
if ! python3 - <<'PY'
import json
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Observer(Node):
    def __init__(self):
        super().__init__('kty_runtime_v7_observer')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.seen = set()
        self.gate_closed_seen = False
        self.gate_open_after_closed = False
        self.second_load = False
        self.error = ''
        self.last_cycle = 0
        self.create_subscription(String, '/kty/flow/state', self.on_state, qos)

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

    def on_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(data.get('state', ''))
        cycle = int(data.get('cycle_id', 0) or 0)
        self.last_cycle = max(self.last_cycle, cycle)
        self.seen.add(state)
        if state == 'ERROR':
            self.error = str(data.get('detail', 'unknown error'))
        exists = self.gate_exists()
        if state in {'CLOSE_GATE', 'COMPACT', 'EJECT_ACTIVE', 'POSITION_NEXT', 'VERIFY_READY'} and exists:
            self.gate_closed_seen = True
        if self.gate_closed_seen and state in {'OPEN_GATE', 'LOAD'} and not exists:
            self.gate_open_after_closed = True
        if state == 'LOAD' and cycle >= 2:
            self.second_load = True
        print(
            f"cycle={cycle} state={state} gate_model={exists} "
            f"fill={float(data.get('estimated_fill_ratio', 0.0) or 0.0):.3f} "
            f"height={float(data.get('maximum_height_m', 0.0) or 0.0):.3f}",
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
        if required <= node.seen and node.gate_closed_seen and node.gate_open_after_closed and node.second_load:
            success = True
            print('OK: slide gate closed, loaded KTY exited and second KTY entered LOAD')
            break
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if success else 1)
PY
then
  echo 'FAIL: corrected physical changeover was not observed' >&2
  failures=$((failures + 1))
fi

printf '\nCurrent KTY / gate models:\n'
gz model --list 2>/dev/null | grep -E 'kty_mech_container_|kty_mech_chute_gate' || true

printf '\n3-D perception heartbeat:\n'
if timeout 15 ros2 topic echo /kty/perception/contours --once >/tmp/kty_v7_contours.txt; then
  sed -n '1,45p' /tmp/kty_v7_contours.txt
  echo 'OK: 3-D contour frame received'
else
  echo 'FAIL: no 3-D contour frame; inspect /kty/perception/fault and launch terminal' >&2
  timeout 3 ros2 topic echo /kty/perception/fault --once || true
  failures=$((failures + 1))
fi

printf '\nGazebo real-time factor:\n'
stats="$(timeout 5 gz topic -e -t /world/kty_mechatronics_v2/stats -n 1 2>/dev/null || true)"
printf '%s\n' "$stats" | sed -n '1,25p'
python3 - "$stats" <<'PY'
import re
import sys
match = re.search(r'real_time_factor:\s*([0-9.]+)', sys.argv[1])
if match:
    value = float(match.group(1))
    print(f'Measured RTF: {value:.3f}')
    if value < 0.45:
        print('WARNING: RTF remains below 0.45; keep show_dashboard:=false for mechanics tests.')
PY

cat <<'EOF'

Runtime-v7 expected changes:
  - roller axles stay parallel to Y and no roller orbits around the machine origin;
  - every roller has its own JointController on the shared group topic;
  - product interval is 1.15 s by default (1.77x slower than 0.65 s);
  - kty_mech_chute_gate exists while closed and is absent while open;
  - camera is 640x480 at 8 Hz; 3-D processing is throttled to 4 Hz;
  - show_dashboard defaults to false for the first transport acceptance run.
EOF

if (( failures > 0 )); then
  echo "KTY runtime v7 diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo 'KTY runtime v7 diagnostics: OK'
