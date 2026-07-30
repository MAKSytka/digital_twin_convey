#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

for node in /kty_mechatronics_cycle_v3 /kty_model_pose_bridge; do
  ros2 node list 2>/dev/null | grep -Fxq "$node" || {
    echo "FAIL node: $node" >&2
    exit 1
  }
  echo "OK node: $node"
done

ros2 topic list 2>/dev/null | grep -Fxq /kty/mech/model_poses || {
  echo "FAIL topic: /kty/mech/model_poses" >&2
  exit 1
}
echo "OK topic: /kty/mech/model_poses"

python3 - <<'PY'
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Observer(Node):
    def __init__(self):
        super().__init__('kty_runtime_v13_acceptance_observer')
        qos = QoSProfile(depth=30)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.runtime = ''
        self.error = ''
        self.pose_feedback = ''
        self.max_pose_age = 0.0
        self.first_close_guard = None
        self.prefeed_seen = False
        self.despawn_seen = False
        self.open_seen = False
        self.second_load_seen = False
        self.closed_buffer_max = 0
        self.create_subscription(String, '/kty/flow/state', self.on_state, qos)

    def on_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(data.get('state', ''))
        cycle = int(data.get('cycle_id', 0) or 0)
        self.runtime = str(data.get('runtime_profile', ''))
        self.pose_feedback = str(data.get('pose_feedback', ''))
        pose_age = float(data.get('pose_cache_age_s', 99.0) or 99.0)
        self.max_pose_age = max(self.max_pose_age, pose_age)
        self.prefeed_seen = self.prefeed_seen or bool(data.get('prefeed_reached', False))
        self.closed_buffer_max = max(
            self.closed_buffer_max,
            int(data.get('closed_gate_spawned_products', 0) or 0),
        )
        if state == 'ERROR':
            self.error = str(data.get('detail', 'unknown error'))
        guard = data.get('load_guard')
        if state == 'CLOSE_GATE' and self.first_close_guard is None and isinstance(guard, dict):
            self.first_close_guard = dict(guard)
        if state == 'DESPAWN_ACTIVE':
            self.despawn_seen = True
        if state == 'OPEN_GATE' and bool(data.get('gate_open_confirmed', False)):
            self.open_seen = True
        if state == 'LOAD' and cycle >= 2 and bool(data.get('gate_open_confirmed', False)):
            self.second_load_seen = True
        print(
            f"cycle={cycle} state={state} pose_age={pose_age:.2f}s "
            f"prefeed={data.get('prefeed_reached')} buffered={data.get('closed_gate_spawned_products')} "
            f"gate_confirmed={data.get('gate_open_confirmed')}",
            flush=True,
        )


rclpy.init()
node = Observer()
deadline = time.monotonic() + 540.0
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.error:
            break
        if node.second_load_seen and node.prefeed_seen and node.despawn_seen:
            break
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()

failures = 0
if node.runtime != 'kty_mechatronics_v13':
    print(f'FAIL runtime: {node.runtime!r}')
    failures += 1
if node.pose_feedback != 'persistent_ros_gz_pose_bridge':
    print(f'FAIL pose feedback: {node.pose_feedback!r}')
    failures += 1
if node.error:
    print(f'FAIL ERROR state: {node.error}')
    failures += 1
if node.first_close_guard is None:
    print('FAIL: first guarded CLOSE_GATE not observed')
    failures += 1
else:
    guard = node.first_close_guard
    spawned = int(guard.get('spawned_products', 0) or 0)
    elapsed = float(guard.get('elapsed_s', 0.0) or 0.0)
    if spawned < 3 or elapsed < 3.8:
        print(f'FAIL early close: spawned={spawned}, elapsed={elapsed:.1f}s')
        failures += 1
if not node.prefeed_seen:
    print('FAIL: queued KTY never reached the prefeed staging point')
    failures += 1
if not node.despawn_seen:
    print('FAIL: DESPAWN_ACTIVE not observed')
    failures += 1
if not node.open_seen or not node.second_load_seen:
    print('FAIL: gate did not reopen for second LOAD')
    failures += 1
if node.closed_buffer_max > 5:
    print(f'FAIL: closed-gate buffer exceeded cap: {node.closed_buffer_max}')
    failures += 1

if failures:
    raise SystemExit(f'KTY runtime-v13 diagnostics failed: {failures} problem(s)')
print('KTY runtime-v13 persistent pose, prefeed and second gate opening: OK')
PY
