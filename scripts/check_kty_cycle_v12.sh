#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

python3 - <<'PY'
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
        super().__init__('kty_cycle_v12_acceptance_observer')
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.runtime = ''
        self.error = ''
        self.first_close_guard = None
        self.despawn_seen = False
        self.position_next_seen = False
        self.open_gate_seen = False
        self.second_load_seen = False
        self.second_load_gate_confirmed = False
        self.first_kty = ''
        self.last_despawned = ''
        self.create_subscription(String, '/kty/flow/state', self.on_state, qos)

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

    def on_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(data.get('state', ''))
        cycle = int(data.get('cycle_id', 0) or 0)
        self.runtime = str(data.get('runtime_profile', ''))
        if not self.first_kty:
            self.first_kty = str(data.get('active_kty', ''))
        if state == 'ERROR':
            self.error = str(data.get('detail', 'unknown error'))
        guard = data.get('load_guard')
        if state == 'CLOSE_GATE' and self.first_close_guard is None and isinstance(guard, dict):
            self.first_close_guard = dict(guard)
        if state == 'DESPAWN_ACTIVE':
            self.despawn_seen = True
        if state == 'POSITION_NEXT':
            self.position_next_seen = True
        if state == 'OPEN_GATE':
            self.open_gate_seen = True
        if state == 'LOAD' and cycle >= 2:
            self.second_load_seen = True
            self.second_load_gate_confirmed = bool(data.get('gate_open_confirmed', False))
        self.last_despawned = str(data.get('last_despawned_kty', ''))
        print(
            f"cycle={cycle} state={state} runtime={self.runtime} "
            f"gate_open={data.get('gate_open')} gate_confirmed={data.get('gate_open_confirmed')} "
            f"despawned={self.last_despawned!r} recoveries={data.get('position_recovery_pulses')} "
            f"guard={guard}",
            flush=True,
        )


rclpy.init()
node = Observer()
deadline = time.monotonic() + 480.0
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.error:
            break
        if node.second_load_seen and node.open_gate_seen and node.last_despawned:
            break
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()

failures = 0
if node.runtime != 'kty_mechatronics_v12':
    print('FAIL: runtime-v12 was not active')
    failures += 1
if node.error:
    print(f'FAIL: runtime entered ERROR: {node.error}')
    failures += 1
if node.first_close_guard is None:
    print('FAIL: first CLOSE_GATE was not observed with load_guard telemetry')
    failures += 1
else:
    guard = node.first_close_guard
    elapsed = float(guard.get('elapsed_s', 0.0) or 0.0)
    spawned = int(guard.get('spawned_products', 0) or 0)
    enough_time = bool(guard.get('enough_time', False))
    enough_products = bool(guard.get('enough_products', False))
    print(f'First close guard: elapsed={elapsed:.1f}s spawned={spawned} data={guard}')
    if not enough_time or elapsed < 3.8:
        print('FAIL: gate closed before minimum load duration')
        failures += 1
    if not enough_products or spawned < 3:
        print('FAIL: gate closed before at least three deterministic spawn events')
        failures += 1
if not node.despawn_seen or not node.last_despawned:
    print('FAIL: loaded KTY despawn was not observed')
    failures += 1
if node.first_kty and node.model_exists(node.first_kty):
    print(f'FAIL: first KTY still exists: {node.first_kty}')
    failures += 1
if not node.position_next_seen:
    print('FAIL: POSITION_NEXT was not observed')
    failures += 1
if not node.open_gate_seen:
    print('FAIL: OPEN_GATE was not observed')
    failures += 1
if not node.second_load_seen:
    print('FAIL: second LOAD was not observed')
    failures += 1
elif not node.second_load_gate_confirmed:
    print('FAIL: second LOAD began without confirmed gate removal')
    failures += 1

if failures:
    raise SystemExit(f'KTY runtime-v12 cycle diagnostics failed: {failures} problem(s)')
print('KTY runtime-v12 guarded fill and two-cycle changeover: OK')
PY
