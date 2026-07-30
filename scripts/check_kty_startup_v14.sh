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
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class StartupObserver(Node):
    def __init__(self):
        super().__init__('kty_runtime_v14_startup_observer')
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.latest = None
        self.error = ''
        self.create_subscription(String, '/kty/flow/state', self.on_state, qos)

    def on_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.latest = data
        if data.get('state') == 'ERROR':
            self.error = str(data.get('detail', 'unknown startup error'))
        print(
            'state={state} runtime={runtime} active={active!r} queue={queue!r} '
            'pose_age={age} confirmations={confirmations} gate_open={gate}'.format(
                state=data.get('state'),
                runtime=data.get('runtime_profile'),
                active=data.get('active_kty'),
                queue=data.get('queue_kty'),
                age=data.get('pose_cache_age_s'),
                confirmations=data.get('startup_pose_confirmations'),
                gate=data.get('gate_open'),
            ),
            flush=True,
        )


rclpy.init()
node = StartupObserver()
deadline = time.monotonic() + 90.0
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.error:
            break
        data = node.latest or {}
        if (
            data.get('runtime_profile') == 'kty_mechatronics_v14'
            and data.get('state') == 'LOAD'
            and data.get('startup_complete') is True
            and int(data.get('startup_pose_confirmations', 0) or 0) >= 2
            and bool(data.get('active_kty'))
            and bool(data.get('queue_kty'))
            and data.get('gate_open') is True
        ):
            break
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()

if node.error:
    raise SystemExit(f'FAIL: runtime entered ERROR: {node.error}')

data = node.latest or {}
failures = 0
if data.get('runtime_profile') != 'kty_mechatronics_v14':
    print('FAIL: runtime v14 is not active')
    failures += 1
if data.get('state') != 'LOAD':
    print(f"FAIL: startup did not reach LOAD; last state={data.get('state')}")
    failures += 1
if not data.get('active_kty') or not data.get('queue_kty'):
    print('FAIL: both initial KTY names were not assigned')
    failures += 1
if int(data.get('startup_pose_confirmations', 0) or 0) < 2:
    print('FAIL: both created KTY models were not confirmed through pose cache')
    failures += 1
if data.get('gate_open') is not True:
    print('FAIL: slide gate is not open at initial LOAD')
    failures += 1
if failures:
    raise SystemExit(f'KTY runtime-v14 startup diagnostics failed: {failures} problem(s)')
print('KTY runtime-v14 startup diagnostics: OK')
PY
