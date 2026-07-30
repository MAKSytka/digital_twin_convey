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
from std_msgs.msg import Float64, String


class VibrationObserver(Node):
    def __init__(self):
        super().__init__('kty_vibration_acceptance_observer')
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state = 'WAITING'
        self.runtime_profile = ''
        self.compact_seen = False
        self.compact_finished = False
        self.command_min = 1.0
        self.command_max = -1.0
        self.frequency_min = 1.0e9
        self.frequency_max = 0.0
        self.accel_max = 0.0
        self.last_compaction = None
        self.create_subscription(String, '/kty/flow/state', self.on_state, state_qos)
        self.create_subscription(
            Float64,
            '/kty/mech/vibration/cmd_pos',
            self.on_command,
            50,
        )

    def on_command(self, message):
        if self.state == 'COMPACT':
            value = float(message.data)
            self.command_min = min(self.command_min, value)
            self.command_max = max(self.command_max, value)

    def on_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        previous = self.state
        self.state = str(data.get('state', ''))
        self.runtime_profile = str(data.get('runtime_profile', ''))
        if self.state == 'COMPACT':
            self.compact_seen = True
            frequency = float(data.get('vibration_frequency_hz', 0.0) or 0.0)
            acceleration = float(data.get('vibration_peak_accel_g', 0.0) or 0.0)
            if frequency > 0.0:
                self.frequency_min = min(self.frequency_min, frequency)
                self.frequency_max = max(self.frequency_max, frequency)
            self.accel_max = max(self.accel_max, acceleration)
        elif self.compact_seen and previous == 'COMPACT':
            self.compact_finished = True
        compaction = data.get('last_compaction')
        if isinstance(compaction, dict) and any(
            abs(float(value or 0.0)) > 1.0e-12 for value in compaction.values()
        ):
            self.last_compaction = compaction


rclpy.init()
node = VibrationObserver()
deadline = time.monotonic() + 360.0
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.compact_finished and node.last_compaction is not None:
            break
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()

if not node.compact_seen:
    raise SystemExit('FAIL: COMPACT state was not observed in 360 s')

peak_positive_mm = 1000.0 * node.command_max
peak_negative_mm = 1000.0 * node.command_min
peak_to_peak_mm = 1000.0 * (node.command_max - node.command_min)
print(f'Runtime profile: {node.runtime_profile}')
print(f'Command extrema: {peak_negative_mm:+.2f} .. {peak_positive_mm:+.2f} mm')
print(f'Command peak-to-peak: {peak_to_peak_mm:.2f} mm')
print(f'Observed frequency range: {node.frequency_min:.2f} .. {node.frequency_max:.2f} Hz')
print(f'Max theoretical acceleration: {node.accel_max:.2f} g')

failures = 0
if node.runtime_profile != 'kty_mechatronics_v11':
    print('FAIL: runtime-v11 controller was not active')
    failures += 1
if node.command_min > -0.0072 or node.command_max < 0.0072:
    print('FAIL: command did not reach approximately +/-8 mm')
    failures += 1
if node.frequency_min > 6.9 or node.frequency_max < 8.6:
    print('FAIL: frequency sweep did not cover enough of the 6.5..9.0 Hz range')
    failures += 1
if node.accel_max < 1.3:
    print('FAIL: commanded acceleration never exceeded 1.3 g')
    failures += 1

if node.last_compaction is None:
    print('FAIL: no post-compaction measurement was published')
    failures += 1
else:
    compaction = node.last_compaction
    before = 1000.0 * float(compaction.get('height_before_m', 0.0) or 0.0)
    after = 1000.0 * float(compaction.get('height_after_m', 0.0) or 0.0)
    drop = 1000.0 * float(compaction.get('height_drop_m', 0.0) or 0.0)
    fill_before = 100.0 * float(compaction.get('fill_before', 0.0) or 0.0)
    fill_after = 100.0 * float(compaction.get('fill_after', 0.0) or 0.0)
    print(f'Max product height: {before:.1f} -> {after:.1f} mm; drop={drop:+.1f} mm')
    print(f'Envelope fill estimate: {fill_before:.1f}% -> {fill_after:.1f}%')
    if drop <= 0.0:
        print('WARNING: this cycle did not reduce maximum height; inspect actual joint travel and product contacts.')

if failures:
    raise SystemExit(f'KTY vibration diagnostics failed: {failures} problem(s)')
print('KTY runtime-v11 vibration command and compaction telemetry: OK')
PY

cat <<'EOF'

Actual Gazebo joint telemetry (manual inspection):
  gz topic -e -t /kty/mech/joint_state | grep -A12 -B2 vibration_joint

During COMPACT the joint position should repeatedly approach roughly -0.008 .. +0.008 m.
EOF
