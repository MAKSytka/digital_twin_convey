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
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("kty_runtime_v18_check")
latest = None
load_cycles = set()
error = None
max_recoveries = 0
last_detail = ""


def callback(message):
    global latest, error, max_recoveries, last_detail
    try:
        latest = json.loads(message.data)
    except json.JSONDecodeError:
        return
    state = str(latest.get("state", ""))
    last_detail = str(latest.get("detail", ""))
    if state == "LOAD":
        load_cycles.add(int(latest.get("cycle_id", 0)))
    if state == "ERROR":
        error = last_detail or "runtime entered ERROR"
    max_recoveries = max(
        max_recoveries,
        int(latest.get("position_recovery_respawns", 0) or 0),
    )


node.create_subscription(String, "/kty/flow/state", callback, 10)
deadline = time.monotonic() + 420.0
last_print = 0.0
while time.monotonic() < deadline and len(load_cycles) < 4 and error is None:
    rclpy.spin_once(node, timeout_sec=0.25)
    now = time.monotonic()
    if latest is not None and now - last_print >= 5.0:
        print(
            "cycle={} state={} active={} queue={} recoveries={} detail={}".format(
                latest.get("cycle_id"),
                latest.get("state"),
                latest.get("active_kty"),
                latest.get("queue_kty"),
                latest.get("position_recovery_respawns", 0),
                last_detail,
            ),
            flush=True,
        )
        last_print = now

try:
    if latest is None:
        raise SystemExit("FAIL: no /kty/flow/state received")
    if latest.get("runtime_profile") != "kty_mechatronics_v18":
        raise SystemExit(
            f"FAIL: expected runtime v18, got {latest.get('runtime_profile')!r}"
        )
    if error is not None:
        raise SystemExit(f"FAIL: {error}")
    if len(load_cycles) < 4:
        raise SystemExit(
            f"FAIL: observed LOAD cycles {sorted(load_cycles)}, expected at least four"
        )
    expected_max = [0.280, 0.190, 0.145]
    if latest.get("product_size_max_m") != expected_max:
        raise SystemExit(
            f"FAIL: unexpected product maximum {latest.get('product_size_max_m')!r}"
        )
    print(
        f"OK runtime v18: LOAD cycles={sorted(load_cycles)} "
        f"deterministic_recoveries={max_recoveries}"
    )
    print("KTY runtime-v18 four-cycle continuity: OK")
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY
