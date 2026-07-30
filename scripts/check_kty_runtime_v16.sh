#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

python3 - <<'PY'
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("kty_runtime_v16_probe")
        self.latest = {}
        self.registry = {}
        self.load_cycles: set[int] = set()
        self.open_gate_cycles: set[int] = set()
        self.gate_violation = False
        self.error = ""
        self.last_printed = None
        self.create_subscription(String, "/kty/flow/state", self.on_state, 20)
        self.create_subscription(
            String,
            "/kty/mech/model_pose_registry_json",
            self.on_registry,
            20,
        )

    def on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.latest = payload
        runtime = payload.get("runtime_profile")
        state = str(payload.get("state", ""))
        cycle = int(payload.get("cycle_id", 0) or 0)
        gate_present = bool(payload.get("gate_registry_present", False))
        key = (cycle, state, gate_present)
        if key != self.last_printed:
            print(
                f"cycle={cycle} state={state} gate_present={gate_present} "
                f"live_products={payload.get('registry_live_products')} "
                f"spawn_failures={payload.get('product_spawn_failures')} "
                f"backoff={payload.get('feeder_backoff_active')}"
            )
            self.last_printed = key
        if runtime != "kty_mechatronics_v16":
            return
        if state == "LOAD":
            self.load_cycles.add(cycle)
            if gate_present:
                self.gate_violation = True
        elif state == "OPEN_GATE":
            self.open_gate_cycles.add(cycle)
        elif state == "ERROR":
            self.error = str(payload.get("detail", "unknown runtime error"))

    def on_registry(self, message: String) -> None:
        try:
            self.registry = json.loads(message.data)
        except json.JSONDecodeError:
            pass


rclpy.init()
node = Probe()
deadline = time.monotonic() + 300.0
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
        if node.error:
            raise SystemExit(f"FAIL runtime entered ERROR: {node.error}")
        if node.gate_violation:
            raise SystemExit("FAIL physical slide gate was present during LOAD")
        if len(node.load_cycles) >= 3 and len(node.open_gate_cycles) >= 2:
            latest = node.latest
            if int(latest.get("gate_confirmed_open_count", 0) or 0) < 3:
                raise SystemExit("FAIL insufficient registry-confirmed gate openings")
            live_products = int(latest.get("registry_live_products", 0) or 0)
            maximum = int(latest.get("maximum_products_per_load", 28) or 28)
            if live_products > maximum + 10:
                raise SystemExit(
                    f"FAIL live product count grew unexpectedly: {live_products}"
                )
            models = node.registry.get("models", [])
            containers = sum(
                str(item.get("name", "")).startswith("kty_mech_container_")
                for item in models
                if isinstance(item, dict)
            )
            print(
                "OK runtime v16: "
                f"load_cycles={sorted(node.load_cycles)} "
                f"open_gate_cycles={sorted(node.open_gate_cycles)} "
                f"containers={containers} live_products={live_products} "
                f"spawn_recoveries={latest.get('product_spawn_recoveries')}"
            )
            print("KTY runtime-v16 multi-cycle lifecycle and feeder recovery: OK")
            break
    else:
        raise SystemExit(
            "FAIL timeout waiting for three LOAD cycles; "
            f"seen LOAD={sorted(node.load_cycles)} OPEN_GATE={sorted(node.open_gate_cycles)} "
            f"latest={node.latest}"
        )
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY
