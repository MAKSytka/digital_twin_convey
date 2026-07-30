#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

ros2 node list | grep -Fxq /kty_model_pose_registry_bridge || {
  echo "FAIL node: /kty_model_pose_registry_bridge" >&2
  exit 1
}
ros2 node list | grep -Fxq /kty_mechatronics_cycle_v3 || {
  echo "FAIL node: /kty_mechatronics_cycle_v3" >&2
  exit 1
}

echo "OK nodes: registry bridge and mechatronics controller"

python3 - <<'PY'
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__("kty_runtime_v15_startup_probe")
        self.registry = None
        self.state = None
        self.create_subscription(
            String,
            "/kty/mech/model_pose_registry_json",
            self.on_registry,
            10,
        )
        self.create_subscription(String, "/kty/flow/state", self.on_state, 10)

    def on_registry(self, message):
        try:
            self.registry = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def on_state(self, message):
        try:
            self.state = json.loads(message.data)
        except json.JSONDecodeError:
            pass


rclpy.init()
node = Probe()
deadline = time.monotonic() + 70.0
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
        registry = node.registry or {}
        state = node.state or {}
        names = {
            str(item.get("name", ""))
            for item in registry.get("models", [])
            if isinstance(item, dict)
        }
        containers = {name for name in names if name.startswith("kty_mech_container_")}
        products = {name for name in names if name.startswith("kty_mech_product_")}
        if state.get("state") == "ERROR":
            raise SystemExit(f"FAIL controller ERROR: {state.get('detail')}")
        if (
            state.get("runtime_profile") == "kty_mechatronics_v15"
            and state.get("state") == "LOAD"
            and len(containers) >= 2
            and len(products) >= 1
            and state.get("pose_registry_sequence", 0) > 0
        ):
            print(
                "OK runtime v15 startup: "
                f"containers={len(containers)} products={len(products)} "
                f"registry_sequence={state.get('pose_registry_sequence')}"
            )
            break
    else:
        raise SystemExit(
            "FAIL: runtime v15 did not reach LOAD with two KTYs and one product"
        )
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY

echo "KTY runtime-v15 startup and product spawn: OK"
