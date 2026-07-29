"""Publish a std_msgs/String mirror of the KTY ground-truth registry.

The typed topic remains the canonical machine interface.  The JSON mirror is a
human-facing diagnostic channel which can be inspected even from a shell where
the workspace overlay has not been sourced.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from singulator_interfaces.msg import KtyGroundTruthArray


class RegistryJsonMirror(Node):
    def __init__(self) -> None:
        super().__init__("kty_registry_json_mirror")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            String,
            "/kty/ground_truth/registry_json",
            qos,
        )
        self.create_subscription(
            KtyGroundTruthArray,
            "/kty/ground_truth/registry",
            self._on_registry,
            qos,
        )

    def _on_registry(self, message: KtyGroundTruthArray) -> None:
        payload = {
            "cycle_id": int(message.cycle_id),
            "product_count": len(message.products),
            "products": [
                {
                    "product_id": int(item.product_id),
                    "model_name": item.model_name,
                    "profile": item.profile,
                    "size_m": [
                        float(item.size_m.x),
                        float(item.size_m.y),
                        float(item.size_m.z),
                    ],
                    "mass_kg": float(item.mass_kg),
                }
                for item in message.products
            ],
        }
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RegistryJsonMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
