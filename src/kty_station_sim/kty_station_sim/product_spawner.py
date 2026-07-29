"""Random 1 product/s feeder for the KTY chute."""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
import random
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32

from singulator_interfaces.msg import KtyGroundTruth, KtyGroundTruthArray

from .model_factory import ProductSpec


class ProductSpawner(Node):
    def __init__(self) -> None:
        super().__init__("product_spawner")
        defaults = {
            "world_name": "kty_station",
            "rate_products_per_s": 1.0,
            "seed": 42,
            "spawn_x_m": -1.02,
            "spawn_y_half_range_m": 0.20,
            "spawn_z_m": 1.57,
            "spawn_clearance_m": 0.01,
            "service_timeout_ms": 5000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.world_name = str(self.get_parameter("world_name").value)
        self.rate = float(self.get_parameter("rate_products_per_s").value)
        self.spawn_x = float(self.get_parameter("spawn_x_m").value)
        self.spawn_y_half_range = float(
            self.get_parameter("spawn_y_half_range_m").value
        )
        self.spawn_z = float(self.get_parameter("spawn_z_m").value)
        self.spawn_clearance = float(
            self.get_parameter("spawn_clearance_m").value
        )
        self.service_timeout_ms = int(
            self.get_parameter("service_timeout_ms").value
        )
        if self.rate <= 0.0:
            raise ValueError("rate_products_per_s must be positive")

        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.enabled = False
        self.cycle_id = 0
        self.product_id = 0
        self.registry: list[KtyGroundTruth] = []
        self.model_names: set[str] = set()
        self.lock = threading.Lock()
        self.spawn_future: Future | None = None
        self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="product_entity")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.registry_pub = self.create_publisher(
            KtyGroundTruthArray,
            "/kty/ground_truth/registry",
            qos,
        )
        self.create_subscription(
            Bool,
            "/kty/product_spawner/enabled",
            self._on_enabled,
            10,
        )
        self.create_subscription(
            Bool,
            "/kty/product_spawner/clear",
            self._on_clear,
            10,
        )
        self.create_subscription(UInt32, "/kty/cycle_id", self._on_cycle, 10)
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

    def _on_enabled(self, message: Bool) -> None:
        self.enabled = bool(message.data)

    def _on_cycle(self, message: UInt32) -> None:
        new_cycle = int(message.data)
        if new_cycle == self.cycle_id:
            return
        self.cycle_id = new_cycle
        self.product_id = 0
        self.registry.clear()
        self._publish_registry()

    def _on_clear(self, message: Bool) -> None:
        if not message.data:
            return
        self.enabled = False
        with self.lock:
            names = list(self.model_names)
            self.model_names.clear()
            self.registry.clear()
        self._publish_registry()
        for name in names:
            self.pool.submit(self._remove_model, name)

    def _sample_spec_and_y(self) -> tuple[ProductSpec, float]:
        for _ in range(300):
            spec = ProductSpec.random(self.rng)
            projected_y = (
                abs(spec.size_x * math.sin(spec.yaw))
                + abs(spec.size_y * math.cos(spec.yaw))
            )
            maximum_center = min(
                self.spawn_y_half_range,
                max(0.0, 0.285 - projected_y / 2.0),
            )
            if maximum_center >= 0.005:
                return spec, self.rng.uniform(-maximum_center, maximum_center)
        raise RuntimeError("Unable to fit a random product on the 600 mm chute")

    def _on_timer(self) -> None:
        if not self.enabled or self.cycle_id <= 0:
            return
        with self.lock:
            if self.spawn_future is not None:
                if not self.spawn_future.done():
                    return
                try:
                    success, truth = self.spawn_future.result()
                except Exception as error:  # pragma: no cover - runtime guard
                    self.get_logger().error(f"Product spawn exception: {error}")
                    success, truth = False, None
                self.spawn_future = None
                if success and truth is not None:
                    self.registry.append(truth)
                    self.model_names.add(truth.model_name)
                    self._publish_registry()

            self.product_id += 1
            spec, spawn_y = self._sample_spec_and_y()
            model_name = (
                f"kty_product_c{self.cycle_id:06d}_p{self.product_id:06d}"
            )
            self.spawn_future = self.pool.submit(
                self._spawn_product,
                model_name,
                spec,
                spawn_y,
                self.cycle_id,
                self.product_id,
            )

    def _spawn_product(
        self,
        model_name: str,
        spec: ProductSpec,
        spawn_y: float,
        cycle_id: int,
        product_id: int,
    ) -> tuple[bool, KtyGroundTruth | None]:
        sdf = spec.to_sdf(model_name)
        half_yaw = spec.yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        center_z = self.spawn_z + spec.size_z / 2.0 + self.spawn_clearance
        request = "\n".join(
            (
                f"sdf: {json.dumps(sdf)}",
                f'name: "{model_name}"',
                "allow_renaming: false",
                "pose {",
                f"  position {{ x: {self.spawn_x:.9f} y: {spawn_y:.9f} z: {center_z:.9f} }}",
                f"  orientation {{ x: 0 y: 0 z: {qz:.12f} w: {qw:.12f} }}",
                "}",
            )
        )
        command = [
            "gz", "service", "-s", f"/world/{self.world_name}/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(self.service_timeout_ms),
            "--req", request,
        ]
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=self.service_timeout_ms / 1000.0 + 2.0,
            check=False,
        )
        success = result.returncode == 0 and "data: true" in result.stdout.lower()
        if not success:
            self.get_logger().error(
                f"Failed to spawn {model_name}: {result.stdout} {result.stderr}"
            )
            return False, None

        truth = KtyGroundTruth()
        truth.header.stamp = self.get_clock().now().to_msg()
        truth.header.frame_id = "kty_station"
        truth.cycle_id = cycle_id
        truth.product_id = product_id
        truth.model_name = model_name
        truth.profile = spec.profile
        truth.size_m.x = spec.size_x
        truth.size_m.y = spec.size_y
        truth.size_m.z = spec.size_z
        truth.mass_kg = spec.mass
        self.get_logger().info(
            f"Spawned {model_name}: {spec.profile}, "
            f"{spec.size_x:.3f}x{spec.size_y:.3f}x{spec.size_z:.3f} m, "
            f"{spec.mass:.3f} kg"
        )
        return True, truth

    def _remove_model(self, model_name: str) -> bool:
        command = [
            "gz", "service", "-s", f"/world/{self.world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(self.service_timeout_ms),
            "--req", f'name: "{model_name}" type: MODEL',
        ]
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=self.service_timeout_ms / 1000.0 + 2.0,
            check=False,
        )
        return result.returncode == 0 and "data: true" in result.stdout.lower()

    def _publish_registry(self) -> None:
        message = KtyGroundTruthArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "kty_station"
        message.cycle_id = self.cycle_id
        message.products = list(self.registry)
        self.registry_pub.publish(message)

    def close(self) -> None:
        self.pool.shutdown(wait=False, cancel_futures=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProductSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
