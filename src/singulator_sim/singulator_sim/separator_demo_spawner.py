#!/usr/bin/env python3
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
import random
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from singulator_sim.box_model import BoxSpec


class SeparatorDemoSpawner(Node):
    """Create a controllable random flow on ten fixed transverse spots."""

    SPAWN_SPOTS_M = tuple(-1.0 + index * (2.0 / 9.0) for index in range(10))

    def __init__(self) -> None:
        super().__init__("separator_demo_spawner")
        self.declare_parameter("world_name", "infeed_size_separator_demo")
        self.declare_parameter("spawn_x", -3.20)
        self.declare_parameter("belt_top_z", 0.08)
        self.declare_parameter("conveyor_half_width_m", 1.25)
        self.declare_parameter("target_rate_boxes_per_sec", 4.0)
        self.declare_parameter("spawn_mode", "continuous")
        self.declare_parameter("maximum_items", 100)
        self.declare_parameter("small_item_probability", 0.20)
        self.declare_parameter("cutoff_m", 0.070)
        self.declare_parameter("seed", 42)
        self.declare_parameter("service_timeout_ms", 5000)
        self.declare_parameter("statistics_every_items", 20)

        self.world_name = str(self.get_parameter("world_name").value)
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.belt_top_z = float(self.get_parameter("belt_top_z").value)
        self.half_width = float(
            self.get_parameter("conveyor_half_width_m").value
        )
        self.target_rate = float(
            self.get_parameter("target_rate_boxes_per_sec").value
        )
        self.spawn_mode = str(self.get_parameter("spawn_mode").value).lower()
        self.maximum_items = int(
            self.get_parameter("maximum_items").value
        )
        self.small_probability = float(
            self.get_parameter("small_item_probability").value
        )
        self.cutoff = float(self.get_parameter("cutoff_m").value)
        self.service_timeout_ms = int(
            self.get_parameter("service_timeout_ms").value
        )
        self.statistics_every_items = int(
            self.get_parameter("statistics_every_items").value
        )
        seed = int(self.get_parameter("seed").value)

        self._validate_parameters()
        self.rng = random.Random(seed)
        self.session_id = int(time.time())
        self.item_index = 0
        self.created_count = 0
        self.expected_upper = 0
        self.expected_lower = 0
        self.skipped_busy = 0
        self.started_sim_ns: int | None = None

        self.state_lock = threading.Lock()
        self.spawn_in_progress = False
        self.spawn_pool = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="separator_spawn",
        )
        self.timer = self.create_timer(
            1.0 / self.target_rate,
            self._on_timer,
        )

        spots = ", ".join(f"{value:.3f}" for value in self.SPAWN_SPOTS_M)
        self.get_logger().info(
            "Separator flow configured: "
            f"mode={self.spawn_mode}, target_rate={self.target_rate:.2f} item/s, "
            f"small_probability={self.small_probability:.2f}, "
            f"cutoff={self.cutoff * 1000:.0f} mm"
        )
        self.get_logger().info(f"Ten fixed centre-of-mass spots Y=[{spots}] m")

    def _validate_parameters(self) -> None:
        if self.target_rate <= 0.0:
            raise ValueError("target_rate_boxes_per_sec must be positive")
        if self.spawn_mode not in {"continuous", "finite"}:
            raise ValueError("spawn_mode must be continuous or finite")
        if self.maximum_items <= 0:
            raise ValueError("maximum_items must be positive")
        if not 0.0 <= self.small_probability <= 1.0:
            raise ValueError("small_item_probability must be in [0, 1]")
        if self.cutoff <= 0.0:
            raise ValueError("cutoff_m must be positive")
        if self.half_width <= 0.0:
            raise ValueError("conveyor_half_width_m must be positive")
        if self.statistics_every_items <= 0:
            raise ValueError("statistics_every_items must be positive")

    @staticmethod
    def _projections(
        size_x: float,
        size_y: float,
        yaw: float,
    ) -> tuple[float, float]:
        projected_x = (
            abs(size_x * math.cos(yaw))
            + abs(size_y * math.sin(yaw))
        )
        projected_y = (
            abs(size_x * math.sin(yaw))
            + abs(size_y * math.cos(yaw))
        )
        return projected_x, projected_y

    @staticmethod
    def _mass(
        size_x: float,
        size_y: float,
        size_z: float,
        density: float,
    ) -> float:
        return max(
            0.01,
            min(5.0, size_x * size_y * size_z * density),
        )

    def _make_box(
        self,
        spot_index: int,
        expected_lower: bool,
    ) -> tuple[BoxSpec, tuple[float, float, float], float, float]:
        y = self.SPAWN_SPOTS_M[spot_index]

        for _ in range(1000):
            if expected_lower:
                size_x = self.rng.uniform(0.035, 0.069)
                size_y = self.rng.uniform(0.015, 0.069)
                size_z = self.rng.uniform(0.010, 0.080)
                yaw = math.radians(self.rng.uniform(-32.0, 32.0))
                density = self.rng.uniform(180.0, 700.0)
            else:
                size_x = self.rng.uniform(0.120, 0.400)
                size_y = self.rng.uniform(0.100, 0.320)
                size_z = self.rng.uniform(0.030, 0.280)
                yaw = math.radians(self.rng.uniform(-35.0, 35.0))
                density = self.rng.uniform(140.0, 650.0)

            projection_x, projection_y = self._projections(
                size_x,
                size_y,
                yaw,
            )
            classified_lower = min(projection_x, projection_y) < self.cutoff
            if classified_lower != expected_lower:
                continue

            if not expected_lower and min(projection_x, projection_y) < 0.090:
                continue

            if abs(y) + projection_y / 2.0 > self.half_width - 0.015:
                continue

            mass = self._mass(size_x, size_y, size_z, density)
            box = BoxSpec(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=spot_index,
            )
            if expected_lower:
                color = (
                    self.rng.uniform(0.82, 0.98),
                    self.rng.uniform(0.12, 0.45),
                    self.rng.uniform(0.04, 0.12),
                )
            else:
                color = (
                    self.rng.uniform(0.08, 0.25),
                    self.rng.uniform(0.45, 0.85),
                    self.rng.uniform(0.18, 0.85),
                )
            return box, color, projection_x, projection_y

        raise RuntimeError(
            f"Could not generate a fitting box for spot {spot_index}"
        )

    def _on_timer(self) -> None:
        if (
            self.spawn_mode == "finite"
            and self.item_index >= self.maximum_items
        ):
            self.timer.cancel()
            self.get_logger().info(
                "Finite separator flow finished; existing boxes keep moving"
            )
            return

        with self.state_lock:
            if self.spawn_in_progress:
                self.skipped_busy += 1
                if self.skipped_busy % 20 == 1:
                    self.get_logger().warning(
                        "Spawn service is still busy; a flow tick was skipped"
                    )
                return
            self.spawn_in_progress = True

        expected_lower = self.rng.random() < self.small_probability
        spot_index = self.rng.randrange(len(self.SPAWN_SPOTS_M))
        try:
            box, color, projection_x, projection_y = self._make_box(
                spot_index,
                expected_lower,
            )
        except Exception:
            with self.state_lock:
                self.spawn_in_progress = False
            raise

        route = "lower" if expected_lower else "upper"
        model_name = (
            f"box_separator_{self.session_id}_"
            f"n{self.item_index:07d}_exp_{route}_spot{spot_index:02d}"
        )
        self.item_index += 1

        self.get_logger().info(
            f"Spawn {model_name}: "
            f"{box.size_x * 1000:.0f}x{box.size_y * 1000:.0f}x"
            f"{box.size_z * 1000:.0f} mm, "
            f"yaw={math.degrees(box.yaw):.1f} deg, "
            f"projection={projection_x * 1000:.0f}x"
            f"{projection_y * 1000:.0f} mm, expected={route.upper()}"
        )

        future = self.spawn_pool.submit(
            self._spawn_item,
            model_name,
            box,
            color,
            expected_lower,
        )
        future.add_done_callback(self._on_spawn_finished)

    def _spawn_item(
        self,
        model_name: str,
        box: BoxSpec,
        color: tuple[float, float, float],
        expected_lower: bool,
    ) -> tuple[bool, bool]:
        sdf = box.to_sdf(model_name=model_name, color=color)
        spawn_z = self.belt_top_z + box.size_z / 2.0 + 0.008
        escaped_sdf = json.dumps(sdf)
        request = "\n".join(
            (
                "data {",
                f"  sdf: {escaped_sdf}",
                f'  name: "{model_name}"',
                "  allow_renaming: false",
                "  pose {",
                "    position {",
                f"      x: {self.spawn_x:.9f}",
                f"      y: {box.y:.9f}",
                f"      z: {spawn_z:.9f}",
                "    }",
                "  }",
                "}",
            )
        )
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/create_multiple",
            "--reqtype",
            "gz.msgs.EntityFactory_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self.service_timeout_ms),
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self.service_timeout_ms / 1000.0 + 2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f"Gazebo spawn failed: {error}")
            return False, expected_lower

        success = (
            result.returncode == 0
            and "data: true" in result.stdout.lower()
        )
        if not success:
            self.get_logger().error(
                "Gazebo rejected separator item. "
                f"stdout={result.stdout.strip()} "
                f"stderr={result.stderr.strip()}"
            )
        return success, expected_lower

    def _on_spawn_finished(self, future: Future) -> None:
        try:
            success, expected_lower = future.result()
            if success:
                self.created_count += 1
                if expected_lower:
                    self.expected_lower += 1
                else:
                    self.expected_upper += 1
                if self.started_sim_ns is None:
                    self.started_sim_ns = self.get_clock().now().nanoseconds
                if self.created_count % self.statistics_every_items == 0:
                    self._log_statistics()
        except Exception as error:
            self.get_logger().error(f"Separator spawn task failed: {error}")
        finally:
            with self.state_lock:
                self.spawn_in_progress = False

    def _log_statistics(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (
            (now_ns - self.started_sim_ns) / 1e9
            if self.started_sim_ns is not None
            else 0.0
        )
        actual_rate = (
            (self.created_count - 1) / elapsed
            if elapsed > 0.0
            else 0.0
        )
        self.get_logger().info(
            "Spawner statistics: "
            f"created={self.created_count}, "
            f"expected_upper={self.expected_upper}, "
            f"expected_lower={self.expected_lower}, "
            f"actual_rate={actual_rate:.2f} item/s, "
            f"busy_ticks={self.skipped_busy}"
        )

    def destroy_node(self) -> bool:
        self.timer.cancel()
        self.spawn_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeparatorDemoSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
