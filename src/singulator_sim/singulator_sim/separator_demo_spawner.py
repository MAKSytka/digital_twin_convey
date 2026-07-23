#!/usr/bin/env python3
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from singulator_sim.box_model import BoxSpec


class SeparatorDemoSpawner(Node):
    """Spawns a finite deterministic sequence of accepted and rejected items."""

    SCENARIOS = (
        (
            "accepted_large",
            "UPPER",
            BoxSpec(0.240, 0.180, 0.120, 1.20, -0.18, 0.00),
            (0.20, 0.65, 0.25),
        ),
        (
            "rejected_small",
            "LOWER",
            BoxSpec(0.055, 0.040, 0.025, 0.04, 0.00, 0.00),
            (0.95, 0.45, 0.08),
        ),
        (
            "accepted_medium",
            "UPPER",
            BoxSpec(0.160, 0.105, 0.080, 0.42, 0.20, 0.00),
            (0.18, 0.45, 0.92),
        ),
        (
            "rejected_flat",
            "LOWER",
            BoxSpec(0.070, 0.048, 0.015, 0.03, -0.08, math.radians(8.0)),
            (0.90, 0.18, 0.18),
        ),
        (
            "accepted_diagonal",
            "UPPER",
            BoxSpec(0.220, 0.140, 0.085, 0.68, 0.12, math.radians(18.0)),
            (0.55, 0.25, 0.85),
        ),
    )

    def __init__(self) -> None:
        super().__init__("separator_demo_spawner")
        self.declare_parameter("world_name", "infeed_size_separator_demo")
        self.declare_parameter("spawn_x", -1.72)
        self.declare_parameter("belt_top_z", 0.08)
        self.declare_parameter("spawn_period_s", 1.60)
        self.declare_parameter("cycles", 3)
        self.declare_parameter("service_timeout_ms", 5000)

        self.world_name = str(self.get_parameter("world_name").value)
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.belt_top_z = float(self.get_parameter("belt_top_z").value)
        self.spawn_period_s = float(
            self.get_parameter("spawn_period_s").value
        )
        self.cycles = int(self.get_parameter("cycles").value)
        self.service_timeout_ms = int(
            self.get_parameter("service_timeout_ms").value
        )

        if self.spawn_period_s <= 0.0:
            raise ValueError("spawn_period_s must be positive")
        if self.cycles <= 0:
            raise ValueError("cycles must be positive")

        self.session_id = int(time.time())
        self.item_index = 0
        self.maximum_items = self.cycles * len(self.SCENARIOS)
        self.state_lock = threading.Lock()
        self.spawn_in_progress = False
        self.spawn_pool = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="separator_spawn",
        )
        self.timer = self.create_timer(
            self.spawn_period_s,
            self._on_timer,
        )

        self.get_logger().info(
            "Separator demo sequence: "
            f"{self.maximum_items} items, "
            "green/blue -> upper path, orange/red -> lower path"
        )

    def _on_timer(self) -> None:
        if self.item_index >= self.maximum_items:
            self.timer.cancel()
            self.get_logger().info(
                "Separator demo sequence finished. "
                "Restart the launch to repeat it."
            )
            return

        with self.state_lock:
            if self.spawn_in_progress:
                self.get_logger().warning(
                    "Previous spawn is still running; current tick skipped"
                )
                return
            self.spawn_in_progress = True

        scenario_index = self.item_index % len(self.SCENARIOS)
        cycle_index = self.item_index // len(self.SCENARIOS)
        label, expected_path, box, color = self.SCENARIOS[scenario_index]
        model_name = (
            f"box_separator_{self.session_id}_"
            f"c{cycle_index:02d}_{label}"
        )
        self.item_index += 1

        self.get_logger().info(
            f"Spawn {model_name}: "
            f"{box.size_x * 1000:.0f}x{box.size_y * 1000:.0f}x"
            f"{box.size_z * 1000:.0f} mm, expected={expected_path}"
        )

        future = self.spawn_pool.submit(
            self._spawn_item,
            model_name,
            box,
            color,
        )
        future.add_done_callback(self._on_spawn_finished)

    def _spawn_item(
        self,
        model_name: str,
        box: BoxSpec,
        color: tuple[float, float, float],
    ) -> bool:
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
            return False

        success = (
            result.returncode == 0
            and "data: true" in result.stdout.lower()
        )
        if not success:
            self.get_logger().error(
                "Gazebo rejected separator demo item. "
                f"stdout={result.stdout.strip()} "
                f"stderr={result.stderr.strip()}"
            )
        return success

    def _on_spawn_finished(self, future: Future) -> None:
        try:
            future.result()
        finally:
            with self.state_lock:
                self.spawn_in_progress = False

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
