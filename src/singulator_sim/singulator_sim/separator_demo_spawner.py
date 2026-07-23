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
        self.declare_parameter("spawn_x", -1.60)
        self.declare_parameter("belt_top_z", 0.08)
        self.declare_parameter("spawn_period_s", 1.60)
        self.declare_parameter("cycles", 3)
        self.declare_parameter("service_timeout_ms", 5000)
        self.declare_parameter("motion_assist_enabled", True)
        self.declare_parameter("transport_assist_acceleration_mps2", 0.95)
        self.declare_parameter("linear_velocity_decay", 1.50)
        self.declare_parameter("angular_velocity_decay", 2.00)

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
        self.motion_assist_enabled = bool(
            self.get_parameter("motion_assist_enabled").value
        )
        self.transport_assist_acceleration = float(
            self.get_parameter("transport_assist_acceleration_mps2").value
        )
        self.linear_velocity_decay = float(
            self.get_parameter("linear_velocity_decay").value
        )
        self.angular_velocity_decay = float(
            self.get_parameter("angular_velocity_decay").value
        )

        if self.spawn_period_s <= 0.0:
            raise ValueError("spawn_period_s must be positive")
        if self.cycles <= 0:
            raise ValueError("cycles must be positive")
        if self.transport_assist_acceleration < 0.0:
            raise ValueError(
                "transport_assist_acceleration_mps2 cannot be negative"
            )
        if self.linear_velocity_decay < 0.0:
            raise ValueError("linear_velocity_decay cannot be negative")
        if self.angular_velocity_decay < 0.0:
            raise ValueError("angular_velocity_decay cannot be negative")

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

        assist_state = "enabled" if self.motion_assist_enabled else "disabled"
        estimated_speed = (
            self.transport_assist_acceleration / self.linear_velocity_decay
            if self.linear_velocity_decay > 0.0
            else float("inf")
        )
        self.get_logger().info(
            "Separator demo sequence: "
            f"{self.maximum_items} items, "
            "green/blue -> upper path, orange/red -> lower path"
        )
        self.get_logger().info(
            "Transport motion assist "
            f"{assist_state}: acceleration="
            f"{self.transport_assist_acceleration:.2f} m/s^2, "
            f"linear_decay={self.linear_velocity_decay:.2f} 1/s, "
            f"estimated_free_speed={estimated_speed:.2f} m/s"
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

    def _with_velocity_decay(self, sdf: str) -> str:
        marker = '    <link name="base_link">\n'
        if marker not in sdf:
            raise RuntimeError("Box SDF does not contain base_link")

        replacement = (
            marker
            + "\n"
            + "      <velocity_decay>\n"
            + f"        <linear>{self.linear_velocity_decay:.6f}</linear>\n"
            + f"        <angular>{self.angular_velocity_decay:.6f}</angular>\n"
            + "      </velocity_decay>\n"
        )
        return sdf.replace(marker, replacement, 1)

    def _spawn_item(
        self,
        model_name: str,
        box: BoxSpec,
        color: tuple[float, float, float],
    ) -> bool:
        sdf = box.to_sdf(model_name=model_name, color=color)
        sdf = self._with_velocity_decay(sdf)
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
            return False

        if not self.motion_assist_enabled:
            return True

        return self._apply_transport_assist(model_name, box.mass)

    def _apply_transport_assist(self, model_name: str, mass: float) -> bool:
        force_x = mass * self.transport_assist_acceleration
        topic = f"/world/{self.world_name}/wrench/persistent"
        payload = (
            f'entity: {{name: "{model_name}", type: MODEL}}, '
            f"wrench: {{force: {{x: {force_x:.9f}}}}}"
        )
        command = [
            "gz",
            "topic",
            "-t",
            topic,
            "-m",
            "gz.msgs.EntityWrench",
            "-p",
            payload,
        ]

        for attempt in range(1, 4):
            try:
                result = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=3.0,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                self.get_logger().warning(
                    f"Motion assist attempt {attempt} failed for "
                    f"{model_name}: {error}"
                )
            else:
                if result.returncode == 0:
                    self.get_logger().info(
                        f"Motion assist applied to {model_name}: "
                        f"force_x={force_x:.4f} N"
                    )
                    return True
                self.get_logger().warning(
                    f"Motion assist attempt {attempt} returned "
                    f"{result.returncode}: {result.stderr.strip()}"
                )
            time.sleep(0.10)

        self.get_logger().error(
            f"Could not apply transport motion to {model_name}"
        )
        return False

    def _on_spawn_finished(self, future: Future) -> None:
        try:
            success = bool(future.result())
            if not success:
                self.get_logger().error(
                    "Separator item was created without confirmed motion"
                )
        except Exception as error:  # noqa: BLE001 - callback must release lock
            self.get_logger().error(f"Separator spawn task failed: {error}")
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
