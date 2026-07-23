#!/usr/bin/env python3
"""Track the two separator branches, log statistics and remove completed items."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node


BOX_NAME = re.compile(
    r"^box_separator_[A-Za-z0-9_]+_exp_(upper|lower)_spot\d+$"
)
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def pose_blocks(stream):
    """Yield complete ``pose { ... }`` records from ``gz topic -e``."""
    collecting = False
    depth = 0
    lines: list[str] = []
    for line in stream:
        if not collecting:
            if line.strip() == "pose {":
                collecting, depth, lines = True, 1, [line]
            continue

        lines.append(line)
        depth += line.count("{") - line.count("}")
        if depth == 0:
            yield "".join(lines)
            collecting, lines = False, []


def parse_pose(block: str) -> tuple[str, float, float, float] | None:
    name = re.search(r'name:\s*"([^"]+)"', block)
    position = re.search(r"position\s*\{(.*?)\}", block, re.DOTALL)
    if name is None or position is None:
        return None

    def coordinate(axis: str) -> float:
        match = re.search(
            rf"\b{axis}:\s*({NUMBER})",
            position.group(1),
        )
        return float(match.group(1)) if match else 0.0

    return (
        name.group(1),
        coordinate("x"),
        coordinate("y"),
        coordinate("z"),
    )


class SeparatorDemoCleanup(Node):
    """Classify actual paths and despawn boxes after either 3 m output."""

    def __init__(self) -> None:
        super().__init__("separator_demo_cleanup")
        self.declare_parameter("world_name", "infeed_size_separator_demo")
        self.declare_parameter("route_decision_x_m", 0.85)
        self.declare_parameter("upper_exit_x_m", 3.45)
        self.declare_parameter("lower_exit_x_m", 3.52)
        self.declare_parameter("upper_route_min_z_m", -0.05)
        self.declare_parameter("lower_route_max_z_m", -0.08)
        self.declare_parameter("fallen_z_m", -0.85)
        self.declare_parameter("lateral_limit_m", 1.80)
        self.declare_parameter("maximum_lifetime_s", 30.0)
        self.declare_parameter("statistics_period_s", 2.0)
        self.declare_parameter("service_timeout_ms", 2000)

        self.world_name = str(self.get_parameter("world_name").value)
        self.route_x = float(
            self.get_parameter("route_decision_x_m").value
        )
        self.upper_exit_x = float(
            self.get_parameter("upper_exit_x_m").value
        )
        self.lower_exit_x = float(
            self.get_parameter("lower_exit_x_m").value
        )
        self.upper_min_z = float(
            self.get_parameter("upper_route_min_z_m").value
        )
        self.lower_max_z = float(
            self.get_parameter("lower_route_max_z_m").value
        )
        self.fallen_z = float(
            self.get_parameter("fallen_z_m").value
        )
        self.lateral_limit = float(
            self.get_parameter("lateral_limit_m").value
        )
        self.maximum_lifetime = float(
            self.get_parameter("maximum_lifetime_s").value
        )
        statistics_period = float(
            self.get_parameter("statistics_period_s").value
        )
        self.timeout_ms = int(
            self.get_parameter("service_timeout_ms").value
        )

        if self.upper_exit_x <= self.route_x:
            raise ValueError("upper_exit_x_m must be after route_decision_x_m")
        if self.lower_exit_x <= self.route_x:
            raise ValueError("lower_exit_x_m must be after route_decision_x_m")
        if self.lower_max_z >= self.upper_min_z:
            raise ValueError(
                "lower_route_max_z_m must be below upper_route_min_z_m"
            )
        if self.maximum_lifetime <= 0.0:
            raise ValueError("maximum_lifetime_s must be positive")
        if statistics_period <= 0.0:
            raise ValueError("statistics_period_s must be positive")

        self.pending: set[str] = set()
        self.deleted: set[str] = set()
        self.first_seen: dict[str, float] = {}
        self.actual_route: dict[str, str] = {}
        self.route_times_upper: list[float] = []
        self.route_times_lower: list[float] = []
        self.seen_count = 0
        self.actual_upper = 0
        self.actual_lower = 0
        self.mismatch_count = 0
        self.removed_count = 0
        self.remove_failures = 0

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.monitor_process: subprocess.Popen | None = None
        self.pool = ThreadPoolExecutor(
            max_workers=2,
            thread_name_prefix="separator_remove",
        )
        self.monitor = threading.Thread(
            target=self._monitor,
            daemon=True,
        )
        self.monitor.start()
        self.statistics_timer = self.create_timer(
            statistics_period,
            self._log_statistics,
        )
        self.get_logger().info(
            "Separator flow monitor: "
            f"route at x={self.route_x:.2f} m, "
            f"upper delete x={self.upper_exit_x:.2f} m, "
            f"lower delete x={self.lower_exit_x:.2f} m"
        )

    @staticmethod
    def _expected_route(name: str) -> str:
        match = BOX_NAME.fullmatch(name)
        if match is None:
            raise ValueError(f"Unexpected separator model name: {name}")
        return match.group(1)

    def _monitor(self) -> None:
        command = [
            "gz",
            "topic",
            "-e",
            "-t",
            f"/world/{self.world_name}/pose/info",
        ]
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self.monitor_process = process
        except OSError as error:
            self.get_logger().error(
                f"Cannot subscribe to Gazebo poses: {error}"
            )
            return

        if process.stdout is None:
            return

        for block in pose_blocks(process.stdout):
            if self.stop_event.is_set():
                break
            parsed = parse_pose(block)
            if parsed is None:
                continue
            name, x, y, z = parsed
            if BOX_NAME.fullmatch(name) is None:
                continue
            self._process_pose(name, x, y, z)

    def _process_pose(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
    ) -> None:
        now = time.monotonic()
        with self.lock:
            if name in self.deleted or name in self.pending:
                return
            if name not in self.first_seen:
                self.first_seen[name] = now
                self.seen_count += 1

            route = self.actual_route.get(name)
            if route is None and x >= self.route_x:
                if z >= self.upper_min_z:
                    route = "upper"
                elif z <= self.lower_max_z:
                    route = "lower"

                if route is not None:
                    self.actual_route[name] = route
                    transit = now - self.first_seen[name]
                    if route == "upper":
                        self.actual_upper += 1
                        self.route_times_upper.append(transit)
                    else:
                        self.actual_lower += 1
                        self.route_times_lower.append(transit)

                    expected = self._expected_route(name)
                    if expected != route:
                        self.mismatch_count += 1
                        self.get_logger().warning(
                            f"Route mismatch for {name}: "
                            f"expected={expected.upper()}, "
                            f"actual={route.upper()}, x={x:.3f}, z={z:.3f}"
                        )
                    else:
                        self.get_logger().info(
                            f"Route confirmed for {name}: "
                            f"{route.upper()}, transit={transit:.2f} s"
                        )

            age = now - self.first_seen[name]
            completed_upper = (
                x >= self.upper_exit_x
                and z >= self.upper_min_z
            )
            completed_lower = (
                x >= self.lower_exit_x
                and z <= self.lower_max_z
            )
            unsafe = (
                z <= self.fallen_z
                or abs(y) >= self.lateral_limit
                or age >= self.maximum_lifetime
            )
            if not (completed_upper or completed_lower or unsafe):
                return
            self.pending.add(name)

        reason = (
            "upper_exit"
            if completed_upper
            else "lower_exit"
            if completed_lower
            else "safety_cleanup"
        )
        self.pool.submit(self._remove, name, reason)

    def _remove(self, name: str, reason: str) -> None:
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/remove",
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self.timeout_ms),
            "--req",
            f'name: "{name}" type: MODEL',
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self.timeout_ms / 1000.0 + 2.0,
                check=False,
            )
            success = (
                result.returncode == 0
                and "data: true" in result.stdout.lower()
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f"Failed to remove {name}: {error}")
            success = False

        with self.lock:
            self.pending.discard(name)
            if success:
                self.deleted.add(name)
                self.removed_count += 1
                self.first_seen.pop(name, None)
                self.actual_route.pop(name, None)
                self.get_logger().info(
                    f"Despawned {name}: reason={reason}"
                )
            else:
                self.remove_failures += 1

    @staticmethod
    def _average(values: list[float]) -> float:
        return sum(values) / len(values) if values else 0.0

    def _log_statistics(self) -> None:
        with self.lock:
            active = len(self.first_seen)
            self.get_logger().info(
                "Separator statistics: "
                f"seen={self.seen_count}, "
                f"upper={self.actual_upper}, "
                f"lower={self.actual_lower}, "
                f"mismatches={self.mismatch_count}, "
                f"removed={self.removed_count}, "
                f"active={active}, "
                f"avg_upper={self._average(self.route_times_upper):.2f} s, "
                f"avg_lower={self._average(self.route_times_lower):.2f} s, "
                f"remove_failures={self.remove_failures}"
            )

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.monitor_process is not None:
            self.monitor_process.terminate()
        self.statistics_timer.cancel()
        self.pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeparatorDemoCleanup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
