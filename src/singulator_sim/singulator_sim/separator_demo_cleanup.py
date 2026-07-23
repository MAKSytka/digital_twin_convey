#!/usr/bin/env python3
"""Remove completed items from the infeed-size-separator demonstration."""

from __future__ import annotations

import re
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node


BOX_NAME = re.compile(r"^box_separator_[A-Za-z0-9_]+$")
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


def parse_pose(block: str) -> tuple[str, float, float] | None:
    name = re.search(r'name:\s*"([^"]+)"', block)
    position = re.search(r"position\s*\{(.*?)\}", block, re.DOTALL)
    if name is None or position is None:
        return None

    def coordinate(axis: str) -> float:
        match = re.search(rf"\b{axis}:\s*({NUMBER})", position.group(1))
        return float(match.group(1)) if match else 0.0

    return name.group(1), coordinate("x"), coordinate("z")


class SeparatorDemoCleanup(Node):
    """Despawn separator-demo boxes after either output conveyor."""

    def __init__(self) -> None:
        super().__init__("separator_demo_cleanup")
        self.declare_parameter("world_name", "infeed_size_separator_demo")
        self.declare_parameter("exit_x_m", 1.82)
        self.declare_parameter("fallen_z_m", -0.80)
        self.declare_parameter("service_timeout_ms", 2000)
        self.world_name = str(self.get_parameter("world_name").value)
        self.exit_x = float(self.get_parameter("exit_x_m").value)
        self.fallen_z = float(self.get_parameter("fallen_z_m").value)
        self.timeout_ms = int(self.get_parameter("service_timeout_ms").value)
        self.pending: set[str] = set()
        self.deleted: set[str] = set()
        self.lock = threading.Lock()
        self.pool = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="separator_remove"
        )
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        self.monitor.start()
        self.get_logger().info(
            f"Removing separator items at x >= {self.exit_x:.2f} m "
            f"or z <= {self.fallen_z:.2f} m"
        )

    def _monitor(self) -> None:
        command = ["gz", "topic", "-e", "-t", f"/world/{self.world_name}/pose/info"]
        try:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, text=True, bufsize=1
            )
        except OSError as error:
            self.get_logger().error(f"Cannot subscribe to Gazebo poses: {error}")
            return
        if process.stdout is None:
            return
        for block in pose_blocks(process.stdout):
            parsed = parse_pose(block)
            if parsed is None:
                continue
            name, x, z = parsed
            if not BOX_NAME.fullmatch(name) or (x < self.exit_x and z > self.fallen_z):
                continue
            with self.lock:
                if name in self.pending or name in self.deleted:
                    continue
                self.pending.add(name)
            self.pool.submit(self._remove, name)

    def _remove(self, name: str) -> None:
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
            success = result.returncode == 0 and "data: true" in result.stdout.lower()
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f"Failed to remove {name}: {error}")
            success = False
        with self.lock:
            self.pending.discard(name)
            if success:
                self.deleted.add(name)
                self.get_logger().info(f"Despawned {name}")

    def destroy_node(self) -> bool:
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
