"""Deterministic stage-2 KTY flow cycle.

This node intentionally uses wall time and Gazebo UserCommands services.  It
implements one independently testable cycle:

create empty KTY -> move to active position -> spawn fixed products on chute ->
wait until they are inside -> move loaded KTY and products as one pose group ->
remove all dynamic models.

The controller does not depend on /clock, custom KTY messages, perception,
safety or metrics.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import math
import re
import subprocess
import threading
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .model_factory import make_kty_sdf


class RestartRequested(RuntimeError):
    """Raised inside the worker when a restart was requested."""


@dataclass(frozen=True, slots=True)
class Pose:
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def translated_x(self, distance: float) -> "Pose":
        return Pose(
            x=self.x + distance,
            y=self.y,
            z=self.z,
            qx=self.qx,
            qy=self.qy,
            qz=self.qz,
            qw=self.qw,
        )


@dataclass(frozen=True, slots=True)
class ProductProfile:
    size_x: float
    size_y: float
    size_z: float
    mass: float
    yaw: float
    spawn_y: float
    color: tuple[float, float, float]


PRODUCT_PROFILES = (
    ProductProfile(0.120, 0.080, 0.060, 0.25, 0.12, -0.08, (0.80, 0.42, 0.15)),
    ProductProfile(0.150, 0.090, 0.070, 0.35, -0.18, 0.07, (0.22, 0.48, 0.82)),
    ProductProfile(0.100, 0.100, 0.100, 0.40, 0.32, 0.00, (0.76, 0.64, 0.22)),
    ProductProfile(0.180, 0.110, 0.050, 0.30, -0.27, -0.04, (0.48, 0.24, 0.16)),
    ProductProfile(0.130, 0.070, 0.090, 0.32, 0.20, 0.09, (0.36, 0.66, 0.34)),
    ProductProfile(0.160, 0.120, 0.060, 0.45, -0.08, -0.10, (0.72, 0.30, 0.58)),
)


def _box_inertia(
    mass: float,
    size_x: float,
    size_y: float,
    size_z: float,
) -> tuple[float, float, float]:
    return (
        mass * (size_y**2 + size_z**2) / 12.0,
        mass * (size_x**2 + size_z**2) / 12.0,
        mass * (size_x**2 + size_y**2) / 12.0,
    )


def make_flow_product_sdf(name: str, profile: ProductProfile) -> str:
    """Build a low-bounce product that reliably slides down the stage-2 chute."""
    ixx, iyy, izz = _box_inertia(
        profile.mass,
        profile.size_x,
        profile.size_y,
        profile.size_z,
    )
    red, green, blue = profile.color
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <allow_auto_disable>false</allow_auto_disable>
    <link name="body">
      <inertial>
        <mass>{profile.mass:.9f}</mass>
        <inertia>
          <ixx>{ixx:.12f}</ixx>
          <iyy>{iyy:.12f}</iyy>
          <izz>{izz:.12f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <velocity_decay><linear>0.01</linear><angular>0.04</angular></velocity_decay>
      <collision name="collision">
        <geometry>
          <box><size>{profile.size_x:.9f} {profile.size_y:.9f} {profile.size_z:.9f}</size></box>
        </geometry>
        <surface>
          <friction><ode><mu>0.35</mu><mu2>0.35</mu2></ode></friction>
          <bounce><restitution_coefficient>0.02</restitution_coefficient><threshold>0.2</threshold></bounce>
          <contact><ode><kp>4000000</kp><kd>100</kd><max_vel>0.08</max_vel><min_depth>0.0005</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{profile.size_x:.9f} {profile.size_y:.9f} {profile.size_z:.9f}</size></box>
        </geometry>
        <material>
          <ambient>{red:.4f} {green:.4f} {blue:.4f} 1</ambient>
          <diffuse>{red:.4f} {green:.4f} {blue:.4f} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class KtyFlowCycle(Node):
    """One deterministic visual KTY load-and-eject cycle."""

    def __init__(self) -> None:
        super().__init__("kty_flow_cycle")

        defaults = {
            "world_name": "kty_flow",
            "product_count": 6,
            "approach_duration_s": 3.0,
            "empty_settle_s": 0.8,
            "product_spawn_interval_s": 0.9,
            "loaded_settle_timeout_s": 10.0,
            "outfeed_duration_s": 3.0,
            "outfeed_hold_s": 1.5,
            "cycle_pause_s": 3.0,
            "pose_update_hz": 5.0,
            "auto_repeat": False,
            "service_timeout_ms": 5000,
            "kty_spawn_x_m": -1.25,
            "kty_active_x_m": 0.0,
            "kty_outfeed_x_m": 1.35,
            "support_top_z_m": 0.50,
            "product_spawn_x_m": -1.08,
            "product_spawn_z_m": 1.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.world_name = str(self.get_parameter("world_name").value)
        self.product_count = int(self.get_parameter("product_count").value)
        self.approach_duration = float(
            self.get_parameter("approach_duration_s").value
        )
        self.empty_settle = float(self.get_parameter("empty_settle_s").value)
        self.spawn_interval = float(
            self.get_parameter("product_spawn_interval_s").value
        )
        self.loaded_settle_timeout = float(
            self.get_parameter("loaded_settle_timeout_s").value
        )
        self.outfeed_duration = float(
            self.get_parameter("outfeed_duration_s").value
        )
        self.outfeed_hold = float(self.get_parameter("outfeed_hold_s").value)
        self.cycle_pause = float(self.get_parameter("cycle_pause_s").value)
        self.pose_update_hz = float(self.get_parameter("pose_update_hz").value)
        self.auto_repeat = bool(self.get_parameter("auto_repeat").value)
        self.service_timeout_ms = int(
            self.get_parameter("service_timeout_ms").value
        )
        self.kty_spawn_x = float(self.get_parameter("kty_spawn_x_m").value)
        self.kty_active_x = float(self.get_parameter("kty_active_x_m").value)
        self.kty_outfeed_x = float(
            self.get_parameter("kty_outfeed_x_m").value
        )
        self.support_top_z = float(
            self.get_parameter("support_top_z_m").value
        )
        self.product_spawn_x = float(
            self.get_parameter("product_spawn_x_m").value
        )
        self.product_spawn_z = float(
            self.get_parameter("product_spawn_z_m").value
        )

        if not 1 <= self.product_count <= len(PRODUCT_PROFILES):
            raise ValueError(
                f"product_count must be in 1..{len(PRODUCT_PROFILES)}"
            )
        if self.pose_update_hz <= 0.0:
            raise ValueError("pose_update_hz must be positive")

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String,
            "/kty/flow/state",
            state_qos,
        )
        self.heartbeat_pub = self.create_publisher(
            String,
            "/kty/flow/heartbeat",
            10,
        )
        self.create_service(Trigger, "/kty/flow/restart", self._on_restart)

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._restart_event = threading.Event()
        self._known_models: set[str] = set()
        self._cycle_id = 0
        self._completed_cycles = 0
        self._state_name = "STARTING"
        self._state_detail = "node created"
        self._spawned_products = 0
        self._inside_products = 0
        self._removed_models = 0
        self._started_monotonic = time.monotonic()

        self._service_pool = ThreadPoolExecutor(
            max_workers=8,
            thread_name_prefix="kty_flow_service",
        )
        self._heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self._worker = threading.Thread(
            target=self._worker_main,
            name="kty_flow_worker",
            daemon=True,
        )
        self._worker.start()

    @property
    def _create_service(self) -> str:
        return f"/world/{self.world_name}/create"

    @property
    def _set_pose_service(self) -> str:
        return f"/world/{self.world_name}/set_pose"

    @property
    def _remove_service(self) -> str:
        return f"/world/{self.world_name}/remove"

    @property
    def _pose_topic(self) -> str:
        return f"/world/{self.world_name}/pose/info"

    def _on_restart(self, request, response):
        del request
        self._restart_event.set()
        response.success = True
        response.message = "KTY flow cycle restart requested"
        return response

    def _publish_heartbeat(self) -> None:
        with self._lock:
            payload = {
                "status": "alive",
                "wall_uptime_s": round(
                    time.monotonic() - self._started_monotonic,
                    3,
                ),
                "cycle_id": self._cycle_id,
                "state": self._state_name,
                "completed_cycles": self._completed_cycles,
            }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.heartbeat_pub.publish(message)

    def _publish_state(self, state: str, detail: str, **extra) -> None:
        with self._lock:
            self._state_name = state
            self._state_detail = detail
            payload = {
                "cycle_id": self._cycle_id,
                "state": state,
                "detail": detail,
                "spawned_products": self._spawned_products,
                "inside_products": self._inside_products,
                "removed_models": self._removed_models,
                "completed_cycles": self._completed_cycles,
                "expected_product_count": self.product_count,
                "kty_name": "kty_flow_container",
            }
            payload.update(extra)
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.state_pub.publish(message)
        self.get_logger().info(f"{state}: {detail}")

    def _worker_main(self) -> None:
        while not self._stop_event.is_set():
            self._restart_event.clear()
            try:
                self._run_cycle()
            except RestartRequested:
                self._publish_state("RESTARTING", "restart requested")
                self._cleanup_known_models(strict=False)
                continue
            except Exception as error:  # pragma: no cover - runtime guard
                self.get_logger().error(f"KTY flow cycle failed: {error!r}")
                self._publish_state("ERROR", str(error))
                self._cleanup_known_models(strict=False)
                while (
                    not self._stop_event.is_set()
                    and not self._restart_event.wait(0.25)
                ):
                    pass
                continue

            if self.auto_repeat:
                self._interruptible_sleep(self.cycle_pause)
                continue

            while (
                not self._stop_event.is_set()
                and not self._restart_event.wait(0.25)
            ):
                pass

    def _run_cycle(self) -> None:
        self._cycle_id += 1
        self._spawned_products = 0
        self._inside_products = 0
        self._removed_models = 0

        self._publish_state("WAIT_SERVICES", "waiting for Gazebo UserCommands")
        self._wait_for_services(timeout_s=20.0)
        self._cleanup_stale_names()

        kty_name = "kty_flow_container"
        product_names = [
            f"kty_flow_product_{index:02d}"
            for index in range(1, self.product_count + 1)
        ]

        kty_start = Pose(
            x=self.kty_spawn_x,
            y=0.0,
            z=self.support_top_z,
        )
        self._publish_state("SPAWN_KTY", "creating empty dynamic KTY")
        if not self._create_model(
            kty_name,
            make_kty_sdf(kty_name),
            kty_start,
        ):
            raise RuntimeError("Gazebo rejected KTY create request")
        self._known_models.add(kty_name)

        self._publish_state(
            "APPROACH",
            "moving empty KTY from infeed to active position",
        )
        self._move_pose_group(
            {kty_name: kty_start},
            distance_x=self.kty_active_x - self.kty_spawn_x,
            duration_s=self.approach_duration,
        )
        self._interruptible_sleep(self.empty_settle)

        self._publish_state("LOAD", "spawning products on the chute")
        for index, (name, profile) in enumerate(
            zip(product_names, PRODUCT_PROFILES, strict=False),
            start=1,
        ):
            yaw_half = 0.5 * profile.yaw
            pose = Pose(
                x=self.product_spawn_x,
                y=profile.spawn_y,
                z=self.product_spawn_z + 0.012 * ((index - 1) % 2),
                qz=math.sin(yaw_half),
                qw=math.cos(yaw_half),
            )
            if not self._create_model(
                name,
                make_flow_product_sdf(name, profile),
                pose,
            ):
                raise RuntimeError(f"Gazebo rejected create request for {name}")
            self._known_models.add(name)
            self._spawned_products = index
            self._publish_state(
                "LOAD",
                f"spawned product {index}/{self.product_count}",
                active_model=name,
            )
            self._interruptible_sleep(self.spawn_interval)

        self._publish_state(
            "SETTLE",
            "waiting for all products to slide and fall inside the KTY",
        )
        captured_poses = self._wait_until_loaded(
            kty_name,
            product_names,
            timeout_s=self.loaded_settle_timeout,
        )

        self._publish_state(
            "OUTFEED",
            "moving loaded KTY and captured products to outfeed",
            captured_models=len(captured_poses),
        )
        kty_pose = captured_poses[kty_name]
        self._move_pose_group(
            captured_poses,
            distance_x=self.kty_outfeed_x - kty_pose.x,
            duration_s=self.outfeed_duration,
        )

        self._publish_state(
            "OUTFEED_HOLD",
            "loaded KTY reached outfeed; holding for visual inspection",
        )
        self._interruptible_sleep(self.outfeed_hold)

        self._publish_state("DESPAWN", "removing products and KTY")
        remove_names = [*product_names, kty_name]
        removed = self._remove_models(remove_names, strict=True)
        self._removed_models = removed
        self._known_models.difference_update(remove_names)

        self._completed_cycles += 1
        self._publish_state(
            "COMPLETE",
            "cycle completed and all dynamic models removed",
            spawned_total=self._spawned_products,
            inside_total=self._inside_products,
            removed_total=self._removed_models,
        )

    def _wait_for_services(self, timeout_s: float) -> None:
        required = {
            self._create_service,
            self._set_pose_service,
            self._remove_service,
        }
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_interrupt()
            result = subprocess.run(
                ["gz", "service", "-l"],
                capture_output=True,
                text=True,
                timeout=5.0,
                check=False,
            )
            available = {
                line.strip()
                for line in result.stdout.splitlines()
                if line.strip()
            }
            missing = required - available
            if result.returncode == 0 and not missing:
                return
            time.sleep(0.25)
        raise RuntimeError(f"Gazebo services unavailable: {sorted(required)}")

    def _cleanup_stale_names(self) -> None:
        stale_names = ["kty_flow_container"]
        stale_names.extend(
            f"kty_flow_product_{index:02d}"
            for index in range(1, len(PRODUCT_PROFILES) + 1)
        )
        self._remove_models(stale_names, strict=False)
        self._known_models.clear()

    def _cleanup_known_models(self, strict: bool) -> None:
        names = sorted(self._known_models)
        if names:
            self._remove_models(names, strict=strict)
        self._known_models.clear()

    def _create_model(self, name: str, sdf: str, pose: Pose) -> bool:
        request = "\n".join(
            (
                f"sdf: {json.dumps(sdf)}",
                f'name: "{name}"',
                "allow_renaming: false",
                "pose {",
                (
                    "  position { "
                    f"x: {pose.x:.9f} y: {pose.y:.9f} z: {pose.z:.9f}"
                    " }"
                ),
                (
                    "  orientation { "
                    f"x: {pose.qx:.9f} y: {pose.qy:.9f} "
                    f"z: {pose.qz:.9f} w: {pose.qw:.9f}"
                    " }"
                ),
                "}",
            )
        )
        return self._call_boolean_service(
            self._create_service,
            "gz.msgs.EntityFactory",
            request,
        )

    def _set_pose(self, name: str, pose: Pose) -> bool:
        request = "\n".join(
            (
                f'name: "{name}"',
                (
                    "position { "
                    f"x: {pose.x:.9f} y: {pose.y:.9f} z: {pose.z:.9f}"
                    " }"
                ),
                (
                    "orientation { "
                    f"x: {pose.qx:.9f} y: {pose.qy:.9f} "
                    f"z: {pose.qz:.9f} w: {pose.qw:.9f}"
                    " }"
                ),
            )
        )
        return self._call_boolean_service(
            self._set_pose_service,
            "gz.msgs.Pose",
            request,
        )

    def _remove_model(self, name: str) -> bool:
        request = f'name: "{name}" type: MODEL'
        return self._call_boolean_service(
            self._remove_service,
            "gz.msgs.Entity",
            request,
        )

    def _call_boolean_service(
        self,
        service: str,
        request_type: str,
        request: str,
    ) -> bool:
        command = [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            request_type,
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
            self.get_logger().error(f"Service {service} failed: {error}")
            return False

        success = (
            result.returncode == 0
            and "data: true" in result.stdout.lower()
        )
        if not success:
            self.get_logger().warning(
                f"Service {service} rejected request for output "
                f"{result.stdout.strip()} {result.stderr.strip()}"
            )
        return success

    def _move_pose_group(
        self,
        base_poses: dict[str, Pose],
        distance_x: float,
        duration_s: float,
    ) -> None:
        if duration_s <= 0.0:
            raise ValueError("motion duration must be positive")
        period = 1.0 / self.pose_update_hz
        steps = max(1, math.ceil(duration_s * self.pose_update_hz))
        started = time.monotonic()

        for step in range(1, steps + 1):
            self._check_interrupt()
            ratio = step / steps
            eased = ratio * ratio * (3.0 - 2.0 * ratio)
            targets = {
                name: pose.translated_x(distance_x * eased)
                for name, pose in base_poses.items()
            }
            futures = {
                name: self._service_pool.submit(self._set_pose, name, pose)
                for name, pose in targets.items()
            }
            failures = [
                name
                for name, future in futures.items()
                if not future.result(
                    timeout=self.service_timeout_ms / 1000.0 + 3.0
                )
            ]
            if failures:
                raise RuntimeError(
                    f"set_pose failed for models: {', '.join(failures)}"
                )

            target_time = started + step * duration_s / steps
            remaining = target_time - time.monotonic()
            if remaining > 0.0:
                self._interruptible_sleep(min(period, remaining))

    def _wait_until_loaded(
        self,
        kty_name: str,
        product_names: list[str],
        timeout_s: float,
    ) -> dict[str, Pose]:
        required = {kty_name, *product_names}
        deadline = time.monotonic() + timeout_s
        last_found: dict[str, Pose] = {}

        while time.monotonic() < deadline:
            self._check_interrupt()
            poses = self._read_world_poses()
            last_found = {name: poses[name] for name in required if name in poses}
            if kty_name not in last_found:
                self._interruptible_sleep(0.25)
                continue

            kty = last_found[kty_name]
            inside = 0
            for name in product_names:
                pose = last_found.get(name)
                if pose is None:
                    continue
                if (
                    abs(pose.x - kty.x) <= 0.34
                    and abs(pose.y - kty.y) <= 0.23
                    and self.support_top_z - 0.03 <= pose.z <= 1.02
                ):
                    inside += 1

            self._inside_products = inside
            self._publish_state(
                "SETTLE",
                f"{inside}/{self.product_count} products inside KTY",
                observed_models=len(last_found),
            )
            if inside == self.product_count and len(last_found) == len(required):
                return last_found
            self._interruptible_sleep(0.35)

        missing = sorted(required - set(last_found))
        raise RuntimeError(
            "products did not settle inside KTY before timeout; "
            f"inside={self._inside_products}/{self.product_count}, "
            f"missing={missing}"
        )

    def _read_world_poses(self) -> dict[str, Pose]:
        command = [
            "gz",
            "topic",
            "-e",
            "-t",
            self._pose_topic,
            "-n",
            "1",
        ]
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=6.0,
            check=False,
        )
        if result.returncode != 0:
            self.get_logger().warning(
                f"Pose snapshot failed: {result.stderr.strip()}"
            )
            return {}
        return self._parse_pose_vector(result.stdout)

    @staticmethod
    def _parse_pose_vector(text: str) -> dict[str, Pose]:
        blocks: list[str] = []
        current: list[str] = []
        depth = 0

        for line in text.splitlines():
            stripped = line.strip()
            if not current:
                if stripped == "pose {":
                    current = [line]
                    depth = line.count("{") - line.count("}")
                continue

            current.append(line)
            depth += line.count("{") - line.count("}")
            if depth == 0:
                blocks.append("\n".join(current))
                current = []

        result: dict[str, Pose] = {}
        for block in blocks:
            name_match = re.search(r'name:\s*"([^"]+)"', block)
            position_match = re.search(
                r"position\s*\{([^}]*)\}",
                block,
                flags=re.DOTALL,
            )
            if name_match is None or position_match is None:
                continue

            position = KtyFlowCycle._parse_numeric_fields(
                position_match.group(1)
            )
            orientation_match = re.search(
                r"orientation\s*\{([^}]*)\}",
                block,
                flags=re.DOTALL,
            )
            orientation = (
                KtyFlowCycle._parse_numeric_fields(
                    orientation_match.group(1)
                )
                if orientation_match is not None
                else {}
            )
            result[name_match.group(1)] = Pose(
                x=position.get("x", 0.0),
                y=position.get("y", 0.0),
                z=position.get("z", 0.0),
                qx=orientation.get("x", 0.0),
                qy=orientation.get("y", 0.0),
                qz=orientation.get("z", 0.0),
                qw=orientation.get("w", 1.0),
            )
        return result

    @staticmethod
    def _parse_numeric_fields(text: str) -> dict[str, float]:
        fields: dict[str, float] = {}
        pattern = re.compile(
            r"\b([xyzw])\s*:\s*"
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
        )
        for key, value in pattern.findall(text):
            fields[key] = float(value)
        return fields

    def _remove_models(self, names: Iterable[str], strict: bool) -> int:
        name_list = list(names)
        futures = {
            name: self._service_pool.submit(self._remove_model, name)
            for name in name_list
        }
        removed = 0
        failures: list[str] = []
        for name, future in futures.items():
            success = future.result(
                timeout=self.service_timeout_ms / 1000.0 + 3.0
            )
            if success:
                removed += 1
            elif strict:
                failures.append(name)
        if failures:
            raise RuntimeError(
                f"remove failed for models: {', '.join(failures)}"
            )
        return removed

    def _check_interrupt(self) -> None:
        if self._stop_event.is_set():
            raise RestartRequested()
        if self._restart_event.is_set():
            raise RestartRequested()

    def _interruptible_sleep(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self._check_interrupt()
            remaining = deadline - time.monotonic()
            time.sleep(min(0.05, max(0.0, remaining)))

    def close(self) -> None:
        self._stop_event.set()
        self._restart_event.set()
        self._worker.join(timeout=3.0)
        self._cleanup_known_models(strict=False)
        self._service_pool.shutdown(wait=False, cancel_futures=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyFlowCycle()
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
