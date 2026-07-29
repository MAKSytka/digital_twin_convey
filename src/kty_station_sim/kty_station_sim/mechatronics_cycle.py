"""Physical stage-4 KTY mechatronics cycle.

Continuous motion is produced inside Gazebo by joint controllers: an infeed
pusher, powered roller groups, a retractable locator, side clamps, a hinged
chute gate and a Z-axis vibration deck. Gazebo services are used only to create
and remove models; moving KTY containers are never teleported with set_pose.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from .flow_cycle import ProductProfile, make_flow_product_sdf
from .model_factory import make_kty_sdf


@dataclass(frozen=True, slots=True)
class Pose:
    x: float
    y: float
    z: float


MECH_PRODUCT_PROFILES = (
    ProductProfile(0.240, 0.150, 0.110, 0.85, 0.08, -0.10, (0.80, 0.42, 0.15)),
    ProductProfile(0.210, 0.140, 0.130, 0.92, -0.15, 0.09, (0.22, 0.48, 0.82)),
    ProductProfile(0.180, 0.160, 0.100, 0.72, 0.24, -0.02, (0.76, 0.64, 0.22)),
    ProductProfile(0.260, 0.120, 0.090, 0.75, -0.22, 0.04, (0.48, 0.24, 0.16)),
    ProductProfile(0.200, 0.130, 0.120, 0.82, 0.18, -0.08, (0.36, 0.66, 0.34)),
    ProductProfile(0.230, 0.170, 0.080, 0.78, -0.10, 0.10, (0.72, 0.30, 0.58)),
    ProductProfile(0.190, 0.110, 0.140, 0.88, 0.28, 0.00, (0.30, 0.64, 0.72)),
    ProductProfile(0.270, 0.145, 0.095, 0.96, -0.20, -0.06, (0.82, 0.50, 0.22)),
)


class RestartRequested(RuntimeError):
    pass


class KtyMechatronicsCycle(Node):
    """Continuous two-KTY physical changeover cycle."""

    def __init__(self) -> None:
        super().__init__("kty_mechatronics_cycle")
        defaults = {
            "world_name": "kty_mechatronics",
            "auto_repeat": True,
            "product_spawn_interval_s": 0.65,
            "fill_ratio_threshold": 0.70,
            "max_height_threshold_m": 0.280,
            "fill_persistence_s": 0.50,
            "weak_vibration_frequency_hz": 8.0,
            "weak_vibration_amplitude_m": 0.0005,
            "strong_vibration_frequency_hz": 18.0,
            "strong_vibration_amplitude_m": 0.0030,
            "strong_vibration_duration_s": 8.0,
            "strong_vibration_ramp_s": 1.0,
            "roller_linear_speed_mps": 0.28,
            "roller_radius_m": 0.035,
            "slow_roller_linear_speed_mps": 0.10,
            "active_target_x_m": 0.032,
            "active_position_tolerance_m": 0.005,
            "ready_velocity_tolerance_mps": 0.020,
            "ready_persistence_s": 0.30,
            "queue_spawn_x_m": -1.55,
            "kty_bottom_z_m": 0.50,
            "pusher_extended_m": 0.90,
            "clamp_closed_m": 0.067,
            "gate_open_rad": 1.20,
            "locator_up_m": 0.225,
            "service_timeout_ms": 5000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.world_name = str(self.get_parameter("world_name").value)
        self.auto_repeat = bool(self.get_parameter("auto_repeat").value)
        self.spawn_interval = float(
            self.get_parameter("product_spawn_interval_s").value
        )
        self.fill_ratio_threshold = float(
            self.get_parameter("fill_ratio_threshold").value
        )
        self.height_threshold = float(
            self.get_parameter("max_height_threshold_m").value
        )
        self.fill_persistence = float(
            self.get_parameter("fill_persistence_s").value
        )
        self.weak_frequency = float(
            self.get_parameter("weak_vibration_frequency_hz").value
        )
        self.weak_amplitude = float(
            self.get_parameter("weak_vibration_amplitude_m").value
        )
        self.strong_frequency = float(
            self.get_parameter("strong_vibration_frequency_hz").value
        )
        self.strong_amplitude = float(
            self.get_parameter("strong_vibration_amplitude_m").value
        )
        self.strong_duration = float(
            self.get_parameter("strong_vibration_duration_s").value
        )
        self.strong_ramp = float(
            self.get_parameter("strong_vibration_ramp_s").value
        )
        linear_speed = float(
            self.get_parameter("roller_linear_speed_mps").value
        )
        slow_linear_speed = float(
            self.get_parameter("slow_roller_linear_speed_mps").value
        )
        roller_radius = float(self.get_parameter("roller_radius_m").value)
        self.roller_speed = linear_speed / roller_radius
        self.slow_roller_speed = slow_linear_speed / roller_radius
        self.active_target_x = float(
            self.get_parameter("active_target_x_m").value
        )
        self.position_tolerance = float(
            self.get_parameter("active_position_tolerance_m").value
        )
        self.velocity_tolerance = float(
            self.get_parameter("ready_velocity_tolerance_mps").value
        )
        self.ready_persistence = float(
            self.get_parameter("ready_persistence_s").value
        )
        self.queue_spawn_x = float(
            self.get_parameter("queue_spawn_x_m").value
        )
        self.kty_bottom_z = float(self.get_parameter("kty_bottom_z_m").value)
        self.pusher_extended = float(
            self.get_parameter("pusher_extended_m").value
        )
        self.clamp_closed = float(
            self.get_parameter("clamp_closed_m").value
        )
        self.gate_open = float(self.get_parameter("gate_open_rad").value)
        self.locator_up = float(
            self.get_parameter("locator_up_m").value
        )
        self.service_timeout_ms = int(
            self.get_parameter("service_timeout_ms").value
        )

        if roller_radius <= 0.0:
            raise ValueError("roller_radius_m must be positive")
        if not 0.0 < self.fill_ratio_threshold <= 1.0:
            raise ValueError("fill_ratio_threshold must be in (0, 1]")

        self.command_publishers = {
            "infeed": self.create_publisher(
                Float64, "/kty/mech/infeed_rollers/cmd_vel", 10
            ),
            "active": self.create_publisher(
                Float64, "/kty/mech/active_rollers/cmd_vel", 10
            ),
            "outfeed": self.create_publisher(
                Float64, "/kty/mech/outfeed_rollers/cmd_vel", 10
            ),
            "pusher": self.create_publisher(
                Float64, "/kty/mech/pusher/cmd_pos", 10
            ),
            "clamps": self.create_publisher(
                Float64, "/kty/mech/clamps/cmd_pos", 10
            ),
            "gate": self.create_publisher(
                Float64, "/kty/mech/gate/cmd_pos", 10
            ),
            "vibration": self.create_publisher(
                Float64, "/kty/mech/vibration/cmd_pos", 10
            ),
            "locator": self.create_publisher(
                Float64, "/kty/mech/locator_stop/cmd_pos", 10
            ),
        }

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, "/kty/flow/state", state_qos
        )
        self.heartbeat_pub = self.create_publisher(
            String, "/kty/mech/heartbeat", 10
        )
        self.create_subscription(
            String, "/kty/fill/state", self._on_fill_state, 10
        )
        self.create_service(Trigger, "/kty/mech/restart", self._on_restart)

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._restart_event = threading.Event()
        self._state_name = "STARTING"
        self._state_detail = "node created"
        self._cycle_id = 0
        self._product_serial = 0
        self._active_kty = ""
        self._queue_kty = ""
        self._known_models: set[str] = set()
        self._active_product_names: set[str] = set()
        self._latest_fill = {
            "camera_ok": False,
            "fill_ratio": 0.0,
            "maximum_height_m": 0.0,
            "valid_depth_fraction": 0.0,
        }

        self._commands = {
            "infeed": 0.0,
            "active": 0.0,
            "outfeed": 0.0,
            "pusher": 0.0,
            "clamps": 0.0,
            "gate": 0.0,
            "locator": 0.0,
        }
        self._vibration_mode = "off"
        self._vibration_started = time.monotonic()

        self._command_timer = self.create_timer(0.01, self._publish_commands)
        self._state_timer = self.create_timer(0.5, self._publish_periodic_state)
        self._heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self._worker = threading.Thread(
            target=self._worker_main,
            name="kty_mechatronics_worker",
            daemon=True,
        )
        self._worker.start()

    @property
    def _create_service(self) -> str:
        return f"/world/{self.world_name}/create"

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
        response.message = "KTY mechatronics restart requested"
        return response

    def _on_fill_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._latest_fill = payload

    def _set_commands(self, **updates: float) -> None:
        with self._lock:
            for key, value in updates.items():
                if key not in self._commands:
                    raise KeyError(key)
                self._commands[key] = float(value)

    def _set_vibration(self, mode: str) -> None:
        if mode not in {"off", "weak", "strong"}:
            raise ValueError(mode)
        with self._lock:
            self._vibration_mode = mode
            self._vibration_started = time.monotonic()

    def _publish_commands(self) -> None:
        with self._lock:
            commands = dict(self._commands)
            mode = self._vibration_mode
            started = self._vibration_started

        now = time.monotonic()
        elapsed = now - started
        vibration = 0.0
        if mode == "weak":
            vibration = self.weak_amplitude * math.sin(
                2.0 * math.pi * self.weak_frequency * elapsed
            )
        elif mode == "strong":
            ramp = max(0.05, self.strong_ramp)
            up = min(1.0, elapsed / ramp)
            down = min(1.0, max(0.0, self.strong_duration - elapsed) / ramp)
            envelope = min(up, down)
            vibration = (
                self.strong_amplitude
                * envelope
                * math.sin(2.0 * math.pi * self.strong_frequency * elapsed)
            )

        for key, value in commands.items():
            message = Float64()
            message.data = value
            self.command_publishers[key].publish(message)
        vibration_message = Float64()
        vibration_message.data = vibration
        self.command_publishers["vibration"].publish(vibration_message)

    def _state_payload(self) -> dict:
        with self._lock:
            fill = dict(self._latest_fill)
            return {
                "cycle_id": self._cycle_id,
                "state": self._state_name,
                "detail": self._state_detail,
                "active_kty": self._active_kty,
                "queue_kty": self._queue_kty,
                "known_models": len(self._known_models),
                "active_products": len(self._active_product_names),
                "gate_open": self._commands["gate"] > 0.5 * self.gate_open,
                "clamps_closed": self._commands["clamps"] > 0.5 * self.clamp_closed,
                "locator_up": self._commands["locator"] > 0.5 * self.locator_up,
                "vibration_mode": self._vibration_mode,
                "estimated_fill_ratio": float(fill.get("fill_ratio", 0.0)),
                "maximum_height_m": float(fill.get("maximum_height_m", 0.0)),
                "camera_ok": bool(fill.get("camera_ok", False)),
            }

    def _publish_periodic_state(self) -> None:
        payload = self._state_payload()
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.state_pub.publish(message)

    def _publish_heartbeat(self) -> None:
        payload = self._state_payload()
        payload["status"] = "alive"
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.heartbeat_pub.publish(message)

    def _transition(self, state: str, detail: str) -> None:
        with self._lock:
            self._state_name = state
            self._state_detail = detail
        self._publish_periodic_state()
        self.get_logger().info(f"{state}: {detail}")

    def _worker_main(self) -> None:
        while not self._stop_event.is_set():
            self._restart_event.clear()
            try:
                self._run()
            except RestartRequested:
                self._transition("RESTARTING", "restart requested")
                self._safe_mechanics()
                self._cleanup_known_models()
                continue
            except Exception as error:  # pragma: no cover
                self.get_logger().error(f"Mechatronics cycle failed: {error!r}")
                self._transition("ERROR", str(error))
                self._safe_mechanics()
                while (
                    not self._stop_event.is_set()
                    and not self._restart_event.wait(0.25)
                ):
                    pass
                continue

            if not self.auto_repeat:
                while (
                    not self._stop_event.is_set()
                    and not self._restart_event.wait(0.25)
                ):
                    pass
                continue

    def _run(self) -> None:
        self._cycle_id = 1
        self._transition("WAIT_SERVICES", "waiting for Gazebo lifecycle services")
        self._wait_for_services(20.0)
        self._cleanup_stale_models()
        self._initialise_mechanics()

        self._active_kty = self._new_kty_name(self._cycle_id)
        self._queue_kty = self._new_kty_name(self._cycle_id + 1)
        self._spawn_kty(self._active_kty, self.active_target_x)
        self._spawn_kty(self._queue_kty, self.queue_spawn_x)
        self._set_commands(clamps=self.clamp_closed, locator=self.locator_up)
        self._set_vibration("weak")
        self._interruptible_sleep(1.0)

        while not self._stop_event.is_set():
            self._load_until_full()
            self._close_gate_and_compact()
            self._changeover()
            self._cycle_id += 1

    def _initialise_mechanics(self) -> None:
        self._set_vibration("off")
        self._set_commands(
            infeed=0.0,
            active=0.0,
            outfeed=0.0,
            pusher=0.0,
            clamps=0.0,
            gate=self.gate_open,
            locator=self.locator_up,
        )
        self._interruptible_sleep(0.8)

    def _safe_mechanics(self) -> None:
        self._set_vibration("off")
        self._set_commands(
            infeed=0.0,
            active=0.0,
            outfeed=0.0,
            pusher=0.0,
            clamps=0.0,
            gate=0.0,
            locator=self.locator_up,
        )

    def _load_until_full(self) -> None:
        self._transition(
            "LOAD",
            "gate open; loading active KTY with weak 8 Hz vibration",
        )
        self._set_commands(
            gate=self.gate_open,
            clamps=self.clamp_closed,
            locator=self.locator_up,
            infeed=0.0,
            active=0.0,
            outfeed=0.0,
        )
        self._set_vibration("weak")

        next_spawn = time.monotonic()
        threshold_since: float | None = None
        while True:
            self._check_interrupt()
            now = time.monotonic()
            if now >= next_spawn:
                name = self._spawn_product()
                self._active_product_names.add(name)
                next_spawn = now + self.spawn_interval

            with self._lock:
                fill = dict(self._latest_fill)
            reached = (
                float(fill.get("fill_ratio", 0.0)) >= self.fill_ratio_threshold
                or float(fill.get("maximum_height_m", 0.0)) >= self.height_threshold
            )
            if reached and bool(fill.get("camera_ok", False)):
                if threshold_since is None:
                    threshold_since = now
                elif now - threshold_since >= self.fill_persistence:
                    return
            else:
                threshold_since = None
            self._interruptible_sleep(0.05)

    def _close_gate_and_compact(self) -> None:
        self._transition(
            "CLOSE_GATE",
            "closing hinged chute gate before KTY changeover",
        )
        self._set_commands(gate=0.0)
        self._interruptible_sleep(0.45)

        self._transition(
            "COMPACT",
            "18 Hz physical vibration, ±3 mm, while products accumulate on chute",
        )
        self._set_vibration("strong")
        next_spawn = time.monotonic()
        deadline = time.monotonic() + self.strong_duration
        while time.monotonic() < deadline:
            self._check_interrupt()
            now = time.monotonic()
            if now >= next_spawn:
                self._spawn_product()
                next_spawn = now + self.spawn_interval
            self._interruptible_sleep(0.03)
        self._set_vibration("off")
        self._interruptible_sleep(0.25)

    def _changeover(self) -> None:
        old_kty = self._active_kty
        next_kty = self._queue_kty
        old_products = self._products_inside(old_kty)

        self._transition(
            "EJECT_ACTIVE",
            "release clamps and move loaded KTY on active/outfeed rollers",
        )
        self._set_commands(
            clamps=0.0,
            locator=0.0,
            active=self.roller_speed,
            outfeed=self.roller_speed,
        )
        self._wait_for_x(old_kty, minimum_x=1.25, timeout_s=7.0)

        self._transition(
            "POSITION_NEXT",
            "raise locator, extend pusher and move queued KTY to active position",
        )
        self._set_commands(
            locator=self.locator_up,
            pusher=self.pusher_extended,
            infeed=self.roller_speed,
            active=self.roller_speed,
        )

        self._approach_locator(next_kty, timeout_s=8.0)
        self._set_commands(infeed=0.0, active=0.0, pusher=0.0)
        self._set_commands(clamps=self.clamp_closed)
        self._set_vibration("weak")

        self._transition(
            "VERIFY_READY",
            "checking position, velocity, clamps, camera and previous-KTY clearance",
        )
        self._wait_until_ready(next_kty, old_kty, timeout_s=8.0)

        self._remove_model(old_kty)
        self._known_models.discard(old_kty)
        for name in sorted(old_products):
            self._remove_model(name)
            self._known_models.discard(name)
            self._active_product_names.discard(name)
        self._set_commands(outfeed=0.0)

        self._active_kty = next_kty
        self._queue_kty = self._new_kty_name(self._cycle_id + 2)
        self._spawn_kty(self._queue_kty, self.queue_spawn_x)

        self._transition(
            "OPEN_GATE",
            "new KTY ready; opening gate and releasing accumulated products",
        )
        self._set_commands(gate=self.gate_open)
        self._interruptible_sleep(0.5)

    def _approach_locator(self, name: str, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_interrupt()
            pose = self._read_pose(name)
            if pose is None:
                self._interruptible_sleep(0.15)
                continue
            if pose.x >= self.active_target_x - 0.18:
                self._set_commands(
                    infeed=self.slow_roller_speed,
                    active=self.slow_roller_speed,
                )
            if pose.x >= self.active_target_x - 0.015:
                self._set_commands(infeed=0.0, active=0.0)
                return
            self._interruptible_sleep(0.12)
        raise RuntimeError(f"{name} did not reach the active locator")

    def _wait_until_ready(
        self,
        name: str,
        previous_name: str,
        timeout_s: float,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        last_pose: Pose | None = None
        last_time: float | None = None
        while time.monotonic() < deadline:
            self._check_interrupt()
            now = time.monotonic()
            poses = self._read_world_poses()
            pose = poses.get(name)
            previous = poses.get(previous_name)
            if pose is None:
                stable_since = None
                self._interruptible_sleep(0.15)
                continue

            velocity = math.inf
            if last_pose is not None and last_time is not None and now > last_time:
                velocity = abs(pose.x - last_pose.x) / (now - last_time)
            last_pose = pose
            last_time = now

            with self._lock:
                camera_ok = bool(self._latest_fill.get("camera_ok", False))
                fill_ratio = float(self._latest_fill.get("fill_ratio", 1.0))
            previous_clear = previous is None or previous.x >= 1.20
            ready = (
                abs(pose.x - self.active_target_x) <= self.position_tolerance
                and velocity <= self.velocity_tolerance
                and previous_clear
                and camera_ok
                and fill_ratio <= 0.15
            )
            if ready:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self.ready_persistence:
                    return
            else:
                stable_since = None
            self._interruptible_sleep(0.12)
        raise RuntimeError(
            f"{name} failed readiness checks at active position"
        )

    def _wait_for_x(self, name: str, minimum_x: float, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_interrupt()
            pose = self._read_pose(name)
            if pose is not None and pose.x >= minimum_x:
                return
            self._interruptible_sleep(0.15)
        raise RuntimeError(f"{name} did not clear active zone")

    def _products_inside(self, kty_name: str) -> set[str]:
        poses = self._read_world_poses()
        kty = poses.get(kty_name)
        if kty is None:
            return set()
        result = set()
        for name, pose in poses.items():
            if not name.startswith("kty_mech_product_"):
                continue
            if (
                abs(pose.x - kty.x) <= 0.34
                and abs(pose.y - kty.y) <= 0.23
                and self.kty_bottom_z - 0.03 <= pose.z <= 1.02
            ):
                result.add(name)
        return result

    def _new_kty_name(self, serial: int) -> str:
        return f"kty_mech_container_{serial:04d}"

    def _spawn_kty(self, name: str, x: float) -> None:
        if not self._create_model(
            name,
            make_kty_sdf(name, mass=3.2),
            x=x,
            y=0.0,
            z=self.kty_bottom_z,
        ):
            raise RuntimeError(f"Gazebo rejected KTY {name}")
        self._known_models.add(name)

    def _spawn_product(self) -> str:
        self._product_serial += 1
        name = f"kty_mech_product_{self._product_serial:06d}"
        profile = MECH_PRODUCT_PROFILES[
            (self._product_serial - 1) % len(MECH_PRODUCT_PROFILES)
        ]
        y_pattern = (-0.12, -0.06, 0.0, 0.06, 0.12)
        y = y_pattern[(self._product_serial - 1) % len(y_pattern)]
        if not self._create_model(
            name,
            make_flow_product_sdf(name, profile),
            x=-1.10,
            y=y,
            z=1.52 + 0.012 * (self._product_serial % 2),
        ):
            raise RuntimeError(f"Gazebo rejected product {name}")
        self._known_models.add(name)
        return name

    def _wait_for_services(self, timeout_s: float) -> None:
        required = {self._create_service, self._remove_service}
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
            if result.returncode == 0 and required <= available:
                return
            time.sleep(0.25)
        raise RuntimeError(f"Gazebo services unavailable: {sorted(required)}")

    def _create_model(
        self,
        name: str,
        sdf: str,
        *,
        x: float,
        y: float,
        z: float,
    ) -> bool:
        request = "\n".join(
            (
                f"sdf: {json.dumps(sdf)}",
                f'name: "{name}"',
                "allow_renaming: false",
                f"pose {{ position {{ x: {x:.9f} y: {y:.9f} z: {z:.9f} }} }}",
            )
        )
        return self._call_boolean_service(
            self._create_service,
            "gz.msgs.EntityFactory",
            request,
        )

    def _remove_model(self, name: str) -> bool:
        return self._call_boolean_service(
            self._remove_service,
            "gz.msgs.Entity",
            f'name: "{name}" type: MODEL',
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
                f"Service {service} rejected request: "
                f"{result.stdout.strip()} {result.stderr.strip()}"
            )
        return success

    def _read_pose(self, name: str) -> Pose | None:
        return self._read_world_poses().get(name)

    def _read_world_poses(self) -> dict[str, Pose]:
        result = subprocess.run(
            ["gz", "topic", "-e", "-t", self._pose_topic, "-n", "1"],
            capture_output=True,
            text=True,
            timeout=6.0,
            check=False,
        )
        if result.returncode != 0:
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

        poses: dict[str, Pose] = {}
        for block in blocks:
            name_match = re.search(r'name:\s*"([^"]+)"', block)
            position_match = re.search(
                r"position\s*\{([^}]*)\}",
                block,
                flags=re.DOTALL,
            )
            if name_match is None or position_match is None:
                continue
            fields = {
                key: float(value)
                for key, value in re.findall(
                    r"\b([xyz])\s*:\s*"
                    r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)",
                    position_match.group(1),
                )
            }
            poses[name_match.group(1)] = Pose(
                x=fields.get("x", 0.0),
                y=fields.get("y", 0.0),
                z=fields.get("z", 0.0),
            )
        return poses

    def _cleanup_stale_models(self) -> None:
        poses = self._read_world_poses()
        for name in sorted(poses):
            if name.startswith("kty_mech_container_") or name.startswith(
                "kty_mech_product_"
            ):
                self._remove_model(name)
        self._known_models.clear()
        self._active_product_names.clear()

    def _cleanup_known_models(self) -> None:
        for name in sorted(self._known_models):
            self._remove_model(name)
        self._known_models.clear()
        self._active_product_names.clear()

    def _check_interrupt(self) -> None:
        if self._stop_event.is_set() or self._restart_event.is_set():
            raise RestartRequested()

    def _interruptible_sleep(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self._check_interrupt()
            time.sleep(min(0.05, deadline - time.monotonic()))

    def close(self) -> None:
        self._stop_event.set()
        self._restart_event.set()
        self._worker.join(timeout=3.0)
        self._safe_mechanics()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycle()
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
