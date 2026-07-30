"""Runtime-v13: persistent pose feedback, varied Ozon products and overlapped changeover.

The accepted v12 fill guards and v11 vibration / despawn ordering are retained.
This runtime removes every `gz topic` subprocess from the controller. Gazebo model
poses arrive through one persistent ros_gz_bridge Pose_V -> TFMessage bridge.

The queued KTY is prefed to a safe staging point as soon as the loaded KTY begins
to leave. Product spawning is slower, gate-closed accumulation is capped, and the
profile set spans the Ozon task range from 15 x 35 x 10 mm to 400 x 320 x 280 mm.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from tf2_msgs.msg import TFMessage

from .flow_cycle import ProductProfile, make_flow_product_sdf
from .mechatronics_cycle import Pose
from .mechatronics_cycle_v12 import KtyMechatronicsCycleV12


OZON_PRODUCT_PROFILES = (
    # Exact lower bound from the task specification: 15 x 35 x 10 mm.
    ProductProfile(0.035, 0.015, 0.010, 0.018, 0.12, -0.13, (0.95, 0.72, 0.18)),
    ProductProfile(0.060, 0.040, 0.015, 0.045, -0.18, 0.11, (0.32, 0.72, 0.92)),
    ProductProfile(0.090, 0.060, 0.025, 0.090, 0.24, -0.08, (0.44, 0.82, 0.42)),
    ProductProfile(0.120, 0.080, 0.045, 0.180, -0.28, 0.07, (0.88, 0.46, 0.20)),
    ProductProfile(0.160, 0.100, 0.070, 0.320, 0.18, -0.05, (0.38, 0.52, 0.90)),
    ProductProfile(0.200, 0.130, 0.100, 0.650, -0.20, 0.08, (0.78, 0.68, 0.20)),
    ProductProfile(0.240, 0.160, 0.120, 0.950, 0.15, -0.07, (0.28, 0.70, 0.62)),
    ProductProfile(0.280, 0.190, 0.145, 1.350, -0.12, 0.05, (0.76, 0.34, 0.58)),
    ProductProfile(0.320, 0.230, 0.180, 2.000, 0.09, 0.00, (0.48, 0.30, 0.18)),
    ProductProfile(0.360, 0.270, 0.220, 3.100, -0.07, 0.00, (0.22, 0.56, 0.82)),
    # Exact upper bound from the task specification: 400 x 320 x 280 mm.
    ProductProfile(0.400, 0.320, 0.280, 4.800, 0.04, 0.00, (0.86, 0.40, 0.16)),
)


class KtyMechatronicsCycleV13(KtyMechatronicsCycleV12):
    """V12 mechanics with persistent pose cache and overlapped KTY admission."""

    POSE_TOPIC = "/kty/mech/model_poses"

    def __init__(self) -> None:
        # The inherited Node constructor starts the worker through dynamic dispatch.
        # Keep it blocked until the pose subscription and v13 parameters exist.
        self._v13_ready = threading.Event()
        self._pose_cache_lock = threading.Lock()
        self._pose_cache: dict[str, Pose] = {}
        self._pose_cache_received_at = 0.0
        self._prefeed_reached = False
        self._prefeed_best_x = -math.inf
        self._closed_gate_spawned = 0
        super().__init__()

        defaults = {
            "pose_cache_timeout_s": 2.5,
            "prefeed_target_x_m": -0.50,
            "prefeed_timeout_s": 24.0,
            "closed_gate_spawn_interval_s": 3.0,
            "closed_gate_max_products": 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        read = lambda name: self.get_parameter(name).value
        self.pose_cache_timeout = float(read("pose_cache_timeout_s"))
        self.prefeed_target_x = float(read("prefeed_target_x_m"))
        self.prefeed_timeout = float(read("prefeed_timeout_s"))
        self.closed_gate_spawn_interval = float(
            read("closed_gate_spawn_interval_s")
        )
        self.closed_gate_max_products = int(read("closed_gate_max_products"))

        self.create_subscription(
            TFMessage,
            self.POSE_TOPIC,
            self._on_model_poses,
            10,
        )
        self._v13_ready.set()
        self.get_logger().info(
            "Runtime v13: persistent ROS pose bridge, Ozon size mix, slower feeder "
            "and concurrent queued-KTY prefeed"
        )

    def _worker_main(self) -> None:
        self._v13_ready.wait()
        super()._worker_main()

    @staticmethod
    def _normalise_entity_name(frame_id: str) -> str:
        name = frame_id.strip().strip("/")
        if "::" in name:
            name = name.rsplit("::", 1)[-1]
        if "/" in name:
            name = name.rsplit("/", 1)[-1]
        return name

    def _on_model_poses(self, message: TFMessage) -> None:
        poses: dict[str, Pose] = {}
        for transform in message.transforms:
            name = self._normalise_entity_name(transform.child_frame_id)
            if not name:
                continue
            translation = transform.transform.translation
            poses[name] = Pose(
                x=float(translation.x),
                y=float(translation.y),
                z=float(translation.z),
            )
        if not poses:
            return
        with self._pose_cache_lock:
            self._pose_cache = poses
            self._pose_cache_received_at = time.monotonic()

    def _read_world_poses(self) -> dict[str, Pose]:
        """Return the persistent ROS pose cache; never launch gz-transport-topic."""
        with self._pose_cache_lock:
            return dict(self._pose_cache)

    def _pose_cache_is_fresh(self) -> bool:
        with self._pose_cache_lock:
            return (
                bool(self._pose_cache)
                and time.monotonic() - self._pose_cache_received_at
                <= self.pose_cache_timeout
            )

    def _wait_for_services(self, timeout_s: float) -> None:
        super()._wait_for_services(timeout_s)
        deadline = time.monotonic() + max(12.0, timeout_s)
        while time.monotonic() < deadline:
            self._check_interrupt()
            if self._pose_cache_is_fresh():
                return
            self._interruptible_sleep(0.10)
        raise RuntimeError(
            f"no fresh model poses received on {self.POSE_TOPIC}; check pose bridge"
        )

    def _spawn_product(self) -> str:
        self._product_serial += 1
        name = f"kty_mech_product_{self._product_serial:06d}"
        profile = OZON_PRODUCT_PROFILES[
            (self._product_serial - 1) % len(OZON_PRODUCT_PROFILES)
        ]
        # Large products are centred; small and medium products use their profile
        # offset to keep the stream visually varied without striking chute walls.
        max_lateral = max(0.0, 0.26 - 0.5 * profile.size_y)
        y = max(-max_lateral, min(max_lateral, profile.spawn_y))
        z = 1.54 + 0.5 * profile.size_z
        if not self._create_model(
            name,
            make_flow_product_sdf(name, profile),
            x=-1.10,
            y=y,
            z=z,
        ):
            raise RuntimeError(f"Gazebo rejected product {name}")
        self._known_models.add(name)
        return name

    def _close_gate_and_compact(self) -> None:
        self._transition(
            "CLOSE_GATE",
            "closing slide gate before v11 compaction; feeder enters reduced-rate buffer mode",
        )
        self._set_commands(gate=0.0)
        self._interruptible_sleep(0.45)

        with self._lock:
            before = dict(self._latest_fill)

        low_hz = self.strong_frequency - self.strong_sweep_hz
        high_hz = self.strong_frequency + self.strong_sweep_hz
        self._closed_gate_spawned = 0
        self._transition(
            "COMPACT",
            (
                f"vertical sweep {low_hz:.1f}-{high_hz:.1f} Hz, "
                f"+/-{1000.0 * self.strong_amplitude:.1f} mm; "
                f"closed-gate feeder {self.closed_gate_spawn_interval:.1f} s, "
                f"cap {self.closed_gate_max_products}"
            ),
        )
        self._set_vibration("strong")
        next_spawn = time.monotonic()
        deadline = time.monotonic() + self.strong_duration
        while time.monotonic() < deadline:
            self._check_interrupt()
            now = time.monotonic()
            if (
                now >= next_spawn
                and self._closed_gate_spawned < self.closed_gate_max_products
            ):
                self._spawn_product()
                self._closed_gate_spawned += 1
                next_spawn = now + self.closed_gate_spawn_interval
            self._interruptible_sleep(0.03)

        self._set_vibration("off")
        self._interruptible_sleep(self.vibration_settle_s)

        with self._lock:
            after = dict(self._latest_fill)
        fill_before = float(before.get("fill_ratio", 0.0) or 0.0)
        fill_after = float(after.get("fill_ratio", 0.0) or 0.0)
        height_before = float(before.get("maximum_height_m", 0.0) or 0.0)
        height_after = float(after.get("maximum_height_m", 0.0) or 0.0)
        self._last_compaction = {
            "fill_before": fill_before,
            "fill_after": fill_after,
            "fill_delta": fill_after - fill_before,
            "height_before_m": height_before,
            "height_after_m": height_after,
            "height_drop_m": height_before - height_after,
        }
        self.get_logger().info(
            "Compaction measurement: "
            f"height {height_before:.3f}->{height_after:.3f} m; "
            f"fill {fill_before:.3f}->{fill_after:.3f}; "
            f"buffered products={self._closed_gate_spawned}"
        )

    def _eject_and_prefeed(self, old_kty: str, next_kty: str) -> None:
        """Move old KTY out while advancing the queued KTY to a safe staging X."""
        self._prefeed_reached = False
        self._prefeed_best_x = -math.inf
        self._transition(
            "EJECT_ACTIVE",
            "old KTY outfeed active; queued KTY simultaneously prefeds to staging point",
        )
        self._set_commands(
            active=self.roller_speed,
            outfeed=self.roller_speed,
            infeed=self.roller_speed,
            pusher=self.pusher_extended,
        )

        deadline = time.monotonic() + max(24.0, self.prefeed_timeout)
        pusher_retracted = False
        while time.monotonic() < deadline:
            self._check_interrupt()
            poses = self._read_world_poses()
            old_pose = poses.get(old_kty)
            next_pose = poses.get(next_kty)

            if next_pose is not None:
                self._prefeed_best_x = max(self._prefeed_best_x, next_pose.x)
                if next_pose.x >= self.queue_spawn_x + 0.28:
                    pusher_retracted = True
                if next_pose.x >= self.prefeed_target_x:
                    self._prefeed_reached = True
                    self._set_commands(infeed=0.0, pusher=0.0)
                elif not self._prefeed_reached:
                    self._set_commands(
                        infeed=self.roller_speed,
                        pusher=0.0 if pusher_retracted else self.pusher_extended,
                    )

            if old_pose is not None and old_pose.x >= 1.25:
                return
            self._interruptible_sleep(0.12)

        old_x = self._read_pose(old_kty)
        old_text = "unknown" if old_x is None else f"{old_x.x:.3f}"
        raise RuntimeError(
            f"{old_kty} did not clear active zone during overlapped changeover; "
            f"old x={old_text}, queued best x={self._prefeed_best_x:.3f}"
        )

    def _changeover(self) -> None:
        old_kty = self._active_kty
        next_kty = self._queue_kty
        old_products = self._products_inside(old_kty)

        self._transition(
            "EJECT_ACTIVE",
            "remove locator and open clamps before overlapped outfeed / prefeed",
        )
        self._set_commands(
            clamps=0.0,
            locator=0.0,
            active=0.0,
            outfeed=0.0,
            infeed=0.0,
            pusher=0.0,
        )
        self._remove_locator_model()
        self._interruptible_sleep(1.2)

        self._eject_and_prefeed(old_kty, next_kty)
        self._set_commands(active=0.0, outfeed=0.0, infeed=0.0, pusher=0.0)
        self._despawn_loaded_kty(old_kty, old_products)

        self._transition(
            "POSITION_NEXT",
            "old KTY absent; locator restored and prefed KTY completes final positioning",
        )
        self._spawn_locator_model()
        self._set_commands(
            locator=0.0,
            pusher=0.0 if self._prefeed_reached else self.pusher_extended,
            infeed=self.slow_roller_speed if self._prefeed_reached else self.roller_speed,
            active=self.slow_roller_speed if self._prefeed_reached else self.roller_speed,
            outfeed=0.0,
        )
        self._approach_locator(next_kty, timeout_s=self.position_next_timeout)
        self._set_commands(infeed=0.0, active=0.0, pusher=0.0)
        self._set_commands(clamps=self.clamp_closed)
        self._set_vibration("weak")

        self._transition(
            "VERIFY_READY",
            "checking prefed KTY, clamps, locator and fresh empty depth frame",
        )
        self._wait_until_ready(next_kty, old_kty, timeout_s=self.readiness_timeout)

        self._active_kty = next_kty
        self._queue_kty = self._new_kty_name(self._cycle_id + 2)
        self._spawn_kty(self._queue_kty, self.queue_spawn_x)

        self._transition(
            "OPEN_GATE",
            "new KTY ready; removing slide gate and confirming open state",
        )
        self._set_commands(gate=self.gate_open)
        self._ensure_gate_open(timeout_s=8.0)
        self._interruptible_sleep(0.35)

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        with self._pose_cache_lock:
            pose_age = time.monotonic() - self._pose_cache_received_at
            pose_count = len(self._pose_cache)
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v13",
                "pose_feedback": "persistent_ros_gz_pose_bridge",
                "pose_cache_age_s": pose_age,
                "pose_cache_models": pose_count,
                "prefeed_target_x_m": self.prefeed_target_x,
                "prefeed_reached": self._prefeed_reached,
                "prefeed_best_x_m": (
                    self._prefeed_best_x if math.isfinite(self._prefeed_best_x) else None
                ),
                "closed_gate_spawn_interval_s": self.closed_gate_spawn_interval,
                "closed_gate_spawned_products": self._closed_gate_spawned,
                "closed_gate_max_products": self.closed_gate_max_products,
                "product_profile_count": len(OZON_PRODUCT_PROFILES),
                "product_size_min_m": [0.015, 0.035, 0.010],
                "product_size_max_m": [0.400, 0.320, 0.280],
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV13()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
