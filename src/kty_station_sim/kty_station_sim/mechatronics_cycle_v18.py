"""Runtime-v18: medium carton mix and repeatable queued-KTY admission.

The supplied long-run log proves that vision is not the cause of the third-cycle
failure: container_0002 completes loading, compaction, ejection and despawn, while
container_0003 stalls at x=-0.748 during POSITION_NEXT.  This runtime keeps the
accepted v17 gate / registry behavior, limits carton dimensions to a medium range,
and adds a bounded empty-KTY recovery when physical admission makes no progress.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy

from .flow_cycle import ProductProfile, make_flow_product_sdf
from .mechatronics_cycle_v17 import KtyMechatronicsCycleV17


MEDIUM_PRODUCT_PROFILES = (
    ProductProfile(0.035, 0.015, 0.010, 0.018, 0.12, -0.13, (0.95, 0.72, 0.18)),
    ProductProfile(0.060, 0.040, 0.015, 0.045, -0.18, 0.11, (0.32, 0.72, 0.92)),
    ProductProfile(0.090, 0.060, 0.025, 0.090, 0.24, -0.08, (0.44, 0.82, 0.42)),
    ProductProfile(0.120, 0.080, 0.045, 0.180, -0.28, 0.07, (0.88, 0.46, 0.20)),
    ProductProfile(0.160, 0.100, 0.070, 0.320, 0.18, -0.05, (0.38, 0.52, 0.90)),
    ProductProfile(0.190, 0.120, 0.085, 0.500, -0.20, 0.08, (0.78, 0.68, 0.20)),
    ProductProfile(0.220, 0.140, 0.100, 0.720, 0.15, -0.07, (0.28, 0.70, 0.62)),
    ProductProfile(0.240, 0.160, 0.115, 0.900, -0.12, 0.05, (0.76, 0.34, 0.58)),
    ProductProfile(0.260, 0.180, 0.130, 1.100, 0.09, 0.00, (0.48, 0.30, 0.18)),
    ProductProfile(0.280, 0.190, 0.145, 1.350, -0.07, 0.00, (0.22, 0.56, 0.82)),
)


class KtyMechatronicsCycleV18(KtyMechatronicsCycleV17):
    """V17 lifecycle with medium products and bounded admission recovery."""

    def __init__(self) -> None:
        self._v18_ready = threading.Event()
        self._position_recovery_respawns = 0
        self._position_recovery_failures = 0
        self._last_position_recovery_name = ""
        self._last_position_recovery_from_x: float | None = None
        self._last_position_recovery_from_z: float | None = None
        super().__init__()

        defaults = {
            "position_recovery_after_stalls": 2,
            "position_recovery_spawn_x_m": -0.320,
            "position_recovery_max_respawns": 2,
            "position_recovery_confirm_timeout_s": 8.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        read = lambda name: self.get_parameter(name).value
        self.position_recovery_after_stalls = int(
            read("position_recovery_after_stalls")
        )
        self.position_recovery_spawn_x = float(
            read("position_recovery_spawn_x_m")
        )
        self.position_recovery_max_respawns = int(
            read("position_recovery_max_respawns")
        )
        self.position_recovery_confirm_timeout = float(
            read("position_recovery_confirm_timeout_s")
        )
        if self.position_recovery_after_stalls < 1:
            raise ValueError("position_recovery_after_stalls must be positive")
        if self.position_recovery_max_respawns < 1:
            raise ValueError("position_recovery_max_respawns must be positive")
        if self.position_recovery_spawn_x >= self.active_target_x - 0.08:
            raise ValueError("position recovery spawn must remain behind active target")

        self._v18_ready.set()
        self.get_logger().info(
            "Runtime v18: medium cartons up to 280x190x145 mm and bounded "
            "queued-KTY admission recovery"
        )

    def _worker_main(self) -> None:
        self._v18_ready.wait()
        super()._worker_main()

    def _spawn_product(self) -> str:
        """Use the resilient v16 create loop with the reduced product profile set."""
        self._product_serial += 1
        name = f"kty_mech_product_{self._product_serial:06d}"
        profile = MEDIUM_PRODUCT_PROFILES[
            (self._product_serial - 1) % len(MEDIUM_PRODUCT_PROFILES)
        ]
        max_lateral = max(0.0, 0.26 - 0.5 * profile.size_y)
        y = max(-max_lateral, min(max_lateral, profile.spawn_y))
        z = 1.54 + 0.5 * profile.size_z
        attempts_in_round = 0

        while True:
            self._check_interrupt()
            attempts_in_round += 1
            self._product_spawn_attempts += 1
            if self._registry_has_model(name):
                self._known_models.add(name)
                self._feeder_backoff_active = False
                return name

            accepted = self._create_model(
                name,
                make_flow_product_sdf(name, profile),
                x=-1.10,
                y=y,
                z=z,
            )
            if self._wait_registry_presence(
                name,
                present=True,
                timeout_s=self.product_confirmation_timeout,
            ):
                self._known_models.add(name)
                if attempts_in_round > 1 or not accepted:
                    self._product_spawn_recoveries += 1
                    self.get_logger().info(
                        f"Recovered delayed product create for {name} after "
                        f"{attempts_in_round} attempt(s)"
                    )
                self._feeder_backoff_active = False
                return name

            self._product_spawn_failures += 1
            self._remove_model(name)
            self._wait_registry_presence(name, present=False, timeout_s=0.75)
            if attempts_in_round < self.product_spawn_max_attempts:
                self.get_logger().warning(
                    f"Product {name} was not confirmed after create attempt "
                    f"{attempts_in_round}/{self.product_spawn_max_attempts}; retrying"
                )
                self._interruptible_sleep(
                    self.product_spawn_retry_backoff * attempts_in_round
                )
                continue

            self._feeder_backoff_active = True
            self.get_logger().error(
                f"Feeder back-pressure: {name} not created after "
                f"{attempts_in_round} attempts; pausing "
                f"{self.product_spawn_failure_pause:.1f} s before retry"
            )
            self._interruptible_sleep(self.product_spawn_failure_pause)
            attempts_in_round = 0

    def _recover_empty_queue_kty(self, name: str, pose) -> None:
        """Recreate only the empty queued KTY beyond the troublesome hand-off edge."""
        self._set_commands(infeed=0.0, active=0.0, outfeed=0.0, pusher=0.0)
        self._last_position_recovery_name = name
        self._last_position_recovery_from_x = float(pose.x)
        self._last_position_recovery_from_z = float(pose.z)
        self.get_logger().warning(
            f"Deterministic admission recovery for {name}: "
            f"x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}; "
            f"recreating empty KTY at x={self.position_recovery_spawn_x:.3f}"
        )

        self._remove_model(name)
        if not self._wait_registry_presence(
            name,
            present=False,
            timeout_s=self.position_recovery_confirm_timeout,
        ):
            self._position_recovery_failures += 1
            raise RuntimeError(f"could not remove stalled queued KTY {name}")
        self._known_models.discard(name)

        self._spawn_kty(name, self.position_recovery_spawn_x)
        if not self._wait_registry_presence(
            name,
            present=True,
            timeout_s=self.position_recovery_confirm_timeout,
        ):
            self._position_recovery_failures += 1
            raise RuntimeError(f"could not confirm recovered queued KTY {name}")
        self._position_recovery_respawns += 1
        self._interruptible_sleep(0.50)

    def _approach_locator(self, name: str, timeout_s: float) -> None:
        """Approach physically, then recover a persistently jammed empty KTY."""
        deadline = time.monotonic() + max(timeout_s, self.position_next_timeout)
        best_x = -math.inf
        last_progress_at = time.monotonic()
        recovery_until = 0.0
        pusher_retracted = False
        consecutive_stalls = 0
        respawns_this_approach = 0

        while time.monotonic() < deadline:
            self._check_interrupt()
            now = time.monotonic()
            pose = self._read_pose(name)
            if pose is None:
                self._set_commands(
                    pusher=self.pusher_extended,
                    infeed=self.roller_speed,
                    active=self.roller_speed,
                )
                self._interruptible_sleep(0.25)
                continue

            if pose.x > best_x + 0.010:
                best_x = pose.x
                last_progress_at = now
                consecutive_stalls = 0

            remaining = self.active_target_x - pose.x
            if pose.x >= self.active_target_x - 0.020:
                self._set_commands(infeed=0.0, active=0.0, pusher=0.0)
                self._interruptible_sleep(0.60)
                settled = self._read_pose(name)
                if settled is not None and settled.x >= self.active_target_x - 0.035:
                    return

            if remaining > 0.45:
                speed = self.roller_speed
            elif remaining > 0.12:
                speed = min(self.roller_speed, 0.35)
            else:
                speed = min(self.slow_roller_speed, 0.18)

            if not pusher_retracted and pose.x >= self.queue_spawn_x + 0.28:
                pusher_retracted = True
            pusher = 0.0 if pusher_retracted else self.pusher_extended

            if now - last_progress_at >= self.position_progress_timeout:
                self._position_recovery_pulses += 1
                consecutive_stalls += 1
                last_progress_at = now
                self.get_logger().warning(
                    f"{name} positioning stalled at x={pose.x:.3f}, "
                    f"y={pose.y:.3f}, z={pose.z:.3f}; "
                    f"recovery pulse {self._position_recovery_pulses}"
                )
                if consecutive_stalls >= self.position_recovery_after_stalls:
                    if respawns_this_approach >= self.position_recovery_max_respawns:
                        self._position_recovery_failures += 1
                        raise RuntimeError(
                            f"{name} remained jammed after "
                            f"{respawns_this_approach} deterministic recoveries"
                        )
                    self._recover_empty_queue_kty(name, pose)
                    respawns_this_approach += 1
                    best_x = self.position_recovery_spawn_x
                    last_progress_at = time.monotonic()
                    consecutive_stalls = 0
                    pusher_retracted = True
                    recovery_until = last_progress_at + 1.5
                    continue
                recovery_until = now + 2.0

            if now < recovery_until:
                speed = max(speed, min(0.85, 1.25 * self.roller_speed))
                pusher = 0.0 if pusher_retracted else self.pusher_extended

            self._set_commands(
                pusher=pusher,
                infeed=speed,
                active=speed,
                outfeed=0.0,
            )
            self._interruptible_sleep(0.20)

        pose = self._read_pose(name)
        x_text = "unknown" if pose is None else f"{pose.x:.3f}"
        z_text = "unknown" if pose is None else f"{pose.z:.3f}"
        best_text = "unknown" if not math.isfinite(best_x) else f"{best_x:.3f}"
        raise RuntimeError(
            f"{name} did not reach active locator within "
            f"{self.position_next_timeout:.0f} s; last x={x_text}, "
            f"last z={z_text}, best x={best_text}"
        )

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v18",
                "product_profile_policy": "small_to_medium_only",
                "product_profile_count": len(MEDIUM_PRODUCT_PROFILES),
                "product_size_min_m": [0.035, 0.015, 0.010],
                "product_size_max_m": [0.280, 0.190, 0.145],
                "queue_admission_policy": "physical_then_bounded_empty_kty_respawn",
                "position_recovery_respawns": self._position_recovery_respawns,
                "position_recovery_failures": self._position_recovery_failures,
                "last_position_recovery_name": self._last_position_recovery_name,
                "last_position_recovery_from_x_m": self._last_position_recovery_from_x,
                "last_position_recovery_from_z_m": self._last_position_recovery_from_z,
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV18()
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
