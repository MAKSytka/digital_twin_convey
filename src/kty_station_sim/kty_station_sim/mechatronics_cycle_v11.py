"""Runtime-v11 lifecycle ordering and stronger KTY compaction.

The loaded KTY is removed immediately after it clears the active zone. The next
KTY is not allowed to move until Gazebo confirms that the previous KTY and its
carried products no longer exist.

Compaction is also changed from the v10 8..12 Hz / +/-5 mm sweep to a slower,
larger-stroke 6.5..9.0 Hz / +/-8 mm sweep. The lower frequency is easier for the
loaded physical deck to track, while the larger displacement creates clearly
visible repeated unloading and settling of products.
"""

from __future__ import annotations

import threading
import time

import rclpy

from .mechatronics_cycle_v10 import KtyMechatronicsCycleV10


class KtyMechatronicsCycleV11(KtyMechatronicsCycleV10):
    """Stronger vibration plus deterministic despawn-before-position changeover."""

    def __init__(self) -> None:
        # The inherited constructor starts the worker using dynamic dispatch.
        # Hold it until v11 parameters and telemetry fields are available.
        self._v11_ready = threading.Event()
        self._last_despawned_kty = ""
        self._despawned_cycles = 0
        super().__init__()

        # Runtime-v11 effective profile. These values intentionally override the
        # legacy launch defaults while preserving the public parameter interface.
        self.weak_frequency = 5.0
        self.weak_amplitude = 0.0018
        self.strong_frequency = 7.75
        self.strong_sweep_hz = 1.25
        self.strong_modulation_hz = 0.22
        self.strong_amplitude = 0.0080
        self.strong_duration = 15.0
        self.strong_ramp = 2.0
        self.vibration_settle_s = 1.2

        self._v11_ready.set()
        self.get_logger().info(
            "Runtime v11: 6.5..9.0 Hz +/-8 mm compaction and confirmed "
            "despawn before POSITION_NEXT"
        )

    def _worker_main(self) -> None:
        self._v11_ready.wait()
        super()._worker_main()

    def _remove_model_confirmed(
        self,
        name: str,
        *,
        timeout_s: float = 3.0,
        attempts: int = 3,
    ) -> None:
        """Remove one model and wait until the world pose stream confirms absence."""
        for attempt in range(1, attempts + 1):
            if name not in self._read_world_poses():
                return

            self._remove_model(name)
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                self._check_interrupt()
                if name not in self._read_world_poses():
                    return
                self._interruptible_sleep(0.10)

            self.get_logger().warning(
                f"Removal confirmation timeout for {name}; retry {attempt}/{attempts}"
            )

        raise RuntimeError(f"Gazebo did not remove model {name}")

    def _despawn_loaded_kty(self, old_kty: str, product_names: set[str]) -> None:
        """Delete carried products first, then the empty KTY, with confirmation."""
        # Capture products again at the exit position. The union keeps products
        # identified before transport and any that were only clearly inside later.
        product_names.update(self._products_inside(old_kty))

        self._transition(
            "DESPAWN_ACTIVE",
            f"stopping outfeed and removing {old_kty} before positioning next KTY",
        )
        self._set_commands(active=0.0, outfeed=0.0)
        self._interruptible_sleep(0.20)

        for name in sorted(product_names):
            self._remove_model_confirmed(name, timeout_s=2.0)
            self._known_models.discard(name)
            self._active_product_names.discard(name)

        self._remove_model_confirmed(old_kty, timeout_s=3.0)
        self._known_models.discard(old_kty)
        self._last_despawned_kty = old_kty
        self._despawned_cycles += 1
        self.get_logger().info(
            f"Confirmed despawn of {old_kty} and {len(product_names)} carried products"
        )

    def _changeover(self) -> None:
        old_kty = self._active_kty
        next_kty = self._queue_kty
        old_products = self._products_inside(old_kty)

        self._transition(
            "EJECT_ACTIVE",
            "remove locator and open clamps before enabling conveyor surfaces",
        )
        self._set_commands(
            clamps=0.0,
            locator=0.0,
            active=0.0,
            outfeed=0.0,
        )
        self._remove_locator_model()
        self._interruptible_sleep(1.2)

        self._transition(
            "EJECT_ACTIVE",
            "locator absent; imposing transport velocity on active and outfeed surfaces",
        )
        self._set_commands(
            active=self.roller_speed,
            outfeed=self.roller_speed,
        )
        self._wait_for_x(old_kty, minimum_x=1.25, timeout_s=7.0)

        # Critical v11 ordering: remove the old loaded KTY now. No readiness or
        # positioning failure of the next KTY can leave it stranded in the world.
        self._despawn_loaded_kty(old_kty, old_products)

        self._transition(
            "POSITION_NEXT",
            "previous KTY absent; create locator and move queued KTY to active position",
        )
        self._spawn_locator_model()
        self._set_commands(
            locator=0.0,
            pusher=self.pusher_extended,
            infeed=self.roller_speed,
            active=self.roller_speed,
            outfeed=0.0,
        )

        self._approach_locator(next_kty, timeout_s=8.0)
        self._set_commands(infeed=0.0, active=0.0, pusher=0.0)
        self._set_commands(clamps=self.clamp_closed)
        self._set_vibration("weak")

        self._transition(
            "VERIFY_READY",
            "checking position, velocity, clamps, camera and empty active KTY",
        )
        # old_kty is already absent, so previous-clear cannot block readiness.
        self._wait_until_ready(next_kty, old_kty, timeout_s=8.0)

        self._active_kty = next_kty
        self._queue_kty = self._new_kty_name(self._cycle_id + 2)
        self._spawn_kty(self._queue_kty, self.queue_spawn_x)

        self._transition(
            "OPEN_GATE",
            "new KTY ready; opening gate and releasing accumulated products",
        )
        self._set_commands(gate=self.gate_open)
        self._interruptible_sleep(0.5)

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v11",
                "changeover_order": "eject_despawn_position_next",
                "last_despawned_kty": self._last_despawned_kty,
                "despawned_cycles": self._despawned_cycles,
                "effective_weak_frequency_hz": self.weak_frequency,
                "effective_weak_amplitude_m": self.weak_amplitude,
                "effective_strong_min_hz": self.strong_frequency - self.strong_sweep_hz,
                "effective_strong_max_hz": self.strong_frequency + self.strong_sweep_hz,
                "effective_strong_amplitude_m": self.strong_amplitude,
                "effective_strong_duration_s": self.strong_duration,
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV11()
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
