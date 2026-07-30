"""Runtime-v16: registry-confirmed barriers and long-run feeder recovery.

Runtime v15 restored reliable named model poses through the Gazebo JSON registry.
Runtime v16 makes that registry authoritative for the slide gate and locator,
confirms every create / remove operation physically, prevents an unnoticed stale
gate from accumulating hundreds of products, and treats a transient product-create
timeout as feeder back-pressure instead of a fatal station error.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy

from .flow_cycle import make_flow_product_sdf
from .mechatronics_cycle_v13 import OZON_PRODUCT_PROFILES
from .mechatronics_cycle_v15 import KtyMechatronicsCycleV15


class KtyMechatronicsCycleV16(KtyMechatronicsCycleV15):
    """V15 mechanics with registry-confirmed lifecycle and resilient feeding."""

    def __init__(self) -> None:
        self._v16_ready = threading.Event()
        self._gate_remove_attempts = 0
        self._gate_create_attempts = 0
        self._gate_confirmed_open_count = 0
        self._gate_confirmed_closed_count = 0
        self._locator_confirmations = 0
        self._product_spawn_attempts = 0
        self._product_spawn_failures = 0
        self._product_spawn_recoveries = 0
        self._feeder_backoff_active = False
        self._load_spawn_count_current = 0
        self._load_safety_cap_closures = 0
        super().__init__()

        defaults = {
            "static_model_confirmation_timeout_s": 4.0,
            "static_model_stable_s": 0.20,
            "static_model_max_attempts": 4,
            "product_spawn_confirmation_timeout_s": 6.0,
            "product_spawn_max_attempts": 4,
            "product_spawn_retry_backoff_s": 0.75,
            "product_spawn_failure_pause_s": 3.0,
            "maximum_products_per_load": 28,
            "product_cap_close_delay_s": 2.0,
            "minimum_lifecycle_timeout_ms": 12000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        read = lambda name: self.get_parameter(name).value
        self.static_confirmation_timeout = float(
            read("static_model_confirmation_timeout_s")
        )
        self.static_stable_s = float(read("static_model_stable_s"))
        self.static_max_attempts = int(read("static_model_max_attempts"))
        self.product_confirmation_timeout = float(
            read("product_spawn_confirmation_timeout_s")
        )
        self.product_spawn_max_attempts = int(read("product_spawn_max_attempts"))
        self.product_spawn_retry_backoff = float(
            read("product_spawn_retry_backoff_s")
        )
        self.product_spawn_failure_pause = float(
            read("product_spawn_failure_pause_s")
        )
        self.maximum_products_per_load = int(read("maximum_products_per_load"))
        self.product_cap_close_delay = float(read("product_cap_close_delay_s"))
        self.minimum_lifecycle_timeout_ms = int(read("minimum_lifecycle_timeout_ms"))
        self.service_timeout_ms = max(
            self.service_timeout_ms,
            self.minimum_lifecycle_timeout_ms,
        )

        if self.static_confirmation_timeout <= 0.0:
            raise ValueError("static_model_confirmation_timeout_s must be positive")
        if self.static_stable_s < 0.0:
            raise ValueError("static_model_stable_s must be non-negative")
        if self.static_max_attempts < 1:
            raise ValueError("static_model_max_attempts must be positive")
        if self.product_confirmation_timeout <= 0.0:
            raise ValueError("product_spawn_confirmation_timeout_s must be positive")
        if self.product_spawn_max_attempts < 1:
            raise ValueError("product_spawn_max_attempts must be positive")
        if self.maximum_products_per_load < self.minimum_products_for_close:
            raise ValueError(
                "maximum_products_per_load must not be below minimum_products_for_close"
            )

        self._v16_ready.set()
        self.get_logger().info(
            "Runtime v16: registry-confirmed gate / locator, resilient feeder and "
            f"{self.service_timeout_ms} ms lifecycle timeout"
        )

    def _worker_main(self) -> None:
        self._v16_ready.wait()
        super()._worker_main()

    def _registry_has_model(self, name: str) -> bool:
        with self._pose_cache_lock:
            return name in self._pose_cache

    def _registry_product_count(self) -> int:
        with self._pose_cache_lock:
            return sum(
                name.startswith("kty_mech_product_") for name in self._pose_cache
            )

    def _wait_registry_presence(
        self,
        name: str,
        *,
        present: bool,
        timeout_s: float | None = None,
    ) -> bool:
        timeout = self.static_confirmation_timeout if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        while time.monotonic() < deadline:
            self._check_interrupt()
            cache_fresh = self._pose_cache_is_fresh()
            matches = cache_fresh and (self._registry_has_model(name) == present)
            if matches:
                now = time.monotonic()
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self.static_stable_s:
                    return True
            else:
                stable_since = None
            self._interruptible_sleep(0.05)
        return False

    def _spawn_gate_model(self) -> None:
        if self._wait_registry_presence(self.GATE_NAME, present=True, timeout_s=0.25):
            self._gate_model_spawned = True
            return

        for attempt in range(1, self.static_max_attempts + 1):
            self._gate_create_attempts += 1
            created = self._create_model(
                self.GATE_NAME,
                self._gate_sdf(),
                x=self.GATE_X,
                y=self.GATE_Y,
                z=self.GATE_Z,
            )
            if created and self._wait_registry_presence(self.GATE_NAME, present=True):
                self._gate_model_spawned = True
                self._known_models.add(self.GATE_NAME)
                self._gate_confirmed_closed_count += 1
                self.get_logger().info(
                    f"Slide gate CLOSED and registry-confirmed (attempt {attempt})"
                )
                return
            if self._wait_registry_presence(self.GATE_NAME, present=True, timeout_s=0.75):
                self._gate_model_spawned = True
                self._known_models.add(self.GATE_NAME)
                self._gate_confirmed_closed_count += 1
                self.get_logger().info("Slide gate CLOSED after delayed confirmation")
                return
            self._remove_model(self.GATE_NAME)
            self._interruptible_sleep(0.15 * attempt)

        raise RuntimeError("slide gate could not be created and confirmed in registry")

    def _remove_gate_model(self) -> None:
        # Never trust the local ledger. An asynchronous remove may have timed out
        # after the request was accepted, leaving the physical model behind.
        for attempt in range(1, self.static_max_attempts + 1):
            self._gate_remove_attempts += 1
            self._remove_model(self.GATE_NAME)
            if self._wait_registry_presence(self.GATE_NAME, present=False):
                self._gate_model_spawned = False
                self._known_models.discard(self.GATE_NAME)
                self._gate_confirmed_open_count += 1
                self.get_logger().info(
                    f"Slide gate OPEN and registry-confirmed (attempt {attempt})"
                )
                return
            self.get_logger().warning(
                f"Slide gate still present after remove attempt {attempt}; retrying"
            )
            self._interruptible_sleep(0.20 * attempt)
        raise RuntimeError(
            "slide gate remove service returned but model remained in registry"
        )

    def _gate_is_absent(self) -> bool:
        return self._pose_cache_is_fresh() and not self._registry_has_model(self.GATE_NAME)

    def _ensure_gate_open(self, timeout_s: float = 8.0) -> None:
        del timeout_s
        self._remove_gate_model()
        if not self._gate_is_absent():
            raise RuntimeError("slide gate was not physically absent after removal")

    def _spawn_locator_model(self) -> None:
        if self._wait_registry_presence(self.LOCATOR_NAME, present=True, timeout_s=0.25):
            self._locator_spawned = True
            return
        for attempt in range(1, self.static_max_attempts + 1):
            created = self._create_model(
                self.LOCATOR_NAME,
                self._locator_sdf(),
                x=self.LOCATOR_X,
                y=self.LOCATOR_Y,
                z=self.LOCATOR_Z,
            )
            if created and self._wait_registry_presence(self.LOCATOR_NAME, present=True):
                self._locator_spawned = True
                self._known_models.add(self.LOCATOR_NAME)
                self._locator_confirmations += 1
                self.get_logger().info(
                    f"Runtime locator UP and registry-confirmed (attempt {attempt})"
                )
                return
            if self._wait_registry_presence(self.LOCATOR_NAME, present=True, timeout_s=0.75):
                self._locator_spawned = True
                self._known_models.add(self.LOCATOR_NAME)
                self._locator_confirmations += 1
                return
            self._remove_model(self.LOCATOR_NAME)
            self._interruptible_sleep(0.15 * attempt)
        raise RuntimeError("runtime locator could not be created and confirmed")

    def _remove_locator_model(self) -> None:
        for attempt in range(1, self.static_max_attempts + 1):
            self._remove_model(self.LOCATOR_NAME)
            if self._wait_registry_presence(self.LOCATOR_NAME, present=False):
                self._locator_spawned = False
                self._known_models.discard(self.LOCATOR_NAME)
                self._locator_confirmations += 1
                self.get_logger().info(
                    f"Runtime locator DOWN and registry-confirmed (attempt {attempt})"
                )
                return
            self._interruptible_sleep(0.20 * attempt)
        raise RuntimeError("runtime locator remained in registry after remove attempts")

    def _spawn_product(self) -> str:
        """Create one product without converting a transient timeout into ERROR."""
        self._product_serial += 1
        name = f"kty_mech_product_{self._product_serial:06d}"
        profile = OZON_PRODUCT_PROFILES[
            (self._product_serial - 1) % len(OZON_PRODUCT_PROFILES)
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

            # Keep the station alive. The old runtime raised here, entered ERROR,
            # and left the gate closed. Back off and retry the same serial instead.
            self._feeder_backoff_active = True
            self.get_logger().error(
                f"Feeder back-pressure: {name} not created after "
                f"{attempts_in_round} attempts; pausing "
                f"{self.product_spawn_failure_pause:.1f} s before retry"
            )
            self._interruptible_sleep(self.product_spawn_failure_pause)
            attempts_in_round = 0

    def _load_until_full(self) -> None:
        """Load with a physical gate watchdog and a bounded model-count fallback."""
        self._set_commands(gate=self.gate_open)
        self._ensure_gate_open(timeout_s=8.0)
        self._transition(
            "LOAD",
            "gate registry-confirmed open; guarded loading with resilient feeder",
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

        started = time.monotonic()
        next_spawn = started
        spawned_products = 0
        threshold_since: float | None = None
        cap_since: float | None = None
        self._load_spawn_count_current = 0

        while True:
            self._check_interrupt()
            now = time.monotonic()

            # A physically stale gate during LOAD is repaired before another model
            # can be added. This is the direct guard against the product_000126 run.
            if self._registry_has_model(self.GATE_NAME):
                self.get_logger().warning(
                    "Gate appeared in LOAD; pausing feeder and forcing confirmed removal"
                )
                self._ensure_gate_open(timeout_s=8.0)
                next_spawn = time.monotonic() + self.spawn_interval

            if now >= next_spawn and spawned_products < self.maximum_products_per_load:
                name = self._spawn_product()
                self._active_product_names.add(name)
                spawned_products += 1
                self._load_spawn_count_current = spawned_products
                next_spawn = time.monotonic() + self.spawn_interval

            with self._lock:
                fill = dict(self._latest_fill)

            now = time.monotonic()
            elapsed = now - started
            fill_ratio = float(fill.get("fill_ratio", 0.0) or 0.0)
            maximum_height = float(fill.get("maximum_height_m", 0.0) or 0.0)
            occupied_ratio = float(fill.get("occupied_floor_ratio", 0.0) or 0.0)
            camera_ok = bool(fill.get("camera_ok", False))
            fill_fresh = now - self._fill_received_at <= self.fill_freshness_timeout
            enough_products = spawned_products >= self.minimum_products_for_close
            enough_time = elapsed >= self.minimum_load_duration
            volume_reached = (
                fill_ratio >= self.fill_ratio_threshold
                and occupied_ratio >= self.volume_guard_occupied
            )
            height_reached = (
                maximum_height >= self.height_threshold
                and fill_ratio >= self.height_guard_fill
                and occupied_ratio >= self.height_guard_occupied
            )
            guarded_reached = (
                enough_time
                and enough_products
                and camera_ok
                and fill_fresh
                and (volume_reached or height_reached)
            )

            cap_reached = spawned_products >= self.maximum_products_per_load
            if cap_reached:
                if cap_since is None:
                    cap_since = now
                    self.get_logger().warning(
                        f"Per-load safety cap reached ({spawned_products}); feeder held"
                    )
            else:
                cap_since = None
            cap_fallback = (
                cap_since is not None
                and now - cap_since >= self.product_cap_close_delay
                and enough_time
                and camera_ok
                and fill_fresh
            )

            reason = "waiting"
            if volume_reached:
                reason = "volume"
            elif height_reached:
                reason = "height_with_support"
            elif cap_fallback:
                reason = "product_count_safety_cap"
            elif maximum_height >= self.height_threshold:
                reason = "height_rejected_as_sparse"

            self._load_guard = {
                "elapsed_s": elapsed,
                "spawned_products": spawned_products,
                "volume_reached": volume_reached,
                "height_reached": height_reached,
                "occupied_floor_ratio": occupied_ratio,
                "close_reason": reason,
                "enough_time": enough_time,
                "enough_products": enough_products,
                "fill_fresh": fill_fresh,
                "gate_registry_present": self._registry_has_model(self.GATE_NAME),
                "safety_cap_reached": cap_reached,
            }

            accepted = guarded_reached or cap_fallback
            if accepted:
                if threshold_since is None:
                    threshold_since = now
                elif now - threshold_since >= max(
                    self.fill_persistence,
                    self.load_close_persistence,
                ):
                    if cap_fallback and not guarded_reached:
                        self._load_safety_cap_closures += 1
                    self.get_logger().info(
                        "Load close accepted: "
                        f"reason={reason} fill={fill_ratio:.3f} "
                        f"height={maximum_height:.3f} m occupied={occupied_ratio:.3f} "
                        f"spawned={spawned_products} elapsed={elapsed:.1f} s"
                    )
                    return
            else:
                threshold_since = None

            self._interruptible_sleep(0.05)

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v16",
                "static_lifecycle": "gazebo_json_registry_confirmed",
                "gate_registry_present": self._registry_has_model(self.GATE_NAME),
                "gate_remove_attempts": self._gate_remove_attempts,
                "gate_create_attempts": self._gate_create_attempts,
                "gate_confirmed_open_count": self._gate_confirmed_open_count,
                "gate_confirmed_closed_count": self._gate_confirmed_closed_count,
                "locator_confirmations": self._locator_confirmations,
                "product_spawn_attempts": self._product_spawn_attempts,
                "product_spawn_failures": self._product_spawn_failures,
                "product_spawn_recoveries": self._product_spawn_recoveries,
                "feeder_backoff_active": self._feeder_backoff_active,
                "load_spawn_count_current": self._load_spawn_count_current,
                "maximum_products_per_load": self.maximum_products_per_load,
                "load_safety_cap_closures": self._load_safety_cap_closures,
                "registry_live_products": self._registry_product_count(),
                "effective_service_timeout_ms": self.service_timeout_ms,
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV16()
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
