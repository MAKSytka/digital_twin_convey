"""Runtime-v16: registry-confirmed slide-gate and locator lifecycle.

Runtime v15 restored reliable named model poses through the Gazebo JSON registry.
This revision uses that registry as the source of truth for static lifecycle models.
A successful asynchronous remove-service reply is no longer treated as proof that
an entity disappeared from the world.
"""

from __future__ import annotations

import threading
import time

import rclpy

from .mechatronics_cycle_v15 import KtyMechatronicsCycleV15


class KtyMechatronicsCycleV16(KtyMechatronicsCycleV15):
    """V15 mechanics with physically confirmed gate / locator lifecycle."""

    def __init__(self) -> None:
        self._v16_ready = threading.Event()
        self._gate_remove_attempts = 0
        self._gate_create_attempts = 0
        self._gate_confirmed_open_count = 0
        self._gate_confirmed_closed_count = 0
        self._locator_confirmations = 0
        super().__init__()

        self.declare_parameter("static_model_confirmation_timeout_s", 4.0)
        self.declare_parameter("static_model_stable_s", 0.20)
        self.declare_parameter("static_model_max_attempts", 4)
        self.static_confirmation_timeout = float(
            self.get_parameter("static_model_confirmation_timeout_s").value
        )
        self.static_stable_s = float(
            self.get_parameter("static_model_stable_s").value
        )
        self.static_max_attempts = int(
            self.get_parameter("static_model_max_attempts").value
        )
        if self.static_confirmation_timeout <= 0.0:
            raise ValueError("static_model_confirmation_timeout_s must be positive")
        if self.static_stable_s < 0.0:
            raise ValueError("static_model_stable_s must be non-negative")
        if self.static_max_attempts < 1:
            raise ValueError("static_model_max_attempts must be positive")

        self._v16_ready.set()
        self.get_logger().info(
            "Runtime v16: gate and locator lifecycle confirmed by Gazebo JSON registry"
        )

    def _worker_main(self) -> None:
        self._v16_ready.wait()
        super()._worker_main()

    def _registry_has_model(self, name: str) -> bool:
        with self._pose_cache_lock:
            return name in self._pose_cache

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
            matches = cache_fresh and self._registry_has_model(name) is present
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
        # The registry, not the ledger flag, is authoritative. This also recovers
        # from a previous delayed create response or a controller restart.
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

            # A late create may have materialised after the service timeout. Check
            # once more before removing and retrying.
            if self._wait_registry_presence(self.GATE_NAME, present=True, timeout_s=0.5):
                self._gate_model_spawned = True
                self._known_models.add(self.GATE_NAME)
                self._gate_confirmed_closed_count += 1
                self.get_logger().info("Slide gate CLOSED after delayed create confirmation")
                return

            self._remove_model(self.GATE_NAME)
            self._interruptible_sleep(0.15 * attempt)

        raise RuntimeError("slide gate could not be created and confirmed in registry")

    def _remove_gate_model(self) -> None:
        # Always issue at least one remove request. The old code skipped removal
        # when the local ledger was false, which allowed a physically stale gate
        # to survive into later cycles.
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

        raise RuntimeError("slide gate remove service returned but model remained in registry")

    def _gate_is_absent(self) -> bool:
        return self._pose_cache_is_fresh() and not self._registry_has_model(self.GATE_NAME)

    def _ensure_gate_open(self, timeout_s: float = 8.0) -> None:
        del timeout_s
        self._remove_gate_model()
        if not self._gate_is_absent():
            raise RuntimeError("slide gate was not physically absent after confirmed removal")

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
            if self._wait_registry_presence(self.LOCATOR_NAME, present=True, timeout_s=0.5):
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

    def _load_until_full(self) -> None:
        # Reconcile the physical barrier before every loading phase, including the
        # third and subsequent iterations seen in the supplied recording.
        self._ensure_gate_open(timeout_s=8.0)
        super()._load_until_full()

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
