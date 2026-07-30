"""Runtime-v17: empty-registry startup and non-recursive emergency handling.

Runtime v16 correctly made the Gazebo JSON registry authoritative, but inherited
``_pose_cache_is_fresh`` treated an empty model dictionary as a missing frame.
At a clean startup the registry is expected to be empty, so slide-gate absence
could never be confirmed even though Gazebo reported that the entity did not
exist.  This runtime separates frame freshness from model count and requires a
new registry sequence after each lifecycle request.
"""

from __future__ import annotations

import threading
import time

import rclpy

from .mechatronics_cycle import KtyMechatronicsCycle
from .mechatronics_cycle_v16 import KtyMechatronicsCycleV16


class KtyMechatronicsCycleV17(KtyMechatronicsCycleV16):
    """V16 operation with correct empty-registry semantics."""

    def __init__(self) -> None:
        self._v17_ready = threading.Event()
        self._registry_empty_confirmations = 0
        self._registry_post_request_confirmations = 0
        self._safe_mechanics_failures = 0
        super().__init__()
        self._v17_ready.set()
        self.get_logger().info(
            "Runtime v17: fresh empty JSON registry is valid; lifecycle checks "
            "require post-request frames"
        )

    def _worker_main(self) -> None:
        self._v17_ready.wait()
        super()._worker_main()

    def _pose_cache_is_fresh(self) -> bool:
        """A received empty registry is fresh and means that no models exist."""
        with self._pose_cache_lock:
            return (
                self._registry_sequence > 0
                and self._pose_cache_received_at > 0.0
                and time.monotonic() - self._pose_cache_received_at
                <= self.pose_cache_timeout
            )

    def _wait_registry_presence(
        self,
        name: str,
        *,
        present: bool,
        timeout_s: float | None = None,
    ) -> bool:
        """Confirm state using registry frames newer than the lifecycle request.

        Requiring a sequence increment avoids accepting an old frame that was
        published immediately before a create / remove request.  Empty frames are
        valid confirmations for ``present=False``.
        """
        timeout = self.static_confirmation_timeout if timeout_s is None else timeout_s
        with self._pose_cache_lock:
            baseline_sequence = self._registry_sequence

        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        last_matching_sequence = baseline_sequence
        while time.monotonic() < deadline:
            self._check_interrupt()
            now = time.monotonic()
            with self._pose_cache_lock:
                sequence = self._registry_sequence
                received_at = self._pose_cache_received_at
                has_model = name in self._pose_cache
                model_count = len(self._pose_cache)

            frame_fresh = (
                sequence > baseline_sequence
                and received_at > 0.0
                and now - received_at <= self.pose_cache_timeout
            )
            matches = frame_fresh and (has_model == present)
            if matches:
                if sequence != last_matching_sequence:
                    last_matching_sequence = sequence
                    if stable_since is None:
                        stable_since = now
                if stable_since is not None and now - stable_since >= self.static_stable_s:
                    self._registry_post_request_confirmations += 1
                    if not present and model_count == 0:
                        self._registry_empty_confirmations += 1
                    return True
            else:
                stable_since = None

            self._interruptible_sleep(0.05)
        return False

    def _safe_mechanics(self) -> None:
        """Stop motion without recursively invoking gate lifecycle operations.

        V16 previously entered ERROR in ``_remove_gate_model`` and then called the
        inherited emergency path, whose gate command invoked the same failing
        lifecycle method a second time.  Directly updating command telemetry keeps
        the station stopped while preserving the original exception for diagnosis.
        """
        self._set_vibration("off")
        try:
            KtyMechatronicsCycle._set_commands(
                self,
                infeed=0.0,
                active=0.0,
                outfeed=0.0,
                pusher=0.0,
                clamps=0.0,
                gate=self.gate_open,
                locator=0.0,
            )
        except Exception as error:  # pragma: no cover - emergency fallback
            self._safe_mechanics_failures += 1
            self.get_logger().error(f"Emergency command reset failed: {error!r}")

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v17",
                "registry_empty_frame_is_valid": True,
                "registry_empty_confirmations": self._registry_empty_confirmations,
                "registry_post_request_confirmations": (
                    self._registry_post_request_confirmations
                ),
                "safe_mechanics_failures": self._safe_mechanics_failures,
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV17()
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
