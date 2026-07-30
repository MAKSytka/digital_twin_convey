"""Runtime-v14 startup recovery for the persistent dynamic-pose bridge."""

from __future__ import annotations

import threading
import time

import rclpy

from .mechatronics_cycle import KtyMechatronicsCycle
from .mechatronics_cycle_v13 import KtyMechatronicsCycleV13


class KtyMechatronicsCycleV14(KtyMechatronicsCycleV13):
    """V13 operation with deadlock-free startup and static lifecycle handling."""

    def __init__(self) -> None:
        self._v14_ready = threading.Event()
        self._startup_complete = False
        self._startup_pose_confirmations = 0
        self._static_lifecycle_retries = 3
        super().__init__()

        self.declare_parameter("spawn_pose_timeout_s", 12.0)
        self.declare_parameter("static_lifecycle_retries", 3)
        self.spawn_pose_timeout = float(
            self.get_parameter("spawn_pose_timeout_s").value
        )
        self._static_lifecycle_retries = int(
            self.get_parameter("static_lifecycle_retries").value
        )
        if self.spawn_pose_timeout <= 0.0:
            raise ValueError("spawn_pose_timeout_s must be positive")
        if self._static_lifecycle_retries < 1:
            raise ValueError("static_lifecycle_retries must be positive")

        self._v14_ready.set()
        self.get_logger().info(
            "Runtime v14: deadlock-free startup on SceneBroadcaster "
            "dynamic_pose/info with idempotent static gate / locator lifecycle"
        )

    def _worker_main(self) -> None:
        self._v14_ready.wait()
        super()._worker_main()

    def _wait_for_services(self, timeout_s: float) -> None:
        """Wait for lifecycle services without requiring a non-empty pose cache."""
        KtyMechatronicsCycle._wait_for_services(self, timeout_s)

    def _wait_for_model_pose(self, name: str, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_interrupt()
            pose = self._read_pose(name)
            if pose is not None and self._pose_cache_is_fresh():
                self._startup_pose_confirmations += 1
                return
            self._interruptible_sleep(0.10)
        with self._pose_cache_lock:
            age = time.monotonic() - self._pose_cache_received_at
            names = sorted(self._pose_cache)[:12]
        raise RuntimeError(
            f"{name} was created but never appeared on {self.POSE_TOPIC}; "
            f"pose_cache_age={age:.2f}s cached={names}"
        )

    def _spawn_kty(self, name: str, x: float) -> None:
        super()._spawn_kty(name, x)
        try:
            self._wait_for_model_pose(name, self.spawn_pose_timeout)
        except Exception:
            self._remove_model(name)
            self._known_models.discard(name)
            raise

    def _remove_static_model_best_effort(self, name: str) -> bool:
        removed = False
        for _ in range(self._static_lifecycle_retries):
            if self._remove_model(name):
                removed = True
                break
            self._interruptible_sleep(0.10)
        return removed

    def _create_static_model_with_retry(
        self,
        name: str,
        sdf: str,
        *,
        x: float,
        y: float,
        z: float,
    ) -> None:
        for attempt in range(1, self._static_lifecycle_retries + 1):
            if self._create_model(name, sdf, x=x, y=y, z=z):
                self._known_models.add(name)
                return
            self._remove_static_model_best_effort(name)
            self._interruptible_sleep(0.15 * attempt)
        raise RuntimeError(
            f"Gazebo rejected static model {name} after "
            f"{self._static_lifecycle_retries} attempts"
        )

    def _spawn_gate_model(self) -> None:
        if self._gate_model_spawned:
            return
        self._create_static_model_with_retry(
            self.GATE_NAME,
            self._gate_sdf(),
            x=self.GATE_X,
            y=self.GATE_Y,
            z=self.GATE_Z,
        )
        self._gate_model_spawned = True
        self.get_logger().info("Slide gate CLOSED (static lifecycle ledger)")

    def _remove_gate_model(self) -> None:
        if self._gate_model_spawned:
            if not self._remove_static_model_best_effort(self.GATE_NAME):
                raise RuntimeError("Gazebo did not remove the slide gate")
        self._gate_model_spawned = False
        self._known_models.discard(self.GATE_NAME)
        self.get_logger().info("Slide gate OPEN (static lifecycle ledger)")

    def _gate_is_absent(self) -> bool:
        return not self._gate_model_spawned

    def _ensure_gate_open(self, timeout_s: float = 5.0) -> None:
        del timeout_s
        self._remove_gate_model()

    def _spawn_locator_model(self) -> None:
        if self._locator_spawned:
            return
        self._create_static_model_with_retry(
            self.LOCATOR_NAME,
            self._locator_sdf(),
            x=self.LOCATOR_X,
            y=self.LOCATOR_Y,
            z=self.LOCATOR_Z,
        )
        self._locator_spawned = True
        self.get_logger().info("Runtime locator UP (static lifecycle ledger)")

    def _remove_locator_model(self) -> None:
        if self._locator_spawned:
            if not self._remove_static_model_best_effort(self.LOCATOR_NAME):
                raise RuntimeError("Gazebo did not remove the runtime locator")
        self._locator_spawned = False
        self._known_models.discard(self.LOCATOR_NAME)
        self.get_logger().info("Runtime locator DOWN (static lifecycle ledger)")

    def _cleanup_stale_models(self) -> None:
        # Dynamic KTY / products are visible in dynamic_pose/info.
        KtyMechatronicsCycle._cleanup_stale_models(self)
        # Static entities are intentionally absent from the dynamic stream.
        self._remove_static_model_best_effort(self.GATE_NAME)
        self._remove_static_model_best_effort(self.LOCATOR_NAME)
        self._gate_model_spawned = False
        self._locator_spawned = False
        self._known_models.discard(self.GATE_NAME)
        self._known_models.discard(self.LOCATOR_NAME)

    def _run(self) -> None:
        self._startup_complete = False
        super()._run()

    def _load_until_full(self) -> None:
        self._startup_complete = True
        super()._load_until_full()

    def _safe_mechanics(self) -> None:
        """Avoid leaving a gate-only world after a pre-KTY startup failure."""
        self._set_vibration("off")
        gate_command = (
            0.0 if self._startup_complete and self._active_kty else self.gate_open
        )
        self._set_commands(
            infeed=0.0,
            active=0.0,
            outfeed=0.0,
            pusher=0.0,
            clamps=0.0,
            gate=gate_command,
            locator=0.0,
        )
        if not self._startup_complete:
            self._remove_locator_model()

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v14",
                "pose_feedback": "scene_broadcaster_dynamic_pose_info",
                "startup_complete": self._startup_complete,
                "startup_pose_confirmations": self._startup_pose_confirmations,
                "static_lifecycle": "service_ledger_not_pose_cache",
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV14()
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
