"""Runtime-v12 guards for repeatable loading and second-KTY positioning.

The accepted v11 mechanics are retained. Runtime v12 fixes two target-machine
failures:

* a sparse transient depth spike could close the chute after one carton;
* POSITION_NEXT used an eight-second wall-clock timeout, which is too short at
  low Gazebo real-time factors.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from std_msgs.msg import String

from .mechatronics_cycle_v11 import KtyMechatronicsCycleV11


class KtyMechatronicsCycleV12(KtyMechatronicsCycleV11):
    """V11 vibration / despawn with robust fill and queued-KTY admission."""

    def __init__(self) -> None:
        self._v12_ready = threading.Event()
        self._fill_received_at = 0.0
        self._load_guard = {
            "elapsed_s": 0.0,
            "spawned_products": 0,
            "volume_reached": False,
            "height_reached": False,
            "occupied_floor_ratio": 0.0,
            "close_reason": "waiting",
        }
        self._position_recovery_pulses = 0
        super().__init__()

        defaults = {
            "minimum_load_duration_s": 4.0,
            "minimum_products_for_close": 3,
            "height_guard_min_fill_ratio": 0.10,
            "height_guard_min_occupied_ratio": 0.18,
            "volume_guard_min_occupied_ratio": 0.35,
            "load_close_persistence_s": 1.0,
            "position_next_timeout_s": 60.0,
            "position_progress_timeout_s": 7.0,
            "readiness_timeout_s": 30.0,
            "readiness_position_tolerance_m": 0.012,
            "readiness_velocity_tolerance_mps": 0.040,
            "empty_kty_fill_limit": 0.22,
            "empty_kty_occupied_limit": 0.30,
            "fill_freshness_timeout_s": 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        read = lambda name: self.get_parameter(name).value
        self.minimum_load_duration = float(read("minimum_load_duration_s"))
        self.minimum_products_for_close = int(read("minimum_products_for_close"))
        self.height_guard_fill = float(read("height_guard_min_fill_ratio"))
        self.height_guard_occupied = float(read("height_guard_min_occupied_ratio"))
        self.volume_guard_occupied = float(read("volume_guard_min_occupied_ratio"))
        self.load_close_persistence = float(read("load_close_persistence_s"))
        self.position_next_timeout = float(read("position_next_timeout_s"))
        self.position_progress_timeout = float(read("position_progress_timeout_s"))
        self.readiness_timeout = float(read("readiness_timeout_s"))
        self.readiness_position_tolerance = float(
            read("readiness_position_tolerance_m")
        )
        self.readiness_velocity_tolerance = float(
            read("readiness_velocity_tolerance_mps")
        )
        self.empty_kty_fill_limit = float(read("empty_kty_fill_limit"))
        self.empty_kty_occupied_limit = float(read("empty_kty_occupied_limit"))
        self.fill_freshness_timeout = float(read("fill_freshness_timeout_s"))

        if self.minimum_products_for_close < 1:
            raise ValueError("minimum_products_for_close must be positive")
        if self.minimum_load_duration < 0.0:
            raise ValueError("minimum_load_duration_s cannot be negative")

        self._v12_ready.set()
        self.get_logger().info(
            "Runtime v12: guarded fill thresholds, progress-aware POSITION_NEXT "
            "and confirmed gate reopening"
        )

    def _worker_main(self) -> None:
        self._v12_ready.wait()
        super()._worker_main()

    def _on_fill_state(self, message: String) -> None:
        super()._on_fill_state(message)
        self._fill_received_at = time.monotonic()

    def _gate_is_absent(self) -> bool:
        return self.GATE_NAME not in self._read_world_poses()

    def _ensure_gate_open(self, timeout_s: float = 5.0) -> None:
        """Remove the static gate and confirm absence before entering LOAD."""
        deadline = time.monotonic() + timeout_s
        attempts = 0
        while time.monotonic() < deadline:
            self._check_interrupt()
            if self._gate_is_absent():
                self._gate_model_spawned = False
                return
            attempts += 1
            self._gate_model_spawned = True
            super()._remove_gate_model()
            self._interruptible_sleep(0.15)
        raise RuntimeError(
            f"slide gate remained present after {attempts} removal attempts"
        )

    def _load_until_full(self) -> None:
        # Reassert physical opening at every cycle. This also recovers a gate whose
        # first remove request was accepted late by Gazebo.
        self._set_commands(gate=self.gate_open)
        self._ensure_gate_open()
        self._transition(
            "LOAD",
            "gate confirmed open; guarded volume/height loading with weak vibration",
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

        while True:
            self._check_interrupt()
            now = time.monotonic()
            if now >= next_spawn:
                name = self._spawn_product()
                self._active_product_names.add(name)
                spawned_products += 1
                next_spawn = now + self.spawn_interval

            with self._lock:
                fill = dict(self._latest_fill)

            elapsed = now - started
            fill_ratio = float(fill.get("fill_ratio", 0.0) or 0.0)
            maximum_height = float(fill.get("maximum_height_m", 0.0) or 0.0)
            occupied_ratio = float(fill.get("occupied_floor_ratio", 0.0) or 0.0)
            camera_ok = bool(fill.get("camera_ok", False))
            fill_fresh = now - self._fill_received_at <= self.fill_freshness_timeout
            enough_products = spawned_products >= self.minimum_products_for_close
            enough_time = elapsed >= self.minimum_load_duration

            # Both closure paths require spatial support. A single tall carton or a
            # thin chute / wall artefact can no longer close an almost empty KTY.
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

            reason = "waiting"
            if volume_reached:
                reason = "volume"
            elif height_reached:
                reason = "height_with_support"
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
            }

            if guarded_reached:
                if threshold_since is None:
                    threshold_since = now
                elif now - threshold_since >= max(
                    self.fill_persistence,
                    self.load_close_persistence,
                ):
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

    def _approach_locator(self, name: str, timeout_s: float) -> None:
        """Move the queued KTY with a wall-time budget compatible with low RTF."""
        deadline = time.monotonic() + max(timeout_s, self.position_next_timeout)
        best_x = -math.inf
        last_progress_at = time.monotonic()
        recovery_until = 0.0
        pusher_retracted = False

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

            # The pusher only starts the KTY. It retracts before the contact
            # surfaces perform final positioning, so it cannot drag or wedge it.
            if not pusher_retracted and pose.x >= self.queue_spawn_x + 0.28:
                pusher_retracted = True
            pusher = 0.0 if pusher_retracted else self.pusher_extended

            if now - last_progress_at >= self.position_progress_timeout:
                self._position_recovery_pulses += 1
                recovery_until = now + 2.0
                last_progress_at = now
                self.get_logger().warning(
                    f"{name} positioning stalled at x={pose.x:.3f}; "
                    f"surface recovery pulse {self._position_recovery_pulses}"
                )

            if now < recovery_until:
                speed = max(speed, min(0.85, 1.25 * self.roller_speed))
                pusher = self.pusher_extended

            self._set_commands(
                pusher=pusher,
                infeed=speed,
                active=speed,
                outfeed=0.0,
            )
            self._interruptible_sleep(0.20)

        pose = self._read_pose(name)
        x_text = "unknown" if pose is None else f"{pose.x:.3f}"
        best_text = "unknown" if not math.isfinite(best_x) else f"{best_x:.3f}"
        raise RuntimeError(
            f"{name} did not reach active locator within "
            f"{self.position_next_timeout:.0f} s; last x={x_text}, best x={best_text}"
        )

    def _wait_until_ready(
        self,
        name: str,
        previous_name: str,
        timeout_s: float,
    ) -> None:
        """Verify mechanics and a fresh empty-KTY depth frame before opening."""
        deadline = time.monotonic() + max(timeout_s, self.readiness_timeout)
        stable_since: float | None = None
        last_pose = None
        last_time: float | None = None
        diagnostics = {}

        while time.monotonic() < deadline:
            self._check_interrupt()
            now = time.monotonic()
            poses = self._read_world_poses()
            pose = poses.get(name)
            previous = poses.get(previous_name)
            if pose is None:
                stable_since = None
                self._interruptible_sleep(0.25)
                continue

            velocity = math.inf
            if last_pose is not None and last_time is not None and now > last_time:
                velocity = abs(pose.x - last_pose.x) / (now - last_time)
            last_pose = pose
            last_time = now

            with self._lock:
                fill = dict(self._latest_fill)
                clamps_closed = self._commands["clamps"] > 0.5 * self.clamp_closed

            fill_age = now - self._fill_received_at
            camera_ok = bool(fill.get("camera_ok", False))
            fill_ratio = float(fill.get("fill_ratio", 1.0) or 0.0)
            occupied_ratio = float(fill.get("occupied_floor_ratio", 1.0) or 0.0)
            position_ok = (
                abs(pose.x - self.active_target_x)
                <= max(self.position_tolerance, self.readiness_position_tolerance)
            )
            velocity_ok = velocity <= max(
                self.velocity_tolerance,
                self.readiness_velocity_tolerance,
            )
            previous_clear = previous is None
            locator_ok = self._locator_spawned
            camera_fresh = fill_age <= self.fill_freshness_timeout
            empty_ok = (
                fill_ratio <= self.empty_kty_fill_limit
                and occupied_ratio <= self.empty_kty_occupied_limit
            )

            ready = (
                position_ok
                and velocity_ok
                and previous_clear
                and locator_ok
                and clamps_closed
                and camera_ok
                and camera_fresh
                and empty_ok
            )
            diagnostics = {
                "x": pose.x,
                "velocity": velocity,
                "position_ok": position_ok,
                "velocity_ok": velocity_ok,
                "previous_clear": previous_clear,
                "locator_ok": locator_ok,
                "clamps_closed": clamps_closed,
                "camera_ok": camera_ok,
                "camera_fresh": camera_fresh,
                "fill_ratio": fill_ratio,
                "occupied_ratio": occupied_ratio,
                "empty_ok": empty_ok,
            }

            if ready:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= max(self.ready_persistence, 0.50):
                    return
            else:
                stable_since = None
            self._interruptible_sleep(0.25)

        raise RuntimeError(
            f"{name} failed readiness checks after {self.readiness_timeout:.0f} s: "
            f"{diagnostics}"
        )

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v12",
                "load_threshold_policy": "guarded_volume_or_supported_height",
                "load_guard": dict(self._load_guard),
                "position_next_timeout_s": self.position_next_timeout,
                "position_recovery_pulses": self._position_recovery_pulses,
                # This flag is only set false after a confirmed removal attempt;
                # avoid spawning a blocking gz subprocess from the state timer.
                "gate_open_confirmed": not self._gate_model_spawned,
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV12()
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
