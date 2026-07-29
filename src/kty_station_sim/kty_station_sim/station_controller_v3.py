"""KTY station controller with deterministic Gazebo pose transport.

The previous runtime tried to move the KTY through a dynamically attached
VelocityControl plugin and a ROS -> Gazebo Twist bridge.  In practice the KTY
could remain at the infeed while pose feedback was still alive, which kept the
state machine in POSITION_KTY until a fault.  This version uses the
UserCommands /world/<world>/set_pose service for the short infeed and outfeed
motions.  Between those motions the KTY is a normal dynamic body, so gravity,
products and the vibrating table act on it physically.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import math
import subprocess

from std_msgs.msg import Bool

from singulator_interfaces.msg import KtyStationState

from .station_controller_v2 import StationControllerV2


class StationControllerV3(StationControllerV2):
    """Full station cycle with pause-aware, pose-driven KTY transport."""

    def __init__(self) -> None:
        super().__init__()

        extra_defaults = {
            "transport_update_period_s": 0.05,
            "transport_position_tolerance_m": 0.005,
            "transport_failure_limit": 8,
            "world_reset_jump_threshold_s": 0.10,
        }
        for name, value in extra_defaults.items():
            self.declare_parameter(name, value)

        self.transport_update_period = float(
            self.get_parameter("transport_update_period_s").value
        )
        self.transport_position_tolerance = float(
            self.get_parameter("transport_position_tolerance_m").value
        )
        self.transport_failure_limit = int(
            self.get_parameter("transport_failure_limit").value
        )
        self.world_reset_jump_threshold = float(
            self.get_parameter("world_reset_jump_threshold_s").value
        )

        if self.transport_update_period <= 0.0:
            raise ValueError("transport_update_period_s must be positive")
        if self.transport_position_tolerance <= 0.0:
            raise ValueError("transport_position_tolerance_m must be positive")
        if self.transport_failure_limit < 1:
            raise ValueError("transport_failure_limit must be at least one")

        self.transport_pool = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="kty_pose_transport",
        )
        self.transport_future: Future | None = None
        self.transport_requested_x: float | None = None
        self.transport_successful_x: float | None = None
        self.transport_failures = 0
        self.last_transport_request_s = -math.inf
        self.last_control_time_s = self._now_s()

        self.get_logger().warning(
            "KTY runtime v3 uses /world/%s/set_pose for infeed and outfeed"
            % self.world_name
        )

    def _transition(self, state: int, reason: str) -> None:
        super()._transition(state, reason)
        if state in (KtyStationState.POSITION_KTY, KtyStationState.EJECT):
            self._reset_transport_progress()

    def _reset_transport_progress(self) -> None:
        self.transport_successful_x = None
        self.transport_failures = 0
        self.last_transport_request_s = -math.inf

    def _command_outputs(self) -> None:
        """Command contact surfaces, shutter and feeder.

        KTY translation is deliberately absent here.  The authoritative
        transport actuator is the Gazebo set_pose service.  This removes the
        critical dependency on a ROS Twist bridge and avoids a VelocityControl
        plugin freezing the KTY during physical vibration.
        """

        infeed = 0.0
        platform_surface = 0.0
        outfeed = 0.0
        shutter_closed = True
        feed_enabled = False

        if self.state == KtyStationState.POSITION_KTY:
            infeed = self.approach_speed
            platform_surface = self.approach_speed
        elif self.state in (KtyStationState.LOAD, KtyStationState.VIBRATE):
            shutter_closed = False
            feed_enabled = True
        elif self.state == KtyStationState.EJECT:
            platform_surface = self.eject_speed
            outfeed = self.eject_speed

        self._publish_float(
            self.infeed_pub,
            self.surface_command_sign * infeed,
        )
        self._publish_float(
            self.platform_speed_pub,
            self.surface_command_sign * platform_surface,
        )
        self._publish_float(
            self.outfeed_pub,
            self.surface_command_sign * outfeed,
        )
        self._publish_float(
            self.shutter_pub,
            0.0 if shutter_closed else 0.22,
        )

        enabled = Bool()
        enabled.data = feed_enabled
        self.feed_enable_pub.publish(enabled)

    def _request_model_pose(
        self,
        model_name: str,
        x: float,
        y: float,
        z: float,
    ) -> bool:
        request = (
            f'name: "{model_name}" '
            f'position {{ x: {x:.9f} y: {y:.9f} z: {z:.9f} }} '
            "orientation { x: 0 y: 0 z: 0 w: 1 }"
        )
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
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
            self.get_logger().error(
                f"set_pose failed for {model_name} at x={x:.3f}: {error}"
            )
            return False

        success = result.returncode == 0 and "data: true" in result.stdout.lower()
        if not success:
            self.get_logger().error(
                f"set_pose rejected for {model_name} at x={x:.3f}: "
                f"stdout={result.stdout!r} stderr={result.stderr!r}"
            )
        return success

    def _consume_transport_result(self) -> None:
        if self.transport_future is None or not self.transport_future.done():
            return

        requested_x = self.transport_requested_x
        try:
            success = bool(self.transport_future.result())
        except Exception as error:  # pragma: no cover - runtime guard
            self.get_logger().error(f"KTY pose transport exception: {error}")
            success = False

        self.transport_future = None
        self.transport_requested_x = None

        if success and requested_x is not None:
            self.transport_successful_x = requested_x
            self.transport_failures = 0
        else:
            self.transport_failures += 1

    def _schedule_transport_pose(self, x: float) -> None:
        self._consume_transport_result()
        if self.transport_future is not None:
            return

        now = self._now_s()
        if now - self.last_transport_request_s < self.transport_update_period:
            return
        if not self.active_kty_name:
            return

        self.last_transport_request_s = now
        self.transport_requested_x = x
        self.transport_future = self.transport_pool.submit(
            self._request_model_pose,
            self.active_kty_name,
            x,
            0.0,
            self.support_top_z,
        )

    def _drive_transport(
        self,
        start_x: float,
        target_x: float,
        duration_s: float,
    ) -> bool:
        if duration_s <= 0.0:
            raise ValueError("transport duration must be positive")

        progress = min(1.0, max(0.0, self._elapsed() / duration_s))
        desired_x = start_x + (target_x - start_x) * progress
        self._schedule_transport_pose(desired_x)
        self._consume_transport_result()

        if self.transport_failures >= self.transport_failure_limit:
            self.fault_latched = True
            self._transition(
                KtyStationState.FAULT,
                f"set_pose failed {self.transport_failures} consecutive times",
            )
            return False

        return (
            progress >= 1.0
            and self.transport_successful_x is not None
            and abs(self.transport_successful_x - target_x)
            <= self.transport_position_tolerance
        )

    def _recover_from_world_reset(self) -> None:
        """Synchronise ROS state after the Gazebo reset button resets time."""

        self.get_logger().warning(
            "Gazebo simulation time moved backwards; restarting the KTY cycle"
        )

        clear = Bool()
        clear.data = True
        self.clear_products_pub.publish(clear)

        if self.entity_future is not None:
            self.entity_future.cancel()
            self.entity_future = None
        if self.transport_future is not None:
            self.transport_future.cancel()
            self.transport_future = None

        self.active_kty_name = ""
        self.active_kty_x = None
        self.active_kty_y = None
        self.active_kty_z = None
        self.last_pose_s = -math.inf
        self.estimated_mass = 0.0
        self.latest_perception = None
        self.scan_failures = 0
        self.fault_latched = False
        self._reset_transport_progress()
        self.wait_after_delete_until_s = self._now_s() + 0.25
        self._transition(
            KtyStationState.WAIT_EMPTY_KTY,
            "Gazebo world reset detected",
        )

    def _control_step(self) -> None:
        now = self._now_s()
        if now + self.world_reset_jump_threshold < self.last_control_time_s:
            self._recover_from_world_reset()
            now = self._now_s()
        self.last_control_time_s = now

        self._command_outputs()

        if self.fault_latched:
            if self.state != KtyStationState.FAULT:
                self._transition(KtyStationState.FAULT, "critical fault latched")
            return

        if (
            self.state
            not in (KtyStationState.WAIT_EMPTY_KTY, KtyStationState.FAULT)
            and now - self.cycle_started_s > self.maximum_cycle_duration
        ):
            self.fault_latched = True
            self._transition(
                KtyStationState.FAULT,
                "maximum cycle duration exceeded",
            )
            return

        if self.state == KtyStationState.WAIT_EMPTY_KTY:
            if now >= self.wait_after_delete_until_s:
                self._start_kty_spawn_if_needed()
            return

        if self.state == KtyStationState.POSITION_KTY:
            complete = self._drive_transport(
                self.kty_spawn_x,
                0.0,
                self.approach_duration,
            )
            if complete:
                self.active_kty_x = 0.0
                self._transition(
                    KtyStationState.CLAMP,
                    "KTY moved to platform by set_pose trajectory",
                )
            elif self._elapsed() >= self.positioning_timeout:
                self.fault_latched = True
                self._transition(
                    KtyStationState.FAULT,
                    "KTY set_pose positioning timeout",
                )
            return

        if self.state == KtyStationState.CLAMP:
            if self._elapsed() >= self.clamp_duration:
                self._transition(KtyStationState.LOAD, "side guides engaged")
            return

        if self.state == KtyStationState.LOAD:
            if self._elapsed() >= self.vibration_start_delay:
                self._transition(
                    KtyStationState.VIBRATE,
                    "vibration start delay elapsed",
                )
            return

        if self.state == KtyStationState.VIBRATE:
            if self.estimated_mass >= self.maximum_mass:
                self._transition(KtyStationState.EJECT_PREP, "mass limit reached")
            elif self._elapsed() >= self.inspection_period:
                self._transition(
                    KtyStationState.SETTLE,
                    "periodic depth inspection",
                )
            return

        if self.state == KtyStationState.SETTLE:
            if self._elapsed() >= self.settle_duration:
                self._transition(KtyStationState.SCAN, "micro-pause complete")
            return

        if self.state == KtyStationState.SCAN:
            if self._new_valid_scan_available():
                assert self.latest_perception is not None
                self.scan_failures = 0
                height = float(self.latest_perception.maximum_height_m)
                if height >= self.fill_height_threshold:
                    self._transition(
                        KtyStationState.EJECT_PREP,
                        f"height limit reached: {height:.3f} m",
                    )
                else:
                    self._transition(
                        KtyStationState.VIBRATE,
                        f"height {height:.3f} m below limit",
                    )
            elif self._elapsed() >= self.scan_timeout:
                self.scan_failures += 1
                if self.scan_failures >= self.scan_failures_before_fault:
                    self.fault_latched = True
                    self._transition(
                        KtyStationState.FAULT,
                        f"camera scan timeout ({self.scan_failures} consecutive checks)",
                    )
                else:
                    self._transition(
                        KtyStationState.VIBRATE,
                        f"camera frame missed; retry {self.scan_failures}/"
                        f"{self.scan_failures_before_fault}",
                    )
            return

        if self.state == KtyStationState.EJECT_PREP:
            if self._elapsed() >= self.eject_preparation:
                self._transition(
                    KtyStationState.EJECT,
                    "vibration stopped before eject",
                )
            return

        if self.state == KtyStationState.EJECT:
            target_x = self.eject_speed * self.eject_duration
            complete = self._drive_transport(
                0.0,
                target_x,
                self.eject_duration,
            )
            if complete:
                self.active_kty_x = target_x
                self._finish_cycle()
            elif self._elapsed() >= self.eject_duration + 3.0:
                self.fault_latched = True
                self._transition(
                    KtyStationState.FAULT,
                    "KTY set_pose eject timeout",
                )

    def _finish_cycle(self) -> None:
        self._reset_transport_progress()
        super()._finish_cycle()

    def _on_reset(self, request, response):
        if self.transport_future is not None:
            self.transport_future.cancel()
            self.transport_future = None
        self._reset_transport_progress()
        return super()._on_reset(request, response)

    def close(self) -> None:
        if self.transport_future is not None:
            self.transport_future.cancel()
        self.transport_pool.shutdown(wait=False, cancel_futures=True)
        super().close()


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = StationControllerV3()
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
