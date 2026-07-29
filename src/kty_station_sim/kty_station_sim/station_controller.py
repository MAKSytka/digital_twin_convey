"""State machine for the complete KTY station cycle."""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, UInt32
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from singulator_interfaces.msg import (
    KtyFault,
    KtyGroundTruthArray,
    KtyProductContourArray,
    KtyStationState,
)

from .model_factory import make_kty_sdf


class StationController(Node):
    STATE_NAMES = {
        KtyStationState.WAIT_EMPTY_KTY: "WAIT_EMPTY_KTY",
        KtyStationState.POSITION_KTY: "POSITION_KTY",
        KtyStationState.CLAMP: "CLAMP",
        KtyStationState.LOAD: "LOAD",
        KtyStationState.VIBRATE: "VIBRATE",
        KtyStationState.SETTLE: "SETTLE",
        KtyStationState.SCAN: "SCAN",
        KtyStationState.EJECT_PREP: "EJECT_PREP",
        KtyStationState.EJECT: "EJECT",
        KtyStationState.FAULT: "FAULT",
    }

    def __init__(self) -> None:
        super().__init__("station_controller")

        defaults = {
            "world_name": "kty_station",
            "kty_spawn_x_m": -1.30,
            "support_top_z_m": 0.50,
            "approach_speed_mps": 0.65,
            "approach_duration_s": 2.0,
            "positioning_timeout_s": 8.0,
            "position_tolerance_m": 0.08,
            "clamp_duration_s": 0.20,
            "vibration_start_delay_s": 0.50,
            "vibration_frequency_hz": 25.0,
            "vibration_amplitude_m": 0.001,
            "inspection_period_s": 3.0,
            "settle_duration_s": 0.50,
            "scan_timeout_s": 1.50,
            "fill_height_threshold_m": 0.34,
            "eject_preparation_s": 0.50,
            "eject_speed_mps": 0.80,
            "eject_duration_s": 1.0,
            "maximum_kty_mass_kg": 35.0,
            "maximum_cycle_duration_s": 120.0,
            "service_timeout_ms": 5000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.world_name = str(self.get_parameter("world_name").value)
        self.kty_spawn_x = float(self.get_parameter("kty_spawn_x_m").value)
        self.support_top_z = float(self.get_parameter("support_top_z_m").value)
        self.approach_speed = float(self.get_parameter("approach_speed_mps").value)
        self.approach_duration = float(self.get_parameter("approach_duration_s").value)
        self.positioning_timeout = float(
            self.get_parameter("positioning_timeout_s").value
        )
        self.position_tolerance = float(
            self.get_parameter("position_tolerance_m").value
        )
        self.clamp_duration = float(self.get_parameter("clamp_duration_s").value)
        self.vibration_start_delay = float(
            self.get_parameter("vibration_start_delay_s").value
        )
        self.vibration_frequency = float(
            self.get_parameter("vibration_frequency_hz").value
        )
        self.vibration_amplitude = float(
            self.get_parameter("vibration_amplitude_m").value
        )
        self.inspection_period = float(
            self.get_parameter("inspection_period_s").value
        )
        self.settle_duration = float(self.get_parameter("settle_duration_s").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_s").value)
        self.fill_height_threshold = float(
            self.get_parameter("fill_height_threshold_m").value
        )
        self.eject_preparation = float(
            self.get_parameter("eject_preparation_s").value
        )
        self.eject_speed = float(self.get_parameter("eject_speed_mps").value)
        self.eject_duration = float(self.get_parameter("eject_duration_s").value)
        self.maximum_mass = float(
            self.get_parameter("maximum_kty_mass_kg").value
        )
        self.maximum_cycle_duration = float(
            self.get_parameter("maximum_cycle_duration_s").value
        )
        self.service_timeout_ms = int(
            self.get_parameter("service_timeout_ms").value
        )

        if not 20.0 <= self.vibration_frequency <= 50.0:
            raise ValueError("vibration_frequency_hz must be in 20..50 Hz")
        if not 0.0 < self.vibration_amplitude <= 0.003:
            raise ValueError("vibration_amplitude_m must be in (0, 0.003]")

        acceleration_g = (
            self.vibration_amplitude
            * (2.0 * math.pi * self.vibration_frequency) ** 2
            / 9.81
        )
        self.get_logger().warning(
            "Vibration command: %.1f Hz, %.1f mm, peak acceleration %.2f g"
            % (
                self.vibration_frequency,
                self.vibration_amplitude * 1000.0,
                acceleration_g,
            )
        )

        self.infeed_pub = self.create_publisher(
            Float64, "/kty/infeed/cmd_vel", 10
        )
        self.platform_speed_pub = self.create_publisher(
            Float64, "/kty/platform/cmd_vel", 10
        )
        self.outfeed_pub = self.create_publisher(
            Float64, "/kty/outfeed/cmd_vel", 10
        )
        self.platform_position_pub = self.create_publisher(
            Float64, "/kty/platform/cmd_pos", 20
        )
        self.shutter_pub = self.create_publisher(
            Float64, "/kty/shutter/cmd_pos", 10
        )
        self.feed_enable_pub = self.create_publisher(
            Bool, "/kty/product_spawner/enabled", 10
        )
        self.clear_products_pub = self.create_publisher(
            Bool, "/kty/product_spawner/clear", 10
        )
        cycle_qos = QoSProfile(depth=1)
        cycle_qos.reliability = ReliabilityPolicy.RELIABLE
        cycle_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cycle_pub = self.create_publisher(
            UInt32, "/kty/cycle_id", cycle_qos
        )
        self.state_pub = self.create_publisher(
            KtyStationState, "/kty/station/state", 10
        )

        self.create_subscription(
            KtyProductContourArray,
            "/kty/perception/contours",
            self._on_perception,
            10,
        )
        self.create_subscription(
            KtyGroundTruthArray,
            "/kty/ground_truth/registry",
            self._on_ground_truth,
            10,
        )
        self.create_subscription(KtyFault, "/kty/fault", self._on_fault, 10)
        self.create_subscription(TFMessage, "/kty/world/poses", self._on_poses, 20)
        self.create_service(Trigger, "/kty/station/reset", self._on_reset)

        self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="kty_entity")
        self.entity_lock = threading.Lock()
        self.entity_future: Future | None = None

        self.cycle_id = 0
        self.active_kty_name = ""
        self.active_kty_x: float | None = None
        self.state = KtyStationState.WAIT_EMPTY_KTY
        self.state_reason = "startup"
        self.state_started_s = self._now_s()
        self.cycle_started_s = self.state_started_s
        self.vibration_phase_started_s = self.state_started_s
        self.latest_perception: KtyProductContourArray | None = None
        self.scan_start_sequence = 0
        self.estimated_mass = 0.0
        self.fault_latched = False
        self.wait_after_delete_until_s = 0.0

        self.control_timer = self.create_timer(0.02, self._control_step)
        self.vibration_timer = self.create_timer(0.002, self._vibration_step)
        self.state_timer = self.create_timer(0.10, self._publish_state)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _elapsed(self) -> float:
        return self._now_s() - self.state_started_s

    def _transition(self, state: int, reason: str) -> None:
        previous = self.STATE_NAMES.get(self.state, str(self.state))
        current = self.STATE_NAMES.get(state, str(state))
        self.state = state
        self.state_reason = reason
        self.state_started_s = self._now_s()
        if state == KtyStationState.VIBRATE:
            self.vibration_phase_started_s = self.state_started_s
        if state == KtyStationState.SCAN:
            self.scan_start_sequence = (
                self.latest_perception.frame_sequence
                if self.latest_perception is not None
                else 0
            )
        self.get_logger().info(f"{previous} -> {current}: {reason}")

    def _publish_float(self, publisher, value: float) -> None:
        message = Float64()
        message.data = float(value)
        publisher.publish(message)

    def _command_outputs(self) -> None:
        infeed = 0.0
        platform = 0.0
        outfeed = 0.0
        shutter_closed = True
        feed_enabled = False

        if self.state == KtyStationState.POSITION_KTY:
            infeed = self.approach_speed
            platform = self.approach_speed
        elif self.state in (KtyStationState.LOAD, KtyStationState.VIBRATE):
            shutter_closed = False
            feed_enabled = True
        elif self.state == KtyStationState.EJECT:
            platform = self.eject_speed
            outfeed = self.eject_speed

        self._publish_float(self.infeed_pub, infeed)
        self._publish_float(self.platform_speed_pub, platform)
        self._publish_float(self.outfeed_pub, outfeed)
        self._publish_float(self.shutter_pub, 0.0 if shutter_closed else 0.22)

        enabled = Bool()
        enabled.data = feed_enabled
        self.feed_enable_pub.publish(enabled)

    def _vibration_enabled(self) -> bool:
        return self.state == KtyStationState.VIBRATE and not self.fault_latched

    def _vibration_step(self) -> None:
        if self._vibration_enabled():
            phase = 2.0 * math.pi * self.vibration_frequency * (
                self._now_s() - self.vibration_phase_started_s
            )
            position = self.vibration_amplitude * math.sin(phase)
        else:
            position = 0.0
        self._publish_float(self.platform_position_pub, position)

    def _control_step(self) -> None:
        self._command_outputs()
        now = self._now_s()

        if self.fault_latched:
            if self.state != KtyStationState.FAULT:
                self._transition(KtyStationState.FAULT, "critical fault latched")
            return

        if (
            self.state not in (KtyStationState.WAIT_EMPTY_KTY, KtyStationState.FAULT)
            and now - self.cycle_started_s > self.maximum_cycle_duration
        ):
            self.fault_latched = True
            self._transition(KtyStationState.FAULT, "maximum cycle duration exceeded")
            return

        if self.state == KtyStationState.WAIT_EMPTY_KTY:
            if now < self.wait_after_delete_until_s:
                return
            self._start_kty_spawn_if_needed()
            return

        if self.state == KtyStationState.POSITION_KTY:
            if (
                self.active_kty_x is not None
                and abs(self.active_kty_x) <= self.position_tolerance
            ):
                self._transition(
                    KtyStationState.CLAMP,
                    f"KTY centered at x={self.active_kty_x:.3f} m",
                )
            elif self._elapsed() >= self.positioning_timeout:
                self.fault_latched = True
                position = (
                    "unknown"
                    if self.active_kty_x is None
                    else f"{self.active_kty_x:.3f} m"
                )
                self._transition(
                    KtyStationState.FAULT,
                    f"KTY positioning timeout; x={position}",
                )
            return

        if self.state == KtyStationState.CLAMP:
            if self._elapsed() >= self.clamp_duration:
                self._transition(KtyStationState.LOAD, "side guides engaged")
            return

        if self.state == KtyStationState.LOAD:
            if self._elapsed() >= self.vibration_start_delay:
                self._transition(KtyStationState.VIBRATE, "vibration start delay elapsed")
            return

        if self.state == KtyStationState.VIBRATE:
            if self.estimated_mass >= self.maximum_mass:
                self._transition(KtyStationState.EJECT_PREP, "mass limit reached")
            elif self._elapsed() >= self.inspection_period:
                self._transition(KtyStationState.SETTLE, "periodic depth inspection")
            return

        if self.state == KtyStationState.SETTLE:
            if self._elapsed() >= self.settle_duration:
                self._transition(KtyStationState.SCAN, "micro-pause complete")
            return

        if self.state == KtyStationState.SCAN:
            if self._new_valid_scan_available():
                assert self.latest_perception is not None
                height = self.latest_perception.maximum_height_m
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
                self.fault_latched = True
                self._transition(KtyStationState.FAULT, "camera scan timeout")
            return

        if self.state == KtyStationState.EJECT_PREP:
            if self._elapsed() >= self.eject_preparation:
                self._transition(KtyStationState.EJECT, "vibration stopped before eject")
            return

        if self.state == KtyStationState.EJECT:
            if self._elapsed() >= self.eject_duration:
                self._finish_cycle()

    def _new_valid_scan_available(self) -> bool:
        return (
            self.latest_perception is not None
            and self.latest_perception.camera_ok
            and self.latest_perception.frame_sequence > self.scan_start_sequence
        )

    def _start_kty_spawn_if_needed(self) -> None:
        with self.entity_lock:
            if self.entity_future is not None:
                if not self.entity_future.done():
                    return
                try:
                    success, model_name = self.entity_future.result()
                except Exception as error:  # pragma: no cover - runtime guard
                    self.get_logger().error(f"KTY spawn exception: {error}")
                    success, model_name = False, ""
                self.entity_future = None
                if not success:
                    self.fault_latched = True
                    self._transition(KtyStationState.FAULT, "KTY spawn failed")
                    return
                self.active_kty_name = model_name
                self.active_kty_x = None
                self.cycle_started_s = self._now_s()
                self._transition(KtyStationState.POSITION_KTY, "empty KTY spawned")
                return

            self.cycle_id += 1
            model_name = f"kty_{self.cycle_id:06d}"
            cycle_message = UInt32()
            cycle_message.data = self.cycle_id
            self.cycle_pub.publish(cycle_message)
            self.entity_future = self.pool.submit(self._spawn_kty, model_name)

    def _spawn_kty(self, model_name: str) -> tuple[bool, str]:
        sdf = make_kty_sdf(model_name)
        request = "\n".join(
            (
                f"sdf: {json.dumps(sdf)}",
                f'name: "{model_name}"',
                "allow_renaming: false",
                "pose {",
                f"  position {{ x: {self.kty_spawn_x:.9f} y: 0 z: {self.support_top_z:.9f} }}",
                "}",
            )
        )
        service = f"/world/{self.world_name}/create"
        command = [
            "gz", "service", "-s", service,
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(self.service_timeout_ms),
            "--req", request,
        ]
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=self.service_timeout_ms / 1000.0 + 2.0,
            check=False,
        )
        success = result.returncode == 0 and "data: true" in result.stdout.lower()
        if not success:
            self.get_logger().error(
                f"Failed to spawn {model_name}: {result.stdout} {result.stderr}"
            )
        return success, model_name

    def _remove_model(self, model_name: str) -> bool:
        service = f"/world/{self.world_name}/remove"
        request = f'name: "{model_name}" type: MODEL'
        command = [
            "gz", "service", "-s", service,
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", str(self.service_timeout_ms),
            "--req", request,
        ]
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=self.service_timeout_ms / 1000.0 + 2.0,
            check=False,
        )
        return result.returncode == 0 and "data: true" in result.stdout.lower()

    def _finish_cycle(self) -> None:
        clear = Bool()
        clear.data = True
        self.clear_products_pub.publish(clear)
        if self.active_kty_name:
            self.pool.submit(self._remove_model, self.active_kty_name)
        self.active_kty_name = ""
        self.active_kty_x = None
        self.estimated_mass = 0.0
        self.latest_perception = None
        self.wait_after_delete_until_s = self._now_s() + 0.5
        self._transition(KtyStationState.WAIT_EMPTY_KTY, "KTY handed to outfeed and despawned")

    def _on_perception(self, message: KtyProductContourArray) -> None:
        self.latest_perception = message

    def _on_poses(self, message: TFMessage) -> None:
        if not self.active_kty_name:
            return
        for transform in message.transforms:
            if self.active_kty_name in transform.child_frame_id:
                self.active_kty_x = float(transform.transform.translation.x)
                return

    def _on_ground_truth(self, message: KtyGroundTruthArray) -> None:
        if message.cycle_id != self.cycle_id:
            return
        self.estimated_mass = sum(float(item.mass_kg) for item in message.products)

    def _on_fault(self, message: KtyFault) -> None:
        if not message.active or message.cycle_id not in (0, self.cycle_id):
            return
        self.get_logger().error(f"Fault {message.code}: {message.details}")
        if message.severity >= KtyFault.CRITICAL:
            self.fault_latched = True

    def _on_reset(self, request, response):
        del request
        self.fault_latched = False
        self.estimated_mass = 0.0
        clear = Bool()
        clear.data = True
        self.clear_products_pub.publish(clear)
        if self.active_kty_name:
            self.pool.submit(self._remove_model, self.active_kty_name)
            self.active_kty_name = ""
            self.active_kty_x = None
        self.wait_after_delete_until_s = self._now_s() + 0.5
        self._transition(KtyStationState.WAIT_EMPTY_KTY, "manual reset")
        response.success = True
        response.message = "KTY station reset"
        return response

    def _publish_state(self) -> None:
        message = KtyStationState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "kty_station"
        message.cycle_id = self.cycle_id
        message.state = self.state
        message.state_name = self.STATE_NAMES.get(self.state, "UNKNOWN")
        message.reason = self.state_reason
        message.kty_expected = bool(self.active_kty_name)
        message.shutter_closed = self.state not in (
            KtyStationState.LOAD,
            KtyStationState.VIBRATE,
        )
        message.vibration_enabled = self._vibration_enabled()
        message.product_feed_enabled = self.state in (
            KtyStationState.LOAD,
            KtyStationState.VIBRATE,
        )
        message.vibration_frequency_hz = self.vibration_frequency
        message.vibration_amplitude_m = self.vibration_amplitude
        message.measured_maximum_height_m = (
            self.latest_perception.maximum_height_m
            if self.latest_perception is not None
            else 0.0
        )
        message.fill_height_threshold_m = self.fill_height_threshold
        message.estimated_mass_kg = self.estimated_mass
        self.state_pub.publish(message)

    def close(self) -> None:
        self._command_outputs()
        self._publish_float(self.platform_position_pub, 0.0)
        self.pool.shutdown(wait=False, cancel_futures=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StationController()
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
