"""Fault detection for the KTY simulation.

Pose feedback and RGB-D perception are diagnostic inputs, not transport
actuators.  Missing optional diagnostics must not stop the carrier before the
controller has had a chance to perform its configured retries.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from singulator_interfaces.msg import (
    KtyFault,
    KtyGroundTruthArray,
    KtyProductContourArray,
    KtyStationState,
)


PRODUCT_PATTERN = re.compile(r"(kty_product_c\d+_p\d+)")
KTY_PATTERN = re.compile(r"(kty_\d{6})")


@dataclass(slots=True)
class PoseSample:
    x: float
    y: float
    z: float
    stamp_s: float
    speed: float = 0.0


class KtySafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("kty_safety_monitor")
        defaults = {
            "kty_center_tolerance_m": 0.16,
            "kty_presence_timeout_s": 0.8,
            "pose_stream_timeout_s": 1.5,
            "camera_timeout_s": 1.0,
            "maximum_mass_kg": 35.0,
            "chute_jam_timeout_s": 2.5,
            "still_moving_speed_mps": 0.03,
            "still_moving_timeout_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.center_tolerance = float(
            self.get_parameter("kty_center_tolerance_m").value
        )
        self.kty_presence_timeout = float(
            self.get_parameter("kty_presence_timeout_s").value
        )
        self.pose_stream_timeout = float(
            self.get_parameter("pose_stream_timeout_s").value
        )
        self.camera_timeout = float(self.get_parameter("camera_timeout_s").value)
        self.maximum_mass = float(self.get_parameter("maximum_mass_kg").value)
        self.chute_jam_timeout = float(
            self.get_parameter("chute_jam_timeout_s").value
        )
        self.still_speed = float(
            self.get_parameter("still_moving_speed_mps").value
        )
        self.still_timeout = float(
            self.get_parameter("still_moving_timeout_s").value
        )

        self.state: KtyStationState | None = None
        self.poses: dict[str, PoseSample] = {}
        self.products = {}
        self.last_pose_message_s = -math.inf
        self.last_camera_s = -math.inf
        self.last_camera_ok = False
        self.stationary_since: dict[str, float] = {}
        self.moving_since: dict[str, float] = {}
        self.fault_states: dict[str, bool] = {}
        self.pose_warning_emitted = False

        self.fault_pub = self.create_publisher(KtyFault, "/kty/fault", 10)
        self.create_subscription(TFMessage, "/kty/world/poses", self._on_poses, 20)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            KtyStationState,
            "/kty/station/state",
            self._on_state,
            transient_qos,
        )
        self.create_subscription(
            KtyGroundTruthArray,
            "/kty/ground_truth/registry",
            self._on_registry,
            transient_qos,
        )
        self.create_subscription(
            KtyProductContourArray,
            "/kty/perception/contours",
            self._on_perception,
            10,
        )
        self.timer = self.create_timer(0.10, self._evaluate)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _normalise_name(frame: str) -> str | None:
        product = PRODUCT_PATTERN.search(frame)
        if product:
            return product.group(1)
        kty = KTY_PATTERN.search(frame)
        if kty:
            return kty.group(1)
        return None

    def _on_poses(self, message: TFMessage) -> None:
        now = self._now_s()
        self.last_pose_message_s = now
        for transform in message.transforms:
            # Depending on the bridge version the scoped model name can occur
            # in either frame field.
            name = self._normalise_name(transform.child_frame_id)
            if name is None:
                name = self._normalise_name(transform.header.frame_id)
            if name is None:
                continue
            translation = transform.transform.translation
            previous = self.poses.get(name)
            speed = 0.0
            if previous is not None:
                dt = now - previous.stamp_s
                if dt > 1.0e-4:
                    speed = math.sqrt(
                        (translation.x - previous.x) ** 2
                        + (translation.y - previous.y) ** 2
                        + (translation.z - previous.z) ** 2
                    ) / dt
            self.poses[name] = PoseSample(
                x=float(translation.x),
                y=float(translation.y),
                z=float(translation.z),
                stamp_s=now,
                speed=speed,
            )

    def _on_state(self, message: KtyStationState) -> None:
        self.state = message

    def _on_registry(self, message: KtyGroundTruthArray) -> None:
        if self.state is not None and message.cycle_id != self.state.cycle_id:
            return
        self.products = {item.model_name: item for item in message.products}

    def _on_perception(self, message: KtyProductContourArray) -> None:
        self.last_camera_s = self._now_s()
        self.last_camera_ok = bool(message.camera_ok)

    def _publish_fault(
        self,
        code: str,
        active: bool,
        severity: int,
        details: str,
    ) -> None:
        previous = self.fault_states.get(code, False)
        if previous == active:
            return
        self.fault_states[code] = active
        message = KtyFault()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "kty_station"
        message.cycle_id = self.state.cycle_id if self.state is not None else 0
        message.severity = severity
        message.code = code
        message.details = details
        message.active = active
        self.fault_pub.publish(message)

    def _active_loading_state(self) -> bool:
        return self.state is not None and self.state.state in (
            KtyStationState.CLAMP,
            KtyStationState.LOAD,
            KtyStationState.VIBRATE,
            KtyStationState.SETTLE,
            KtyStationState.SCAN,
            KtyStationState.EJECT_PREP,
        )

    def _current_kty_pose(self) -> PoseSample | None:
        if self.state is None:
            return None
        return self.poses.get(f"kty_{self.state.cycle_id:06d}")

    def _evaluate(self) -> None:
        if self.state is None:
            return

        now = self._now_s()
        active = self._active_loading_state()
        pose_stream_alive = (
            now - self.last_pose_message_s <= self.pose_stream_timeout
        )

        if active and not pose_stream_alive and not self.pose_warning_emitted:
            self.get_logger().warning(
                "Gazebo pose feedback is unavailable; pose-dependent safety "
                "checks are temporarily disabled. Carrier transport continues."
            )
            self.pose_warning_emitted = True
        elif pose_stream_alive:
            self.pose_warning_emitted = False

        kty_pose = self._current_kty_pose()
        missing_or_misplaced = False
        if active and pose_stream_alive and self.state.state != KtyStationState.CLAMP:
            missing_or_misplaced = (
                kty_pose is None
                or now - kty_pose.stamp_s > self.kty_presence_timeout
                or abs(kty_pose.x) > self.center_tolerance
                or abs(kty_pose.y) > self.center_tolerance
            )
        self._publish_fault(
            KtyFault.KTY_NOT_INSTALLED,
            missing_or_misplaced,
            KtyFault.CRITICAL,
            "KTY pose is missing or outside the active platform tolerance",
        )

        total_mass = sum(float(item.mass_kg) for item in self.products.values())
        self._publish_fault(
            KtyFault.MASS_EXCEEDED,
            active and total_mass > self.maximum_mass,
            KtyFault.CRITICAL,
            f"Estimated product mass {total_mass:.3f} kg exceeds "
            f"{self.maximum_mass:.3f} kg",
        )

        camera_lost = (
            active
            and self.state.state in (KtyStationState.SETTLE, KtyStationState.SCAN)
            and (
                now - self.last_camera_s > self.camera_timeout
                or not self.last_camera_ok
            )
        )
        # Warning only: StationControllerV2 owns the retry counter and promotes
        # repeated timeouts to a latched FAULT.  A single dropped frame must not
        # abort the loading cycle.
        self._publish_fault(
            KtyFault.CAMERA_LOST_VIEW,
            camera_lost,
            KtyFault.WARNING,
            "No recent valid depth frame for the KTY ROI",
        )

        outside_names: list[str] = []
        if active and pose_stream_alive and kty_pose is not None:
            for name, truth in self.products.items():
                pose = self.poses.get(name)
                if pose is None:
                    continue
                # Products above the rim may still be on the chute.
                if pose.z > 0.95:
                    continue
                half_x = float(truth.size_m.x) / 2.0
                half_y = float(truth.size_m.y) / 2.0
                outside = (
                    abs(pose.x - kty_pose.x) + half_x > 0.315
                    or abs(pose.y - kty_pose.y) + half_y > 0.215
                    or pose.z < 0.42
                )
                if outside:
                    outside_names.append(name)
        self._publish_fault(
            KtyFault.PRODUCT_OUTSIDE_KTY,
            bool(outside_names),
            KtyFault.CRITICAL,
            "Products outside KTY: " + ", ".join(outside_names[:5]),
        )

        jammed_names: list[str] = []
        shutter_open = self.state.state in (
            KtyStationState.LOAD,
            KtyStationState.VIBRATE,
        )
        if pose_stream_alive:
            for name in self.products:
                pose = self.poses.get(name)
                if pose is None:
                    continue
                on_chute = -1.30 <= pose.x <= -0.23 and 0.88 <= pose.z <= 1.70
                if shutter_open and on_chute and pose.speed < 0.02:
                    self.stationary_since.setdefault(name, now)
                    if now - self.stationary_since[name] >= self.chute_jam_timeout:
                        jammed_names.append(name)
                else:
                    self.stationary_since.pop(name, None)
        self._publish_fault(
            KtyFault.PRODUCT_JAMMED_ON_CHUTE,
            bool(jammed_names),
            KtyFault.CRITICAL,
            "Products stationary on chute: " + ", ".join(jammed_names[:5]),
        )

        moving_names: list[str] = []
        if pose_stream_alive and self.state.state == KtyStationState.SCAN:
            for name in self.products:
                pose = self.poses.get(name)
                if pose is None or pose.z > 0.95:
                    continue
                if pose.speed > self.still_speed:
                    self.moving_since.setdefault(name, now)
                    if now - self.moving_since[name] >= self.still_timeout:
                        moving_names.append(name)
                else:
                    self.moving_since.pop(name, None)
        else:
            self.moving_since.clear()
        self._publish_fault(
            KtyFault.PRODUCT_STILL_MOVING,
            bool(moving_names),
            KtyFault.CRITICAL,
            "Products still moving after vibration: " + ", ".join(moving_names[:5]),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtySafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
