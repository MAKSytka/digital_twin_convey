"""High-rate vertical reference for the KTY vibration platform.

The station controller sequences the cycle at 50 Hz.  This node publishes the
20-50 Hz sinusoidal platform position at 500 Hz.  It deliberately does not
command KTY velocity: the container remains a dynamic body and follows the
platform through contact, as in the real station.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from singulator_interfaces.msg import KtyStationState


class VibrationDriver(Node):
    def __init__(self) -> None:
        super().__init__("kty_vibration_driver")

        self.state: KtyStationState | None = None
        self.previous_state = KtyStationState.WAIT_EMPTY_KTY
        self.phase_started_s = self._now_s()

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            KtyStationState,
            "/kty/station/state",
            self._on_state,
            transient_qos,
        )
        self.platform_pub = self.create_publisher(
            Float64,
            "/kty/platform/cmd_pos_filtered",
            20,
        )
        self.timer = self.create_timer(0.002, self._tick)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_state(self, message: KtyStationState) -> None:
        if (
            message.state == KtyStationState.VIBRATE
            and self.previous_state != KtyStationState.VIBRATE
        ):
            self.phase_started_s = self._now_s()
        self.previous_state = message.state
        self.state = message

    def _tick(self) -> None:
        position = 0.0
        if (
            self.state is not None
            and self.state.state == KtyStationState.VIBRATE
            and self.state.vibration_enabled
        ):
            frequency = float(self.state.vibration_frequency_hz)
            amplitude = float(self.state.vibration_amplitude_m)
            omega = 2.0 * math.pi * frequency
            phase = omega * (self._now_s() - self.phase_started_s)
            position = amplitude * math.sin(phase)

        platform = Float64()
        platform.data = position
        self.platform_pub.publish(platform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VibrationDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        platform = Float64()
        platform.data = 0.0
        node.platform_pub.publish(platform)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
