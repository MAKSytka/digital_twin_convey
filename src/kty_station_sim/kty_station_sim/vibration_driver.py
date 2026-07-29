"""High-rate command driver for the KTY carrier and vibration platform.

The station state machine runs at 50 Hz, which is sufficient for sequencing but
not for a 20-50 Hz vibration reference.  This node republishes transport at
500 Hz and synthesises the sinusoidal platform / carrier motion from the
published station state.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from singulator_interfaces.msg import KtyStationState


class VibrationDriver(Node):
    def __init__(self) -> None:
        super().__init__("kty_vibration_driver")

        self.latest_transport = Twist()
        self.state: KtyStationState | None = None
        self.previous_state = KtyStationState.WAIT_EMPTY_KTY
        self.phase_started_s = self._now_s()

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Twist,
            "/kty/carrier/cmd_vel",
            self._on_transport,
            20,
        )
        self.create_subscription(
            KtyStationState,
            "/kty/station/state",
            self._on_state,
            transient_qos,
        )

        self.carrier_pub = self.create_publisher(
            Twist,
            "/kty/carrier/cmd_vel_filtered",
            20,
        )
        self.platform_pub = self.create_publisher(
            Float64,
            "/kty/platform/cmd_pos_filtered",
            20,
        )

        self.timer = self.create_timer(0.002, self._tick)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_transport(self, message: Twist) -> None:
        self.latest_transport = message

    def _on_state(self, message: KtyStationState) -> None:
        if (
            message.state == KtyStationState.VIBRATE
            and self.previous_state != KtyStationState.VIBRATE
        ):
            self.phase_started_s = self._now_s()
        self.previous_state = message.state
        self.state = message

    def _tick(self) -> None:
        output = Twist()
        output.linear.x = float(self.latest_transport.linear.x)
        output.linear.y = float(self.latest_transport.linear.y)
        output.angular.x = float(self.latest_transport.angular.x)
        output.angular.y = float(self.latest_transport.angular.y)
        output.angular.z = float(self.latest_transport.angular.z)

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
            output.linear.z = amplitude * omega * math.cos(phase)
        else:
            output.linear.z = 0.0

        platform = Float64()
        platform.data = position
        self.platform_pub.publish(platform)
        self.carrier_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VibrationDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.carrier_pub.publish(Twist())
        platform = Float64()
        platform.data = 0.0
        node.platform_pub.publish(platform)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
