"""Continuous command source for the angled-roller centring throat."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class RollerThroatController(Node):
    def __init__(self) -> None:
        super().__init__("roller_throat_controller")
        self.declare_parameter("speed_mps", 2.00)
        self.declare_parameter("publish_rate_hz", 20.0)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.left_publisher = self.create_publisher(
            Float64,
            "/singulator/throat/left/cmd_vel",
            10,
        )
        self.right_publisher = self.create_publisher(
            Float64,
            "/singulator/throat/right/cmd_vel",
            10,
        )
        self.last_speed: float | None = None
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self) -> None:
        speed = float(self.get_parameter("speed_mps").value)
        speed = max(-3.0, min(3.0, speed))
        if speed != self.last_speed:
            self.get_logger().info(
                f"Angled roller throat speed: {speed:.3f} m/s"
            )
            self.last_speed = speed

        message = Float64()
        message.data = speed
        self.left_publisher.publish(message)
        self.right_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RollerThroatController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
