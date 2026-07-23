from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SeparatorDemoController(Node):
    """Continuously commands all moving surfaces of the separator demo."""

    TOPICS = {
        "infeed_speed_mps": "/singulator/separator/infeed/cmd_vel",
        "screen_speed_mps": "/singulator/separator/screen/cmd_vel",
        "accepted_speed_mps": "/singulator/separator/accepted/cmd_vel",
        "reject_speed_mps": "/singulator/separator/reject/cmd_vel",
    }

    DEFAULTS = {
        "infeed_speed_mps": 0.65,
        "screen_speed_mps": 0.55,
        "accepted_speed_mps": 0.70,
        "reject_speed_mps": 0.50,
    }

    def __init__(self) -> None:
        super().__init__("separator_demo_controller")

        for name, default in self.DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter("publish_rate_hz", 20.0)

        publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.publishers = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in self.TOPICS.items()
        }
        self.last_command: tuple[float, ...] | None = None
        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_commands,
        )

    def publish_commands(self) -> None:
        speeds = {
            name: float(self.get_parameter(name).value)
            for name in self.TOPICS
        }

        current = tuple(speeds[name] for name in self.TOPICS)
        if current != self.last_command:
            self.get_logger().info(
                "Separator speeds: "
                + ", ".join(
                    f"{name.removesuffix('_speed_mps')}={value:.2f} m/s"
                    for name, value in speeds.items()
                )
            )
            self.last_command = current

        for name, publisher in self.publishers.items():
            message = Float64()
            message.data = speeds[name]
            publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeparatorDemoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
