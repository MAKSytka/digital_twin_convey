from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SeparatorDemoController(Node):
    """Continuously command every moving surface of the separator demo."""

    BELT_TOPICS = {
        "infeed": "/singulator/separator/infeed/cmd_vel",
        "accepted": "/singulator/separator/accepted/cmd_vel",
        "reject_transfer": "/singulator/separator/reject_transfer/cmd_vel",
        "reject": "/singulator/separator/reject/cmd_vel",
    }
    SCREEN_TOPIC = "/singulator/separator/screen/cmd_vel"

    def __init__(self) -> None:
        super().__init__("separator_demo_controller")
        self.declare_parameter("conveyor_speed_mps", 2.0)
        self.declare_parameter("screen_surface_speed_mps", 2.0)
        self.declare_parameter("shaft_collision_radius_m", 0.025)
        self.declare_parameter("publish_rate_hz", 30.0)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        radius = float(self.get_parameter("shaft_collision_radius_m").value)
        if radius <= 0.0:
            raise ValueError("shaft_collision_radius_m must be positive")

        self.belt_publishers = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in self.BELT_TOPICS.items()
        }
        self.screen_publisher = self.create_publisher(
            Float64,
            self.SCREEN_TOPIC,
            10,
        )
        self.last_command: tuple[float, float, float] | None = None
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_commands)

    def publish_commands(self) -> None:
        conveyor_speed = float(
            self.get_parameter("conveyor_speed_mps").value
        )
        surface_speed = float(
            self.get_parameter("screen_surface_speed_mps").value
        )
        radius = float(
            self.get_parameter("shaft_collision_radius_m").value
        )
        if radius <= 0.0:
            self.get_logger().error(
                "shaft_collision_radius_m became non-positive; command skipped"
            )
            return

        conveyor_speed = max(-2.5, min(2.5, conveyor_speed))
        surface_speed = max(-2.5, min(2.5, surface_speed))
        angular_speed = surface_speed / radius
        angular_speed = max(-120.0, min(120.0, angular_speed))

        current = (conveyor_speed, surface_speed, angular_speed)
        if current != self.last_command:
            self.get_logger().info(
                "Separator commands: "
                f"belts={conveyor_speed:.3f} m/s, "
                f"screen_surface={surface_speed:.3f} m/s, "
                f"shaft={angular_speed:.3f} rad/s"
            )
            self.last_command = current

        belt_message = Float64()
        belt_message.data = conveyor_speed
        for publisher in self.belt_publishers.values():
            publisher.publish(belt_message)

        screen_message = Float64()
        screen_message.data = angular_speed
        self.screen_publisher.publish(screen_message)


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
