import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class AuxConveyorController(Node):
    """Continuously commands the infeed and outfeed conveyors."""

    def __init__(self) -> None:
        super().__init__("aux_conveyor_controller")

        self.declare_parameter("infeed_speed_mps", 2.0)
        self.declare_parameter("outfeed_speed_mps", 2.0)
        self.declare_parameter("publish_rate_hz", 10.0)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.infeed_publisher = self.create_publisher(
            Float64,
            "/singulator/infeed/cmd_vel",
            10,
        )
        self.outfeed_publisher = self.create_publisher(
            Float64,
            "/singulator/outfeed/cmd_vel",
            10,
        )

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_commands)
        self.last_speeds: tuple[float, float] | None = None

    def publish_commands(self) -> None:
        infeed_speed = float(
            self.get_parameter("infeed_speed_mps").value
        )
        outfeed_speed = float(
            self.get_parameter("outfeed_speed_mps").value
        )

        current = (infeed_speed, outfeed_speed)
        if current != self.last_speeds:
            self.get_logger().info(
                "Conveyor speeds: "
                f"infeed={infeed_speed:.3f} m/s, "
                f"outfeed={outfeed_speed:.3f} m/s"
            )
            self.last_speeds = current

        infeed_message = Float64()
        infeed_message.data = infeed_speed
        self.infeed_publisher.publish(infeed_message)

        outfeed_message = Float64()
        outfeed_message.data = outfeed_speed
        self.outfeed_publisher.publish(outfeed_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AuxConveyorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
