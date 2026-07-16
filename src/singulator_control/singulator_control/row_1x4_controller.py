import rclpy
from rclpy.node import Node

from singulator_interfaces.msg import MatrixCommand


class Row1x4Controller(Node):
    """Тестовый контроллер одного поперечного ряда из четырёх ячеек."""

    def __init__(self) -> None:
        super().__init__("row_1x4_controller")

        self.declare_parameter("mode", "stop")

        # Средняя поступательная скорость всего ряда.
        self.declare_parameter("base_speed_mps", 0.15)

        # Добавка / вычитание скорости для создания вращения.
        self.declare_parameter("delta_speed_mps", 0.05)

        self.declare_parameter("publish_rate_hz", 50.0)

        self.publisher = self.create_publisher(
            MatrixCommand,
            "/singulator/matrix/command",
            10,
        )

        publish_rate = float(
            self.get_parameter("publish_rate_hz").value
        )

        if publish_rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_command,
        )

        self.last_mode = None

        self.get_logger().info(
            "Row 1x4 controller started"
        )

    @staticmethod
    def make_profile(
        mode: str,
        base_speed: float,
        delta_speed: float,
    ) -> list[float]:

        # Не позволяем тестовому контроллеру случайно
        # сформировать отрицательную скорость в рабочих режимах.
        slow_speed = max(0.0, base_speed - delta_speed)
        fast_speed = base_speed + delta_speed

        profiles = {
            "stop": [
                0.0,
                0.0,
                0.0,
                0.0,
            ],

            # Обычное поступательное движение.
            "uniform": [
                base_speed,
                base_speed,
                base_speed,
                base_speed,
            ],

            # Левая сторона быстрее правой.
            "turn_ccw": [
                fast_speed,
                fast_speed,
                slow_speed,
                slow_speed,
            ],

            # Правая сторона быстрее левой.
            "turn_cw": [
                slow_speed,
                slow_speed,
                fast_speed,
                fast_speed,
            ],

            # Диагностическое вращение практически на месте.
            # В реальном рабочем алгоритме использовать не обязательно.
            "spin_ccw": [
                delta_speed,
                delta_speed,
                -delta_speed,
                -delta_speed,
            ],

            "spin_cw": [
                -delta_speed,
                -delta_speed,
                delta_speed,
                delta_speed,
            ],
        }

        return profiles.get(mode, profiles["stop"])

    def publish_command(self) -> None:
        mode = str(self.get_parameter("mode").value)

        base_speed = float(
            self.get_parameter("base_speed_mps").value
        )

        delta_speed = float(
            self.get_parameter("delta_speed_mps").value
        )

        if mode != self.last_mode:
            self.get_logger().info(
                f"Mode: {mode}; "
                f"base={base_speed:.3f} m/s; "
                f"delta={delta_speed:.3f} m/s"
            )
            self.last_mode = mode

        command = MatrixCommand()

        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "matrix"

        command.rows = 1
        command.cols = 4

        command.target_speed_mps = self.make_profile(
            mode,
            base_speed,
            delta_speed,
        )

        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Row1x4Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
