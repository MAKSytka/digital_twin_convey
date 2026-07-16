import rclpy

from rclpy.node import Node
from singulator_interfaces.msg import MatrixCommand


class MatrixTestController(Node):
    def __init__(self) -> None:
        super().__init__("matrix_test_controller")

        self.declare_parameter("rows", 3)
        self.declare_parameter("cols", 4)

        self.declare_parameter("mode", "stop")

        self.declare_parameter(
            "base_speed_mps",
            0.15,
        )

        self.declare_parameter(
            "delta_speed_mps",
            0.05,
        )

        self.declare_parameter(
            "publish_rate_hz",
            50.0,
        )

        self.command_publisher = self.create_publisher(
            MatrixCommand,
            "/singulator/matrix/command",
            10,
        )

        publish_rate = float(
            self.get_parameter(
                "publish_rate_hz"
            ).value
        )

        if publish_rate <= 0.0:
            raise ValueError(
                "publish_rate_hz должен быть больше нуля"
            )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_command,
        )

        self.last_signature = None

        self.get_logger().info(
            "Matrix test controller started"
        )

    @staticmethod
    def clamp_speed(speed: float) -> float:
        return max(-2.0, min(2.0, speed))

    @staticmethod
    def row_gains(
        mode: str,
        rows: int,
    ) -> list[float]:
        gains = [0.0] * rows

        middle_row = rows // 2

        if mode.startswith("middle_"):
            gains[middle_row] = 1.0

        elif mode.startswith("progressive_"):
            for row in range(rows):
                distance = abs(row - middle_row)

                gains[row] = max(
                    0.25,
                    1.0 - 0.5 * distance,
                )

        elif mode.startswith("all_"):
            gains = [1.0] * rows

        return gains

    def make_profile(
        self,
        rows: int,
        cols: int,
        mode: str,
        base_speed: float,
        delta_speed: float,
    ) -> list[float]:

        if mode == "stop":
            return [0.0] * (rows * cols)

        if mode == "uniform":
            return [
                self.clamp_speed(base_speed)
            ] * (rows * cols)

        valid_modes = {
            "middle_ccw",
            "middle_cw",
            "progressive_ccw",
            "progressive_cw",
            "all_ccw",
            "all_cw",
        }

        if mode not in valid_modes:
            self.get_logger().error(
                f"Неизвестный режим: {mode}"
            )

            return [0.0] * (rows * cols)

        gains = self.row_gains(
            mode=mode,
            rows=rows,
        )

        rotation_sign = (
            1.0
            if mode.endswith("_ccw")
            else -1.0
        )

        values = []

        for row in range(rows):
            row_delta = (
                rotation_sign
                * delta_speed
                * gains[row]
            )

            for col in range(cols):
                # Колонки 0 и 1 — одна сторона.
                # Колонки 2 и 3 — другая сторона.
                side_sign = (
                    1.0
                    if col < cols / 2
                    else -1.0
                )

                speed = (
                    base_speed
                    + side_sign * row_delta
                )

                values.append(
                    self.clamp_speed(speed)
                )

        return values

    def publish_command(self) -> None:
        rows = int(
            self.get_parameter("rows").value
        )

        cols = int(
            self.get_parameter("cols").value
        )

        mode = str(
            self.get_parameter("mode").value
        )

        base_speed = float(
            self.get_parameter(
                "base_speed_mps"
            ).value
        )

        delta_speed = float(
            self.get_parameter(
                "delta_speed_mps"
            ).value
        )

        if rows <= 0 or cols <= 0:
            self.get_logger().error(
                "rows и cols должны быть положительными"
            )
            return

        signature = (
            rows,
            cols,
            mode,
            base_speed,
            delta_speed,
        )

        if signature != self.last_signature:
            profile = self.make_profile(
                rows=rows,
                cols=cols,
                mode=mode,
                base_speed=base_speed,
                delta_speed=delta_speed,
            )

            self.get_logger().info(
                f"mode={mode}; "
                f"matrix={rows}x{cols}; "
                f"profile={profile}"
            )

            self.last_signature = signature

        message = MatrixCommand()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.header.frame_id = "matrix"

        message.rows = rows
        message.cols = cols

        message.target_speed_mps = self.make_profile(
            rows=rows,
            cols=cols,
            mode=mode,
            base_speed=base_speed,
            delta_speed=delta_speed,
        )

        self.command_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MatrixTestController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
