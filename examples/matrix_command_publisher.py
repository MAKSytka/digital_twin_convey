#!/usr/bin/env python3
from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node

from singulator_interfaces.msg import MatrixCommand


ROWS = 14
COLS = 4


class OneShotProfilePublisher(Node):
    def __init__(self, values: list[float], repeat_hz: float, duration: float) -> None:
        super().__init__("matrix_command_example")
        self.values = values
        self.duration = duration
        self.started_at = self.get_clock().now()
        self.publisher = self.create_publisher(
            MatrixCommand,
            "/singulator/matrix/command",
            10,
        )
        self.timer = self.create_timer(1.0 / repeat_hz, self.publish)

    def publish(self) -> None:
        message = MatrixCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "matrix"
        message.rows = ROWS
        message.cols = COLS
        message.target_speed_mps = self.values
        self.publisher.publish(message)

        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if elapsed >= self.duration:
            raise SystemExit(0)


def make_profile(mode: str, speed: float, delta: float) -> list[float]:
    if mode == "stop":
        return [0.0] * (ROWS * COLS)
    if mode == "uniform":
        return [speed] * (ROWS * COLS)

    values: list[float] = []
    for _row in range(ROWS):
        for col in range(COLS):
            left_half = col < 2
            sign = 1.0 if left_half else -1.0
            if mode == "turn_cw":
                sign *= -1.0
            values.append(max(-2.0, min(2.0, speed + sign * delta)))
    return values


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("stop", "uniform", "turn_ccw", "turn_cw"),
        default="uniform",
    )
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--delta", type=float, default=0.1)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    if args.rate <= 0.0 or args.duration <= 0.0:
        parser.error("--rate and --duration must be positive")

    profile = make_profile(args.mode, args.speed, args.delta)

    rclpy.init()
    node = OneShotProfilePublisher(profile, args.rate, args.duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
