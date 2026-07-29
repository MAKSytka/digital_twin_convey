"""Wait until the bridged Gazebo simulation clock is actually advancing.

ROS nodes with ``use_sim_time=true`` do not execute ROS-time timers before
``/clock`` starts advancing.  Merely seeing the topic in the ROS graph is not
enough: the bridge process can advertise it while Gazebo is paused or while the
clock bridge has an incompatible QoS profile.

This short-lived process uses wall time for its own wait loop and exits only
after it has observed two strictly increasing clock samples.  The launch file
starts the KTY state machine, spawner and vibration driver after this process
exits successfully.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock


class ClockGate(Node):
    """Detect a live, advancing ROS simulation clock."""

    def __init__(self) -> None:
        super().__init__("kty_clock_gate")

        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)

        self.first_stamp_ns: int | None = None
        self.latest_stamp_ns: int | None = None
        self.ready = False

    @staticmethod
    def _stamp_ns(message: Clock) -> int:
        return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)

    def _on_clock(self, message: Clock) -> None:
        stamp_ns = self._stamp_ns(message)
        self.latest_stamp_ns = stamp_ns

        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
            self.get_logger().info(
                "Received first /clock sample; waiting for simulated time to advance"
            )
            return

        if stamp_ns > self.first_stamp_ns:
            self.ready = True
            self.get_logger().info(
                "Gazebo /clock is advancing; KTY runtime may start"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClockGate()
    last_warning_s = time.monotonic()

    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.20)
            now_s = time.monotonic()
            if now_s - last_warning_s >= 5.0:
                if node.latest_stamp_ns is None:
                    node.get_logger().warning(
                        "No /clock samples yet; checking Gazebo clock bridge and world state"
                    )
                else:
                    node.get_logger().warning(
                        "/clock is present but not advancing; resume the Gazebo world"
                    )
                last_warning_s = now_s
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
