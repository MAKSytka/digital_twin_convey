"""Wall-clock heartbeat for the isolated KTY smoke scenario.

This node deliberately does not use Gazebo simulation time.  Its only purpose is
to prove that the ROS package, launch file and executor are alive independently
of /clock, perception, custom messages and the future station state machine.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KtySmokeHeartbeat(Node):
    """Publish an observable heartbeat once per second using wall time."""

    def __init__(self) -> None:
        super().__init__("kty_smoke_heartbeat")

        self.publisher = self.create_publisher(String, "/kty/smoke/heartbeat", 10)
        self.sequence = 0
        self.started_monotonic = time.monotonic()
        self.timer = self.create_timer(1.0, self._publish_heartbeat)

        self.get_logger().info(
            "KTY smoke heartbeat started with system time; use_sim_time=false"
        )

    def _publish_heartbeat(self) -> None:
        self.sequence += 1
        payload = {
            "status": "alive",
            "sequence": self.sequence,
            "wall_uptime_s": round(time.monotonic() - self.started_monotonic, 3),
            "expected_world": "kty_station_smoke",
            "expected_model": "kty_smoke_container",
        }

        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtySmokeHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
