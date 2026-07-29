"""Roller-free KTY cycle using flat contact conveyor zones."""

from __future__ import annotations

import rclpy
from std_msgs.msg import Float64

from .mechatronics_cycle_v2 import KtyMechatronicsCycleV2


class KtyMechatronicsCycleV3(KtyMechatronicsCycleV2):
    """Reuse the accepted state machine with surface velocity commands in m/s."""

    def __init__(self) -> None:
        super().__init__()

        # The stage-4 controller converted linear speed to roller angular speed.
        # The contact-surface plugin consumes linear velocity directly.
        self.roller_speed = float(
            self.get_parameter("roller_linear_speed_mps").value
        )
        self.slow_roller_speed = float(
            self.get_parameter("slow_roller_linear_speed_mps").value
        )

        old_publishers = dict(self.command_publishers)
        self.command_publishers = {
            "infeed": self.create_publisher(
                Float64,
                "/kty/mech/infeed_surface/cmd_vel",
                10,
            ),
            "active": self.create_publisher(
                Float64,
                "/kty/mech/active_surface/cmd_vel",
                10,
            ),
            "outfeed": self.create_publisher(
                Float64,
                "/kty/mech/outfeed_surface/cmd_vel",
                10,
            ),
            "pusher": self.create_publisher(
                Float64,
                "/kty/mech/pusher/cmd_pos",
                10,
            ),
            "clamps": self.create_publisher(
                Float64,
                "/kty/mech/clamps/cmd_pos",
                10,
            ),
            "gate": self.create_publisher(
                Float64,
                "/kty/mech/gate_state",
                10,
            ),
            "vibration": self.create_publisher(
                Float64,
                "/kty/mech/vibration/cmd_pos",
                10,
            ),
            "locator": self.create_publisher(
                Float64,
                "/kty/mech/locator_stop/cmd_pos",
                10,
            ),
        }
        for publisher in old_publishers.values():
            try:
                self.destroy_publisher(publisher)
            except Exception:
                pass

        self.get_logger().info(
            "Runtime v8: roller-free contact surfaces; commands are linear m/s"
        )

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload["transport"] = "flat_contact_surface"
        payload["infeed_surface_velocity_mps"] = self._commands["infeed"]
        payload["active_surface_velocity_mps"] = self._commands["active"]
        payload["outfeed_surface_velocity_mps"] = self._commands["outfeed"]
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
