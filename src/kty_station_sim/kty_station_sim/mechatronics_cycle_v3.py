"""Roller-free KTY cycle using deterministic flat conveyor surfaces."""

from __future__ import annotations

import threading

import rclpy
from std_msgs.msg import Float64

from .mechatronics_cycle_v2 import KtyMechatronicsCycleV2


class KtyMechatronicsCycleV3(KtyMechatronicsCycleV2):
    """Two-KTY cycle with a lifecycle locator and surface velocity in m/s."""

    LOCATOR_NAME = "kty_mech_runtime_locator"
    LOCATOR_X = 0.350
    LOCATOR_Y = 0.0
    LOCATOR_Z = 0.500

    def __init__(self) -> None:
        # The base constructor starts the worker thread. Override _worker_main and
        # hold it here until v3 publishers and linear-speed semantics are ready.
        self._v3_ready = threading.Event()
        self._locator_spawned = False
        self._last_nonzero_outfeed = 0.0
        super().__init__()

        # The legacy stage converted linear belt speed to roller rad/s. The flat
        # conveyor plugin consumes the commanded linear velocity directly.
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
            # Retained as telemetry compatibility. The runtime locator itself is
            # now a static model created and removed through Gazebo services.
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

        self._v3_ready.set()
        self.get_logger().info(
            "Runtime v9: lifecycle locator plus deterministic flat-surface velocity"
        )

    def _worker_main(self) -> None:
        self._v3_ready.wait()
        super()._worker_main()

    @staticmethod
    def _locator_sdf() -> str:
        return """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="kty_mech_runtime_locator">
    <static>true</static>
    <link name="locator">
      <collision name="blade">
        <pose>0 0 0.0575 0 0 0</pose>
        <geometry><box><size>0.020 0.520 0.115</size></box></geometry>
      </collision>
      <visual name="blade_visual">
        <pose>0 0 0.0575 0 0 0</pose>
        <geometry><box><size>0.020 0.520 0.115</size></box></geometry>
        <material>
          <ambient>0.18 0.55 0.72 1</ambient>
          <diffuse>0.24 0.70 0.90 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def _spawn_locator_model(self) -> None:
        poses = self._read_world_poses()
        if self.LOCATOR_NAME in poses:
            self._locator_spawned = True
            return
        created = self._create_model(
            self.LOCATOR_NAME,
            self._locator_sdf(),
            x=self.LOCATOR_X,
            y=self.LOCATOR_Y,
            z=self.LOCATOR_Z,
        )
        if not created:
            raise RuntimeError("Gazebo rejected the runtime locator model")
        self._locator_spawned = True
        self._known_models.add(self.LOCATOR_NAME)
        self.get_logger().info("Runtime locator UP (static model created)")

    def _remove_locator_model(self) -> None:
        poses = self._read_world_poses()
        if self.LOCATOR_NAME not in poses and not self._locator_spawned:
            return
        removed = self._remove_model(self.LOCATOR_NAME)
        if not removed and self.LOCATOR_NAME in self._read_world_poses():
            raise RuntimeError("Gazebo did not remove the runtime locator")
        self._locator_spawned = False
        self._known_models.discard(self.LOCATOR_NAME)
        self.get_logger().info("Runtime locator DOWN (static model removed)")

    def _cleanup_stale_models(self) -> None:
        super()._cleanup_stale_models()
        if self.LOCATOR_NAME in self._read_world_poses():
            self._remove_model(self.LOCATOR_NAME)
        self._locator_spawned = False
        self._known_models.discard(self.LOCATOR_NAME)

    def _initialise_mechanics(self) -> None:
        super()._initialise_mechanics()
        self._spawn_locator_model()

    def _set_commands(self, **updates: float) -> None:
        outfeed = updates.get("outfeed")
        if outfeed is not None and abs(float(outfeed)) > 1.0e-6:
            self._last_nonzero_outfeed = float(outfeed)
        super()._set_commands(**updates)

    def _changeover(self) -> None:
        old_kty = self._active_kty
        next_kty = self._queue_kty
        old_products = self._products_inside(old_kty)

        self._transition(
            "EJECT_ACTIVE",
            "remove locator and open clamps before enabling conveyor surfaces",
        )
        self._set_commands(
            clamps=0.0,
            locator=0.0,
            active=0.0,
            outfeed=0.0,
        )
        # Deleting the physical stop removes all ambiguity about joint travel at
        # low RTF. The dwell is only for the side clamps to open.
        self._remove_locator_model()
        self._interruptible_sleep(1.2)

        self._transition(
            "EJECT_ACTIVE",
            "locator absent; imposing 0.65 m/s on active and outfeed surfaces",
        )
        self._set_commands(
            active=self.roller_speed,
            outfeed=self.roller_speed,
        )
        self._wait_for_x(old_kty, minimum_x=1.25, timeout_s=7.0)

        self._transition(
            "POSITION_NEXT",
            "create locator, extend pusher and move queued KTY to active position",
        )
        self._spawn_locator_model()
        self._set_commands(
            locator=0.0,
            pusher=self.pusher_extended,
            infeed=self.roller_speed,
            active=self.roller_speed,
        )

        self._approach_locator(next_kty, timeout_s=8.0)
        self._set_commands(infeed=0.0, active=0.0, pusher=0.0)
        self._set_commands(clamps=self.clamp_closed)
        self._set_vibration("weak")

        self._transition(
            "VERIFY_READY",
            "checking position, velocity, clamps, camera and previous-KTY clearance",
        )
        self._wait_until_ready(next_kty, old_kty, timeout_s=8.0)

        self._remove_model(old_kty)
        self._known_models.discard(old_kty)
        for name in sorted(old_products):
            self._remove_model(name)
            self._known_models.discard(name)
            self._active_product_names.discard(name)
        self._set_commands(outfeed=0.0)

        self._active_kty = next_kty
        self._queue_kty = self._new_kty_name(self._cycle_id + 2)
        self._spawn_kty(self._queue_kty, self.queue_spawn_x)

        self._transition(
            "OPEN_GATE",
            "new KTY ready; opening gate and releasing accumulated products",
        )
        self._set_commands(gate=self.gate_open)
        self._interruptible_sleep(0.5)

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload["transport"] = "flat_contact_surface_velocity"
        payload["locator_mechanism"] = "spawned_static_model"
        payload["locator_up"] = self._locator_spawned
        payload["infeed_surface_velocity_mps"] = self._commands["infeed"]
        payload["active_surface_velocity_mps"] = self._commands["active"]
        payload["outfeed_surface_velocity_mps"] = self._commands["outfeed"]
        payload["last_nonzero_outfeed_mps"] = self._last_nonzero_outfeed
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
