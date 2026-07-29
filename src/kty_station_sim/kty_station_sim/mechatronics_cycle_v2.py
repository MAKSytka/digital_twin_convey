"""Runtime-v7 corrections for the physical KTY cycle.

The transport state machine from stage 4 is retained, but the unreliable
hinged gate is replaced by an idempotent static slide gate created and removed
through Gazebo lifecycle services.  Ejection monitoring also boosts the roller
command when a loaded KTY stalls before the outfeed.
"""

from __future__ import annotations

import time

import rclpy

from .mechatronics_cycle import KtyMechatronicsCycle, RestartRequested


class KtyMechatronicsCycleV2(KtyMechatronicsCycle):
    GATE_NAME = "kty_mech_chute_gate"
    GATE_X = -0.295
    GATE_Y = 0.0
    GATE_Z = 0.995

    def __init__(self) -> None:
        # These fields must exist before the base class starts its worker.
        self._gate_model_spawned = False
        super().__init__()
        self.get_logger().info(
            "Runtime v7: static slide gate, anchored roller joints and slower feeder"
        )

    @staticmethod
    def _gate_sdf() -> str:
        # A vertical plate at the physical end of the inclined chute.  Its
        # lower edge overlaps the chute surface, so small flat products cannot
        # pass underneath it while the next KTY is being positioned.
        return """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="kty_mech_chute_gate">
    <static>true</static>
    <link name="slide_gate">
      <collision name="blade">
        <geometry><box><size>0.035 0.620 0.260</size></box></geometry>
        <surface><friction><ode><mu>0.85</mu><mu2>0.85</mu2></ode></friction></surface>
      </collision>
      <visual name="blade_visual">
        <geometry><box><size>0.035 0.620 0.260</size></box></geometry>
        <material><ambient>0.88 0.42 0.08 1</ambient><diffuse>1.0 0.56 0.10 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def _spawn_gate_model(self) -> None:
        if self._gate_model_spawned:
            return
        poses = self._read_world_poses()
        if self.GATE_NAME in poses:
            self._gate_model_spawned = True
            return
        created = self._create_model(
            self.GATE_NAME,
            self._gate_sdf(),
            x=self.GATE_X,
            y=self.GATE_Y,
            z=self.GATE_Z,
        )
        if not created:
            raise RuntimeError("Gazebo rejected the slide gate model")
        self._gate_model_spawned = True
        self._known_models.add(self.GATE_NAME)
        self.get_logger().info("Slide gate CLOSED (model created)")

    def _remove_gate_model(self) -> None:
        poses = self._read_world_poses()
        exists = self.GATE_NAME in poses or self._gate_model_spawned
        if not exists:
            self._gate_model_spawned = False
            return
        removed = self._remove_model(self.GATE_NAME)
        if not removed and self.GATE_NAME in self._read_world_poses():
            raise RuntimeError("Gazebo did not remove the slide gate")
        self._gate_model_spawned = False
        self._known_models.discard(self.GATE_NAME)
        self.get_logger().info("Slide gate OPEN (model removed)")

    def _set_commands(self, **updates: float) -> None:
        gate_command = updates.get("gate")
        super()._set_commands(**updates)
        if gate_command is None:
            return
        # Preserve the old numeric command in state telemetry, but use model
        # lifecycle for the actual physical barrier.
        if gate_command > 0.5 * self.gate_open:
            self._remove_gate_model()
        else:
            self._spawn_gate_model()

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload["gate_open"] = not self._gate_model_spawned
        payload["gate_mechanism"] = "spawned_static_slide"
        return payload

    def _cleanup_stale_models(self) -> None:
        super()._cleanup_stale_models()
        if self.GATE_NAME in self._read_world_poses():
            self._remove_model(self.GATE_NAME)
        self._gate_model_spawned = False
        self._known_models.discard(self.GATE_NAME)

    def _wait_for_x(self, name: str, minimum_x: float, timeout_s: float) -> None:
        # The original seven-second timeout was too short at low RTF.  The
        # timeout is wall-clock based, while the physical system advances in
        # simulation time, so runtime v7 permits a longer interval and boosts
        # roller speed only after a genuine lack of positional progress.
        deadline = time.monotonic() + max(18.0, timeout_s)
        last_progress_at = time.monotonic()
        best_x = -1.0e9
        boosted = False
        while time.monotonic() < deadline:
            self._check_interrupt()
            pose = self._read_pose(name)
            if pose is not None:
                if pose.x >= minimum_x:
                    return
                if pose.x > best_x + 0.012:
                    best_x = pose.x
                    last_progress_at = time.monotonic()
                elif time.monotonic() - last_progress_at >= 3.0 and not boosted:
                    self.get_logger().warning(
                        f"{name} stalled at x={pose.x:.3f}; boosting active/outfeed rollers"
                    )
                    self._set_commands(
                        active=1.45 * self.roller_speed,
                        outfeed=1.45 * self.roller_speed,
                    )
                    boosted = True
            self._interruptible_sleep(0.18)
        raise RuntimeError(f"{name} did not clear active zone")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV2()
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
