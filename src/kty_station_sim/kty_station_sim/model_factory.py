"""SDF factories and random product specifications for the KTY station."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random


FRICTION = 0.75


def _box_inertia(
    mass: float,
    size_x: float,
    size_y: float,
    size_z: float,
) -> tuple[float, float, float]:
    return (
        mass * (size_y**2 + size_z**2) / 12.0,
        mass * (size_x**2 + size_z**2) / 12.0,
        mass * (size_x**2 + size_y**2) / 12.0,
    )


def _surface_xml(mu: float = FRICTION) -> str:
    return f"""
      <surface>
        <friction>
          <ode>
            <mu>{mu:.6f}</mu>
            <mu2>{mu:.6f}</mu2>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.03</restitution_coefficient>
          <threshold>0.30</threshold>
        </bounce>
        <contact>
          <ode>
            <kp>5000000</kp>
            <kd>120</kd>
            <max_vel>0.10</max_vel>
            <min_depth>0.0005</min_depth>
          </ode>
        </contact>
      </surface>
    """


@dataclass(frozen=True, slots=True)
class ProductSpec:
    size_x: float
    size_y: float
    size_z: float
    mass: float
    yaw: float
    profile: str
    color: tuple[float, float, float]

    @classmethod
    def random(cls, rng: random.Random) -> "ProductSpec":
        profile = rng.choices(
            ("small_carton", "medium_carton", "large_carton", "flat_item"),
            weights=(0.25, 0.45, 0.15, 0.15),
            k=1,
        )[0]

        if profile == "small_carton":
            size_x = rng.uniform(0.035, 0.160)
            size_y = rng.uniform(0.015, 0.120)
            size_z = rng.uniform(0.010, 0.100)
            density = rng.uniform(120.0, 500.0)
        elif profile == "medium_carton":
            size_x = rng.uniform(0.120, 0.300)
            size_y = rng.uniform(0.080, 0.240)
            size_z = rng.uniform(0.040, 0.180)
            density = rng.uniform(100.0, 360.0)
        elif profile == "large_carton":
            size_x = rng.uniform(0.260, 0.400)
            size_y = rng.uniform(0.180, 0.320)
            size_z = rng.uniform(0.100, 0.280)
            density = rng.uniform(80.0, 260.0)
        else:
            size_x = rng.uniform(0.080, 0.350)
            size_y = rng.uniform(0.050, 0.280)
            size_z = rng.uniform(0.010, 0.035)
            density = rng.uniform(180.0, 650.0)

        volume = size_x * size_y * size_z
        surface_area = 2.0 * (
            size_x * size_y + size_x * size_z + size_y * size_z
        )
        mass = density * volume + 0.35 * surface_area
        mass = max(0.010, min(5.0, mass))

        # Deliberately small palette: equal-colour neighbouring products occur
        # regularly and exercise the depth / watershed split logic.
        palette = (
            (0.72, 0.42, 0.16),
            (0.72, 0.42, 0.16),
            (0.80, 0.65, 0.30),
            (0.25, 0.42, 0.78),
            (0.55, 0.25, 0.18),
        )

        return cls(
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            mass=mass,
            yaw=rng.uniform(-math.pi, math.pi),
            profile=profile,
            color=rng.choice(palette),
        )

    def to_sdf(self, model_name: str) -> str:
        ixx, iyy, izz = _box_inertia(
            self.mass,
            self.size_x,
            self.size_y,
            self.size_z,
        )
        r, g, b = self.color
        return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{model_name}">
    <allow_auto_disable>true</allow_auto_disable>
    <link name="body">
      <inertial>
        <mass>{self.mass:.9f}</mass>
        <inertia>
          <ixx>{ixx:.12f}</ixx><iyy>{iyy:.12f}</iyy><izz>{izz:.12f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <velocity_decay>
        <linear>0.02</linear>
        <angular>0.08</angular>
      </velocity_decay>
      <collision name="collision">
        <geometry><box><size>{self.size_x:.9f} {self.size_y:.9f} {self.size_z:.9f}</size></box></geometry>
        {_surface_xml()}
      </collision>
      <visual name="visual">
        <geometry><box><size>{self.size_x:.9f} {self.size_y:.9f} {self.size_z:.9f}</size></box></geometry>
        <material>
          <ambient>{r:.4f} {g:.4f} {b:.4f} 1</ambient>
          <diffuse>{r:.4f} {g:.4f} {b:.4f} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def make_kty_sdf(
    model_name: str,
    internal_x: float = 0.600,
    internal_y: float = 0.400,
    internal_z: float = 0.400,
    wall: float = 0.003,
    mass: float = 1.600,
) -> str:
    """Create an open, flapless, thin-walled KTY model.

    The model origin is at the outer bottom surface, which makes the spawn Z
    equal to the supporting conveyor / platform top Z.  A VelocityControl
    system is embedded in the spawned model.  It provides a deterministic
    transport actuator on ``/kty/carrier/cmd_vel`` while the visible conveyor
    zones continue to receive their contact-surface commands.
    """

    outer_x = internal_x + 2.0 * wall
    outer_y = internal_y + 2.0 * wall
    outer_z = internal_z + wall
    ixx, iyy, izz = _box_inertia(mass, outer_x, outer_y, outer_z)
    side_z = wall + internal_z / 2.0
    cardboard = "0.63 0.36 0.12 1"

    def collision_visual(
        name: str,
        pose: tuple[float, float, float],
        size: tuple[float, float, float],
    ) -> str:
        px, py, pz = pose
        sx, sy, sz = size
        return f"""
      <collision name="{name}_collision">
        <pose>{px:.9f} {py:.9f} {pz:.9f} 0 0 0</pose>
        <geometry><box><size>{sx:.9f} {sy:.9f} {sz:.9f}</size></box></geometry>
        {_surface_xml()}
      </collision>
      <visual name="{name}_visual">
        <pose>{px:.9f} {py:.9f} {pz:.9f} 0 0 0</pose>
        <geometry><box><size>{sx:.9f} {sy:.9f} {sz:.9f}</size></box></geometry>
        <material><ambient>{cardboard}</ambient><diffuse>{cardboard}</diffuse></material>
      </visual>
        """

    parts = [
        collision_visual(
            "bottom",
            (0.0, 0.0, wall / 2.0),
            (outer_x, outer_y, wall),
        ),
        collision_visual(
            "wall_neg_x",
            (-(internal_x + wall) / 2.0, 0.0, side_z),
            (wall, outer_y, internal_z),
        ),
        collision_visual(
            "wall_pos_x",
            ((internal_x + wall) / 2.0, 0.0, side_z),
            (wall, outer_y, internal_z),
        ),
        collision_visual(
            "wall_neg_y",
            (0.0, -(internal_y + wall) / 2.0, side_z),
            (internal_x, wall, internal_z),
        ),
        collision_visual(
            "wall_pos_y",
            (0.0, (internal_y + wall) / 2.0, side_z),
            (internal_x, wall, internal_z),
        ),
    ]

    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{model_name}">
    <allow_auto_disable>false</allow_auto_disable>
    <link name="body">
      <inertial>
        <pose>0 0 {outer_z / 2.0:.9f} 0 0 0</pose>
        <mass>{mass:.9f}</mass>
        <inertia>
          <ixx>{ixx:.12f}</ixx><iyy>{iyy:.12f}</iyy><izz>{izz:.12f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <velocity_decay><linear>0.02</linear><angular>0.08</angular></velocity_decay>
      {''.join(parts)}
    </link>
    <plugin filename="gz-sim-velocity-control-system"
            name="gz::sim::systems::VelocityControl">
      <topic>/kty/carrier/cmd_vel</topic>
      <initial_linear>0 0 0</initial_linear>
      <initial_angular>0 0 0</initial_angular>
    </plugin>
  </model>
</sdf>
"""
