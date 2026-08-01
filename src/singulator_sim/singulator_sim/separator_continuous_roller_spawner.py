#!/usr/bin/env python3
"""Spawner tuned for the separator with continuous transverse rollers."""

from __future__ import annotations

import math

import rclpy

from singulator_sim.box_model import BoxSpec
from singulator_sim.separator_demo_spawner import (
    BoxProfile,
    SeparatorDemoSpawner,
    UPPER_PROFILES,
)


# A continuous roller leaves openings only in the longitudinal X direction.
# Long and narrow items are therefore spawned almost perpendicular to the flow:
# their short side becomes the longitudinal projection that can enter the 100 mm
# opening between adjacent rollers. The LOWER cutoff remains 70 mm, leaving a
# 30 mm mechanical margin so near-cutoff parcels fall more reliably.
CONTINUOUS_ROLLER_LOWER_PROFILES = (
    BoxProfile(
        "micro_parcel",
        0.30,
        (0.035, 0.065),
        (0.015, 0.060),
        (0.010, 0.085),
        (-8.0, 8.0),
        (220.0, 950.0),
        0.080,
    ),
    BoxProfile(
        "long_narrow",
        0.28,
        (0.250, 0.400),
        (0.025, 0.055),
        (0.025, 0.120),
        (84.0, 96.0),
        (180.0, 850.0),
        0.200,
    ),
    BoxProfile(
        "flat_strip",
        0.18,
        (0.160, 0.380),
        (0.020, 0.055),
        (0.010, 0.030),
        (84.0, 96.0),
        (250.0, 1050.0),
        0.120,
    ),
    BoxProfile(
        "tall_slender",
        0.14,
        (0.100, 0.240),
        (0.025, 0.060),
        (0.150, 0.280),
        (84.0, 96.0),
        (160.0, 700.0),
        0.250,
    ),
    BoxProfile(
        "near_cutoff",
        0.10,
        (0.055, 0.069),
        (0.090, 0.220),
        (0.025, 0.140),
        (-3.0, 3.0),
        (180.0, 900.0),
        0.150,
    ),
)


class ContinuousRollerSeparatorSpawner(SeparatorDemoSpawner):
    """Generate parcels using the longitudinal opening as the class rule."""

    def __init__(self) -> None:
        super().__init__()
        self.get_logger().info(
            "Continuous-roller classification active: "
            "projection_x < cutoff routes to LOWER"
        )

    def _weighted_profile(
        self,
        expected_lower: bool,
    ) -> BoxProfile:
        profiles = (
            CONTINUOUS_ROLLER_LOWER_PROFILES
            if expected_lower
            else UPPER_PROFILES
        )
        return self.rng.choices(
            profiles,
            weights=[profile.weight for profile in profiles],
            k=1,
        )[0]

    def _make_box(
        self,
        spot_index: int,
        expected_lower: bool,
    ) -> tuple[
        BoxSpec,
        tuple[float, float, float],
        float,
        float,
        str,
    ]:
        y = self.SPAWN_SPOTS_M[spot_index]

        for _ in range(2500):
            profile = self._weighted_profile(expected_lower)
            size_x = self.rng.uniform(*profile.size_x)
            size_y = self.rng.uniform(*profile.size_y)
            size_z = self.rng.uniform(*profile.size_z)
            yaw = math.radians(
                self.rng.uniform(*profile.yaw_deg)
            )

            projection_x, projection_y = self._projections(
                size_x,
                size_y,
                yaw,
            )

            # With one roller spanning the whole 2.48 m width, there is no
            # transverse Y opening. Falling is governed by the footprint along
            # the direction of travel and the 100 mm gap between roller rows.
            # The LOWER boundary stays at 70 mm; the extra 30 mm is physical
            # clearance. UPPER parcels must start at 110 mm, retaining a 10 mm
            # safety margin above the enlarged opening.
            classified_lower = projection_x < self.cutoff
            if classified_lower != expected_lower:
                continue
            if (
                not expected_lower
                and projection_x < self.upper_safety_projection
            ):
                continue
            if (
                abs(y) + projection_y / 2.0
                > self.half_width - 0.015
            ):
                continue

            volume = size_x * size_y * size_z
            surface_area = 2.0 * (
                size_x * size_y
                + size_x * size_z
                + size_y * size_z
            )
            mass = self._mass(
                profile,
                volume,
                surface_area,
            )
            box = BoxSpec(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=spot_index,
            )
            if expected_lower:
                color = (
                    self.rng.uniform(0.82, 0.98),
                    self.rng.uniform(0.12, 0.45),
                    self.rng.uniform(0.04, 0.12),
                )
            else:
                color = (
                    self.rng.uniform(0.08, 0.25),
                    self.rng.uniform(0.45, 0.85),
                    self.rng.uniform(0.18, 0.85),
                )
            return (
                box,
                color,
                projection_x,
                projection_y,
                profile.name,
            )

        raise RuntimeError(
            f"Could not generate a fitting box for spot {spot_index}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContinuousRollerSeparatorSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
