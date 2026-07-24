#!/usr/bin/env python3
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
import math
import random
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from singulator_sim.box_model import BoxSpec


@dataclass(frozen=True)
class BoxProfile:
    """Dimension, yaw and mass ranges for one parcel family."""

    name: str
    weight: float
    size_x: tuple[float, float]
    size_y: tuple[float, float]
    size_z: tuple[float, float]
    yaw_deg: tuple[float, float]
    density: tuple[float, float]
    minimum_mass_kg: float


LOWER_PROFILES = (
    BoxProfile(
        "micro_parcel",
        0.30,
        (0.035, 0.100),
        (0.015, 0.060),
        (0.010, 0.085),
        (-30.0, 30.0),
        (220.0, 950.0),
        0.080,
    ),
    BoxProfile(
        "long_narrow",
        0.28,
        (0.250, 0.400),
        (0.025, 0.055),
        (0.025, 0.120),
        (-7.0, 7.0),
        (180.0, 850.0),
        0.200,
    ),
    BoxProfile(
        "flat_strip",
        0.18,
        (0.160, 0.380),
        (0.020, 0.055),
        (0.010, 0.030),
        (-8.0, 8.0),
        (250.0, 1050.0),
        0.120,
    ),
    BoxProfile(
        "tall_slender",
        0.14,
        (0.100, 0.240),
        (0.025, 0.060),
        (0.150, 0.280),
        (-8.0, 8.0),
        (160.0, 700.0),
        0.250,
    ),
    BoxProfile(
        "near_cutoff",
        0.10,
        (0.090, 0.220),
        (0.055, 0.069),
        (0.025, 0.140),
        (-4.0, 4.0),
        (180.0, 900.0),
        0.150,
    ),
)

UPPER_PROFILES = (
    BoxProfile(
        "medium_carton",
        0.25,
        (0.120, 0.260),
        (0.090, 0.180),
        (0.050, 0.200),
        (-40.0, 40.0),
        (120.0, 600.0),
        0.350,
    ),
    BoxProfile(
        "large_carton",
        0.18,
        (0.260, 0.400),
        (0.180, 0.320),
        (0.080, 0.280),
        (-35.0, 35.0),
        (90.0, 450.0),
        1.000,
    ),
    BoxProfile(
        "long_parcel",
        0.18,
        (0.300, 0.400),
        (0.090, 0.150),
        (0.040, 0.160),
        (-25.0, 25.0),
        (130.0, 650.0),
        0.450,
    ),
    BoxProfile(
        "flat_panel",
        0.13,
        (0.180, 0.360),
        (0.100, 0.240),
        (0.015, 0.050),
        (-35.0, 35.0),
        (220.0, 900.0),
        0.250,
    ),
    BoxProfile(
        "tall_carton",
        0.13,
        (0.120, 0.240),
        (0.100, 0.200),
        (0.180, 0.280),
        (-30.0, 30.0),
        (100.0, 500.0),
        0.500,
    ),
    BoxProfile(
        "square_carton",
        0.13,
        (0.150, 0.300),
        (0.150, 0.300),
        (0.050, 0.220),
        (-45.0, 45.0),
        (100.0, 550.0),
        0.500,
    ),
)


class SeparatorDemoSpawner(Node):
    """Create a controllable, physically varied flow on ten fixed spots."""

    SPAWN_SPOTS_M = tuple(
        -1.0 + index * (2.0 / 9.0)
        for index in range(10)
    )

    def __init__(self) -> None:
        super().__init__("separator_demo_spawner")
        self.declare_parameter(
            "world_name",
            "infeed_size_separator_demo",
        )
        self.declare_parameter("spawn_x", -3.20)
        self.declare_parameter("belt_top_z", 0.08)
        self.declare_parameter(
            "spawn_clearance_m",
            0.002,
        )
        self.declare_parameter(
            "conveyor_half_width_m",
            1.25,
        )
        self.declare_parameter(
            "target_rate_boxes_per_sec",
            4.0,
        )
        self.declare_parameter(
            "spawn_mode",
            "continuous",
        )
        self.declare_parameter("maximum_items", 100)
        self.declare_parameter(
            "small_item_probability",
            0.50,
        )
        self.declare_parameter("cutoff_m", 0.070)
        self.declare_parameter(
            "upper_safety_projection_m",
            0.090,
        )
        self.declare_parameter("seed", 42)
        self.declare_parameter(
            "service_timeout_ms",
            5000,
        )
        self.declare_parameter(
            "statistics_every_items",
            20,
        )
        self.declare_parameter(
            "box_restitution",
            0.02,
        )
        self.declare_parameter(
            "bounce_capture_velocity_mps",
            0.35,
        )
        self.declare_parameter(
            "linear_velocity_decay",
            0.05,
        )
        self.declare_parameter(
            "angular_velocity_decay",
            0.30,
        )
        self.declare_parameter(
            "contact_max_correcting_velocity_mps",
            0.05,
        )

        self.world_name = str(
            self.get_parameter("world_name").value
        )
        self.spawn_x = float(
            self.get_parameter("spawn_x").value
        )
        self.belt_top_z = float(
            self.get_parameter("belt_top_z").value
        )
        self.spawn_clearance = float(
            self.get_parameter(
                "spawn_clearance_m"
            ).value
        )
        self.half_width = float(
            self.get_parameter(
                "conveyor_half_width_m"
            ).value
        )
        self.target_rate = float(
            self.get_parameter(
                "target_rate_boxes_per_sec"
            ).value
        )
        self.spawn_mode = str(
            self.get_parameter("spawn_mode").value
        ).lower()
        self.maximum_items = int(
            self.get_parameter("maximum_items").value
        )
        self.small_probability = float(
            self.get_parameter(
                "small_item_probability"
            ).value
        )
        self.cutoff = float(
            self.get_parameter("cutoff_m").value
        )
        self.upper_safety_projection = float(
            self.get_parameter(
                "upper_safety_projection_m"
            ).value
        )
        self.service_timeout_ms = int(
            self.get_parameter(
                "service_timeout_ms"
            ).value
        )
        self.statistics_every_items = int(
            self.get_parameter(
                "statistics_every_items"
            ).value
        )
        self.box_restitution = float(
            self.get_parameter(
                "box_restitution"
            ).value
        )
        self.bounce_threshold = float(
            self.get_parameter(
                "bounce_capture_velocity_mps"
            ).value
        )
        self.linear_decay = float(
            self.get_parameter(
                "linear_velocity_decay"
            ).value
        )
        self.angular_decay = float(
            self.get_parameter(
                "angular_velocity_decay"
            ).value
        )
        self.contact_max_vel = float(
            self.get_parameter(
                "contact_max_correcting_velocity_mps"
            ).value
        )
        seed = int(self.get_parameter("seed").value)

        self._validate_parameters()
        self.rng = random.Random(seed)
        self.session_id = int(time.time())
        self.item_index = 0
        self.created_count = 0
        self.expected_upper = 0
        self.expected_lower = 0
        self.skipped_busy = 0
        self.started_sim_ns: int | None = None
        self.profile_counts: dict[str, int] = {}

        self.state_lock = threading.Lock()
        self.spawn_in_progress = False
        self.spawn_pool = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="separator_spawn",
        )
        self.timer = self.create_timer(
            1.0 / self.target_rate,
            self._on_timer,
        )

        spots = ", ".join(
            f"{value:.3f}"
            for value in self.SPAWN_SPOTS_M
        )
        self.get_logger().info(
            "Separator flow configured: "
            f"mode={self.spawn_mode}, "
            f"target_rate={self.target_rate:.2f} item/s, "
            f"lower_probability={self.small_probability:.2f}, "
            f"cutoff={self.cutoff * 1000:.0f} mm"
        )
        self.get_logger().info(
            "Box impact model: "
            f"restitution={self.box_restitution:.3f}, "
            f"bounce_threshold={self.bounce_threshold:.2f} m/s, "
            f"velocity_decay={self.linear_decay:.2f}/"
            f"{self.angular_decay:.2f}, "
            f"spawn_clearance="
            f"{self.spawn_clearance * 1000:.1f} mm"
        )
        self.get_logger().info(
            f"Ten fixed centre-of-mass spots Y=[{spots}] m"
        )

    def _validate_parameters(self) -> None:
        if self.target_rate <= 0.0:
            raise ValueError(
                "target_rate_boxes_per_sec must be positive"
            )
        if self.spawn_mode not in {
            "continuous",
            "finite",
        }:
            raise ValueError(
                "spawn_mode must be continuous or finite"
            )
        if self.maximum_items <= 0:
            raise ValueError(
                "maximum_items must be positive"
            )
        if not 0.0 <= self.small_probability <= 1.0:
            raise ValueError(
                "small_item_probability must be in [0, 1]"
            )
        if self.cutoff <= 0.0:
            raise ValueError(
                "cutoff_m must be positive"
            )
        if self.upper_safety_projection < self.cutoff:
            raise ValueError(
                "upper_safety_projection_m cannot be below cutoff_m"
            )
        if self.half_width <= 0.0:
            raise ValueError(
                "conveyor_half_width_m must be positive"
            )
        if self.spawn_clearance < 0.0:
            raise ValueError(
                "spawn_clearance_m cannot be negative"
            )
        if self.statistics_every_items <= 0:
            raise ValueError(
                "statistics_every_items must be positive"
            )
        if not 0.0 <= self.box_restitution <= 1.0:
            raise ValueError(
                "box_restitution must be in [0, 1]"
            )
        for name, value in (
            (
                "bounce_capture_velocity_mps",
                self.bounce_threshold,
            ),
            (
                "linear_velocity_decay",
                self.linear_decay,
            ),
            (
                "angular_velocity_decay",
                self.angular_decay,
            ),
            (
                "contact_max_correcting_velocity_mps",
                self.contact_max_vel,
            ),
        ):
            if value < 0.0:
                raise ValueError(
                    f"{name} cannot be negative"
                )

    @staticmethod
    def _projections(
        size_x: float,
        size_y: float,
        yaw: float,
    ) -> tuple[float, float]:
        projected_x = (
            abs(size_x * math.cos(yaw))
            + abs(size_y * math.sin(yaw))
        )
        projected_y = (
            abs(size_x * math.sin(yaw))
            + abs(size_y * math.cos(yaw))
        )
        return projected_x, projected_y

    def _weighted_profile(
        self,
        expected_lower: bool,
    ) -> BoxProfile:
        profiles = (
            LOWER_PROFILES
            if expected_lower
            else UPPER_PROFILES
        )
        return self.rng.choices(
            profiles,
            weights=[
                profile.weight
                for profile in profiles
            ],
            k=1,
        )[0]

    def _mass(
        self,
        profile: BoxProfile,
        volume: float,
        surface_area: float,
    ) -> float:
        bulk_density = self.rng.uniform(
            *profile.density
        )
        cardboard_areal_density = self.rng.uniform(
            0.30,
            0.75,
        )
        contents_mass = volume * bulk_density
        shell_mass = (
            surface_area
            * cardboard_areal_density
        )
        return max(
            profile.minimum_mass_kg,
            min(5.0, contents_mass + shell_mass),
        )

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
            profile = self._weighted_profile(
                expected_lower
            )
            size_x = self.rng.uniform(
                *profile.size_x
            )
            size_y = self.rng.uniform(
                *profile.size_y
            )
            size_z = self.rng.uniform(
                *profile.size_z
            )
            yaw = math.radians(
                self.rng.uniform(*profile.yaw_deg)
            )

            projection_x, projection_y = (
                self._projections(
                    size_x,
                    size_y,
                    yaw,
                )
            )
            minimum_projection = min(
                projection_x,
                projection_y,
            )
            classified_lower = (
                minimum_projection < self.cutoff
            )
            if classified_lower != expected_lower:
                continue
            if (
                not expected_lower
                and minimum_projection
                < self.upper_safety_projection
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

    def _with_realistic_contact(
        self,
        sdf: str,
    ) -> str:
        link_marker = (
            '    <link name="base_link">\n'
        )
        if link_marker not in sdf:
            raise RuntimeError(
                "Box SDF does not contain base_link"
            )
        link_replacement = (
            link_marker
            + "\n"
            + "      <velocity_decay>\n"
            + f"        <linear>{self.linear_decay:.6f}</linear>\n"
            + f"        <angular>{self.angular_decay:.6f}</angular>\n"
            + "      </velocity_decay>\n"
        )
        sdf = sdf.replace(
            link_marker,
            link_replacement,
            1,
        )
        sdf = sdf.replace(
            "<allow_auto_disable>false</allow_auto_disable>",
            "<allow_auto_disable>true</allow_auto_disable>",
            1,
        )

        surface_marker = "        <surface>\n"
        if surface_marker not in sdf:
            raise RuntimeError(
                "Box SDF does not contain collision surface"
            )
        surface_replacement = (
            surface_marker
            + "          <bounce>\n"
            + "            <restitution_coefficient>"
            + f"{self.box_restitution:.6f}"
            + "</restitution_coefficient>\n"
            + f"            <threshold>{self.bounce_threshold:.6f}"
            + "</threshold>\n"
            + "          </bounce>\n"
        )
        sdf = sdf.replace(
            surface_marker,
            surface_replacement,
            1,
        )

        close_marker = "        </surface>\n"
        if close_marker not in sdf:
            raise RuntimeError(
                "Box SDF surface is not closed"
            )
        contact_block = (
            "          <contact>\n"
            "            <ode>\n"
            "              <kp>50000000</kp>\n"
            "              <kd>50000</kd>\n"
            f"              <max_vel>{self.contact_max_vel:.6f}"
            "</max_vel>\n"
            "              <min_depth>0.0005</min_depth>\n"
            "            </ode>\n"
            "            <bullet>\n"
            "              <kp>50000000</kp>\n"
            "              <kd>50000</kd>\n"
            "              <split_impulse>true</split_impulse>\n"
            "              <split_impulse_penetration_threshold>"
            "-0.002"
            "</split_impulse_penetration_threshold>\n"
            "            </bullet>\n"
            "          </contact>\n"
            + close_marker
        )
        return sdf.replace(
            close_marker,
            contact_block,
            1,
        )

    def _on_timer(self) -> None:
        if (
            self.spawn_mode == "finite"
            and self.item_index
            >= self.maximum_items
        ):
            self.timer.cancel()
            self.get_logger().info(
                "Finite separator flow finished; "
                "existing boxes keep moving"
            )
            return

        with self.state_lock:
            if self.spawn_in_progress:
                self.skipped_busy += 1
                if self.skipped_busy % 20 == 1:
                    self.get_logger().warning(
                        "Spawn service is still busy; "
                        "a flow tick was skipped"
                    )
                return
            self.spawn_in_progress = True

        expected_lower = (
            self.rng.random()
            < self.small_probability
        )
        spot_index = self.rng.randrange(
            len(self.SPAWN_SPOTS_M)
        )
        try:
            (
                box,
                color,
                projection_x,
                projection_y,
                profile,
            ) = self._make_box(
                spot_index,
                expected_lower,
            )
        except Exception:
            with self.state_lock:
                self.spawn_in_progress = False
            raise

        route = (
            "lower"
            if expected_lower
            else "upper"
        )
        model_name = (
            f"box_separator_{self.session_id}_"
            f"n{self.item_index:07d}_exp_{route}_"
            f"{profile}_spot{spot_index:02d}"
        )
        self.item_index += 1

        self.get_logger().info(
            f"Spawn {model_name}: profile={profile}, "
            f"{box.size_x * 1000:.0f}x"
            f"{box.size_y * 1000:.0f}x"
            f"{box.size_z * 1000:.0f} mm, "
            f"mass={box.mass:.3f} kg, "
            f"yaw={math.degrees(box.yaw):.1f} deg, "
            f"projection={projection_x * 1000:.0f}x"
            f"{projection_y * 1000:.0f} mm, "
            f"expected={route.upper()}"
        )

        future = self.spawn_pool.submit(
            self._spawn_item,
            model_name,
            box,
            color,
            expected_lower,
            profile,
        )
        future.add_done_callback(
            self._on_spawn_finished
        )

    def _spawn_item(
        self,
        model_name: str,
        box: BoxSpec,
        color: tuple[float, float, float],
        expected_lower: bool,
        profile: str,
    ) -> tuple[bool, bool, str]:
        sdf = box.to_sdf(
            model_name=model_name,
            color=color,
        )
        sdf = self._with_realistic_contact(sdf)
        spawn_z = (
            self.belt_top_z
            + box.size_z / 2.0
            + self.spawn_clearance
        )
        escaped_sdf = json.dumps(sdf)
        request = "\n".join(
            (
                "data {",
                f"  sdf: {escaped_sdf}",
                f'  name: "{model_name}"',
                "  allow_renaming: false",
                "  pose {",
                "    position {",
                f"      x: {self.spawn_x:.9f}",
                f"      y: {box.y:.9f}",
                f"      z: {spawn_z:.9f}",
                "    }",
                "  }",
                "}",
            )
        )
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/create_multiple",
            "--reqtype",
            "gz.msgs.EntityFactory_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self.service_timeout_ms),
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=(
                    self.service_timeout_ms / 1000.0
                    + 2.0
                ),
                check=False,
            )
        except (
            OSError,
            subprocess.TimeoutExpired,
        ) as error:
            self.get_logger().error(
                f"Gazebo spawn failed: {error}"
            )
            return False, expected_lower, profile

        success = (
            result.returncode == 0
            and "data: true"
            in result.stdout.lower()
        )
        if not success:
            self.get_logger().error(
                "Gazebo rejected separator item. "
                f"stdout={result.stdout.strip()} "
                f"stderr={result.stderr.strip()}"
            )
        return success, expected_lower, profile

    def _on_spawn_finished(
        self,
        future: Future,
    ) -> None:
        try:
            success, expected_lower, profile = (
                future.result()
            )
            if success:
                self.created_count += 1
                self.profile_counts[profile] = (
                    self.profile_counts.get(
                        profile,
                        0,
                    )
                    + 1
                )
                if expected_lower:
                    self.expected_lower += 1
                else:
                    self.expected_upper += 1
                if self.started_sim_ns is None:
                    self.started_sim_ns = (
                        self.get_clock()
                        .now()
                        .nanoseconds
                    )
                if (
                    self.created_count
                    % self.statistics_every_items
                    == 0
                ):
                    self._log_statistics()
        except Exception as error:
            self.get_logger().error(
                f"Separator spawn task failed: {error}"
            )
        finally:
            with self.state_lock:
                self.spawn_in_progress = False

    def _log_statistics(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (
            (now_ns - self.started_sim_ns) / 1e9
            if self.started_sim_ns is not None
            else 0.0
        )
        actual_rate = (
            (self.created_count - 1) / elapsed
            if elapsed > 0.0
            else 0.0
        )
        top_profiles = sorted(
            self.profile_counts.items(),
            key=lambda item: (-item[1], item[0]),
        )
        profile_summary = ",".join(
            f"{name}:{count}"
            for name, count in top_profiles
        )
        self.get_logger().info(
            "Spawner statistics: "
            f"created={self.created_count}, "
            f"expected_upper={self.expected_upper}, "
            f"expected_lower={self.expected_lower}, "
            f"actual_rate={actual_rate:.2f} item/s, "
            f"skipped_busy={self.skipped_busy}, "
            f"profiles=[{profile_summary}]"
        )

    def destroy_node(self) -> bool:
        self.timer.cancel()
        self.spawn_pool.shutdown(
            wait=False,
            cancel_futures=True,
        )
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeparatorDemoSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
