#!/usr/bin/env python3
"""Generate the dense, low-shock roller throat used by the singulator."""

from __future__ import annotations

from pathlib import Path
import math

THROAT_LENGTH = 0.80
ENTRY_WIDTH = 0.760
EXIT_WIDTH = 0.600
ROLLER_COUNT = 21
ROLLER_PITCH = THROAT_LENGTH / (ROLLER_COUNT - 1)
ROLLER_RADIUS = 0.012
ROLLER_COLLISION_HEIGHT = 0.016
ROLLER_COLLISION_WIDTH = 0.022
ROLLER_LENGTH = 0.420
ROLLER_CENTRE_Y = 0.170
STEERING_ANGLE_RAD = math.radians(6.0)
ROLLER_TOP_Z = 0.080
ROLLER_COLLISION_Z = ROLLER_TOP_Z - ROLLER_COLLISION_HEIGHT / 2.0
ROLLER_VISUAL_Z = ROLLER_TOP_Z - ROLLER_RADIUS

MU = 0.8
MU2 = 0.8
MAX_SPEED = 3.0
MAX_ACCELERATION = 6.0
MAX_JERK = 30.0

TRANSFER_TOP_Z = 0.079
TRANSFER_HEIGHT = 0.006
TRANSFER_Z = TRANSFER_TOP_Z - TRANSFER_HEIGHT / 2.0
TRANSFER_MU = 0.05
ENTRY_TRANSFER_X = -0.410
EXIT_TRANSFER_X = 0.410
TRANSFER_LENGTH = 0.040

UNDERTRAY_TOP_Z = 0.075
UNDERTRAY_HEIGHT = 0.008
UNDERTRAY_Z = UNDERTRAY_TOP_Z - UNDERTRAY_HEIGHT / 2.0
UNDERTRAY_MU = 0.03


def friction(mu: float, mu2: float) -> str:
    return (
        "<surface><friction><ode>"
        f"<mu>{mu}</mu><mu2>{mu2}</mu2><fdir1>1 0 0</fdir1>"
        "</ode></friction></surface>"
    )


def roller_block(side: str, index: int, x: float) -> str:
    is_left = side == "left"
    y = -ROLLER_CENTRE_Y if is_left else ROLLER_CENTRE_Y
    yaw = math.pi / 2.0 + STEERING_ANGLE_RAD if is_left else math.pi / 2.0 - STEERING_ANGLE_RAD
    return f"""
      <collision name="roller_{index:02d}_collision">
        <pose>{x:.6f} {y:.6f} {ROLLER_COLLISION_Z:.6f} 0 0 {yaw:.9f}</pose>
        <max_contacts>8</max_contacts>
        <geometry><box><size>{ROLLER_LENGTH:.6f} {ROLLER_COLLISION_WIDTH:.6f} {ROLLER_COLLISION_HEIGHT:.6f}</size></box></geometry>
        {friction(MU, MU2)}
      </collision>
      <visual name="roller_{index:02d}_visual">
        <pose>{x:.6f} {y:.6f} {ROLLER_VISUAL_Z:.6f} 0 {math.pi / 2.0:.9f} {yaw:.9f}</pose>
        <geometry><cylinder><radius>{ROLLER_RADIUS:.6f}</radius><length>{ROLLER_LENGTH:.6f}</length></cylinder></geometry>
        <material><ambient>0.72 0.72 0.74 1</ambient><diffuse>0.72 0.72 0.74 1</diffuse><specular>0.35 0.35 0.35 1</specular></material>
      </visual>"""


def track_controller(side: str) -> str:
    orientation = STEERING_ANGLE_RAD if side == "left" else -STEERING_ANGLE_RAD
    return f"""
    <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
      <link>{side}_rollers</link>
      <track_orientation>0 0 {orientation:.9f}</track_orientation>
      <velocity_topic>/singulator/throat/{side}/cmd_vel</velocity_topic>
      <odometry_topic>/singulator/throat/{side}/odometry</odometry_topic>
      <odometry_publish_frequency>5</odometry_publish_frequency>
      <min_velocity>-{MAX_SPEED}</min_velocity><max_velocity>{MAX_SPEED}</max_velocity>
      <min_acceleration>-{MAX_ACCELERATION}</min_acceleration><max_acceleration>{MAX_ACCELERATION}</max_acceleration>
      <min_jerk>-{MAX_JERK}</min_jerk><max_jerk>{MAX_JERK}</max_jerk>
      <max_command_age>2.0</max_command_age>
    </plugin>"""


def roller_link(side: str) -> str:
    start_x = -THROAT_LENGTH / 2.0
    rollers = "\n".join(
        roller_block(side, index, start_x + index * ROLLER_PITCH)
        for index in range(ROLLER_COUNT)
    )
    return f"""
    <link name="{side}_rollers">
{rollers}
    </link>
{track_controller(side)}"""


def generate_model() -> str:
    guide_y = (ENTRY_WIDTH + EXIT_WIDTH) / 4.0 + 0.0125
    guide_yaw = math.atan2((ENTRY_WIDTH - EXIT_WIDTH) / 2.0, THROAT_LENGTH)
    guide_length = math.hypot(THROAT_LENGTH, (ENTRY_WIDTH - EXIT_WIDTH) / 2.0) + 0.04

    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="roller_throat">
    <static>true</static>
    <link name="base">
      <collision name="base_collision"><pose>0 0 0.035 0 0 0</pose><geometry><box><size>0.86 0.76 0.07</size></box></geometry></collision>
      <visual name="base_visual"><pose>0 0 0.035 0 0 0</pose><geometry><box><size>0.86 0.76 0.07</size></box></geometry><material><ambient>0.16 0.16 0.16 1</ambient><diffuse>0.22 0.22 0.22 1</diffuse></material></visual>

      <!-- Plates are 1 mm below the moving surfaces and only catch an edge during transfer. -->
      <collision name="entry_transfer_collision">
        <pose>{ENTRY_TRANSFER_X:.6f} 0 {TRANSFER_Z:.6f} 0 0 0</pose>
        <geometry><box><size>{TRANSFER_LENGTH:.6f} {ENTRY_WIDTH:.6f} {TRANSFER_HEIGHT:.6f}</size></box></geometry>
        {friction(TRANSFER_MU, TRANSFER_MU)}
      </collision>
      <visual name="entry_transfer_visual">
        <pose>{ENTRY_TRANSFER_X:.6f} 0 {TRANSFER_Z:.6f} 0 0 0</pose>
        <geometry><box><size>{TRANSFER_LENGTH:.6f} {ENTRY_WIDTH:.6f} {TRANSFER_HEIGHT:.6f}</size></box></geometry>
        <material><ambient>0.45 0.45 0.48 1</ambient><diffuse>0.55 0.55 0.58 1</diffuse></material>
      </visual>

      <collision name="exit_transfer_collision">
        <pose>{EXIT_TRANSFER_X:.6f} 0 {TRANSFER_Z:.6f} 0 0 0</pose>
        <geometry><box><size>{TRANSFER_LENGTH:.6f} {EXIT_WIDTH:.6f} {TRANSFER_HEIGHT:.6f}</size></box></geometry>
        {friction(TRANSFER_MU, TRANSFER_MU)}
      </collision>
      <visual name="exit_transfer_visual">
        <pose>{EXIT_TRANSFER_X:.6f} 0 {TRANSFER_Z:.6f} 0 0 0</pose>
        <geometry><box><size>{TRANSFER_LENGTH:.6f} {EXIT_WIDTH:.6f} {TRANSFER_HEIGHT:.6f}</size></box></geometry>
        <material><ambient>0.45 0.45 0.48 1</ambient><diffuse>0.55 0.55 0.58 1</diffuse></material>
      </visual>

      <!-- A recessed low-friction tray prevents thin products falling between supports. -->
      <collision name="undertray_collision">
        <pose>0 0 {UNDERTRAY_Z:.6f} 0 0 0</pose>
        <geometry><box><size>0.84 0.62 {UNDERTRAY_HEIGHT:.6f}</size></box></geometry>
        {friction(UNDERTRAY_MU, UNDERTRAY_MU)}
      </collision>

      <collision name="left_guide_collision"><pose>0 {-guide_y:.6f} 0.125 0 0 {guide_yaw:.9f}</pose><geometry><box><size>{guide_length:.6f} 0.025 0.18</size></box></geometry></collision>
      <visual name="left_guide_visual"><pose>0 {-guide_y:.6f} 0.125 0 0 {guide_yaw:.9f}</pose><geometry><box><size>{guide_length:.6f} 0.025 0.18</size></box></geometry><material><ambient>0.55 0.55 0.58 1</ambient><diffuse>0.65 0.65 0.68 1</diffuse></material></visual>
      <collision name="right_guide_collision"><pose>0 {guide_y:.6f} 0.125 0 0 {-guide_yaw:.9f}</pose><geometry><box><size>{guide_length:.6f} 0.025 0.18</size></box></geometry></collision>
      <visual name="right_guide_visual"><pose>0 {guide_y:.6f} 0.125 0 0 {-guide_yaw:.9f}</pose><geometry><box><size>{guide_length:.6f} 0.025 0.18</size></box></geometry><material><ambient>0.55 0.55 0.58 1</ambient><diffuse>0.65 0.65 0.68 1</diffuse></material></visual>
    </link>
{roller_link("left")}
{roller_link("right")}
  </model>
</sdf>
"""


def main() -> None:
    output = Path(__file__).resolve().parents[1] / "models" / "roller_throat" / "model.sdf"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(generate_model(), encoding="utf-8")
    print(f"Created roller throat: {output}")
    print(f"Rollers per side: {ROLLER_COUNT}, pitch={ROLLER_PITCH:.3f} m")
    print(f"Steering angle: {math.degrees(STEERING_ANGLE_RAD):.1f} deg")
    print(f"Transfer top: {TRANSFER_TOP_Z:.3f} m; undertray top: {UNDERTRAY_TOP_Z:.3f} m")


if __name__ == "__main__":
    main()
