#!/usr/bin/env python3

import argparse
from pathlib import Path


PITCH_X = 0.360
PITCH_Y = 0.175

ACTIVE_X = 0.3563
ACTIVE_Y = 0.17275

CELL_HEIGHT = 0.08
BELT_TOP_Z = CELL_HEIGHT

MAX_SPEED = 2.0
MAX_COMMAND_AGE = 2.0


def cell_sdf(row: int, col: int, x: float, y: float) -> str:
    cell_id = f"r{row:02d}_c{col:02d}"

    velocity_topic = (
        f"/singulator/cell/{cell_id}/cmd_vel"
    )

    odometry_topic = (
        f"/singulator/cell/{cell_id}/odometry"
    )

    return f"""
    <model name="cell_{cell_id}">
      <static>true</static>

      <pose>{x:.6f} {y:.6f} 0 0 0 0</pose>

      <link name="belt">
        <pose>0 0 {CELL_HEIGHT / 2:.6f} 0 0 0</pose>

        <collision name="belt_collision">
          <geometry>
            <box>
              <size>
                {ACTIVE_X} {ACTIVE_Y} {CELL_HEIGHT}
              </size>
            </box>
          </geometry>

          <surface>
            <friction>
              <ode>
                <mu>0.7</mu>
                <mu2>0.7</mu2>
                <fdir1>1 0 0</fdir1>
              </ode>
            </friction>
          </surface>
        </collision>

        <visual name="belt_visual">
          <geometry>
            <box>
              <size>
                {ACTIVE_X} {ACTIVE_Y} {CELL_HEIGHT}
              </size>
            </box>
          </geometry>

          <material>
            <ambient>0.04 0.18 0.70 1</ambient>
            <diffuse>0.05 0.30 1.00 1</diffuse>
          </material>
        </visual>
      </link>

      <plugin
        filename="gz-sim-track-controller-system"
        name="gz::sim::systems::TrackController">

        <link>belt</link>

        <!-- Положительная скорость направлена вдоль +X -->
        <track_orientation>0 0 0</track_orientation>

        <velocity_topic>
          {velocity_topic}
        </velocity_topic>

        <odometry_topic>
          {odometry_topic}
        </odometry_topic>

        <odometry_publish_frequency>
          20
        </odometry_publish_frequency>

        <min_velocity>-{MAX_SPEED}</min_velocity>
        <max_velocity>{MAX_SPEED}</max_velocity>

        <min_acceleration>-2.5</min_acceleration>
        <max_acceleration>2.5</max_acceleration>

        <min_jerk>-10.0</min_jerk>
        <max_jerk>10.0</max_jerk>

        <max_command_age>
          {MAX_COMMAND_AGE}
        </max_command_age>
      </plugin>
    </model>
"""


def generate_world(rows: int, cols: int) -> str:
    if rows <= 0 or cols <= 0:
        raise ValueError("rows и cols должны быть положительными")

    cells = []

    for row in range(rows):
        x = (row - (rows - 1) / 2.0) * PITCH_X

        for col in range(cols):
            y = (col - (cols - 1) / 2.0) * PITCH_Y

            cells.append(
                cell_sdf(
                    row=row,
                    col=col,
                    x=x,
                    y=y,
                )
            )

    platform_x = rows * PITCH_X + 0.8
    platform_y = cols * PITCH_Y + 0.4

    first_row_x = -(rows - 1) * PITCH_X / 2.0

    # Коробка немного смещена и повёрнута.
    # Это убирает идеально вырожденный симметричный контакт.
    box_x = first_row_x - 0.02
    box_y = 0.012
    box_yaw = 0.03

    return f"""<?xml version="1.0"?>
<sdf version="1.10">

  <world name="matrix_{rows}x{cols}">

    <gravity>0 0 -9.81</gravity>

    <physics name="physics_1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <pose>0 0 5 0 0 0</pose>

      <cast_shadows>true</cast_shadows>

      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>

      <direction>-0.3 0.2 -1.0</direction>
    </light>

    <model name="platform">
      <static>true</static>

      <pose>0 0 -0.025 0 0 0</pose>

      <link name="platform_link">

        <collision name="platform_collision">
          <geometry>
            <box>
              <size>{platform_x} {platform_y} 0.05</size>
            </box>
          </geometry>
        </collision>

        <visual name="platform_visual">
          <geometry>
            <box>
              <size>{platform_x} {platform_y} 0.05</size>
            </box>
          </geometry>

          <material>
            <ambient>0.22 0.22 0.22 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
          </material>
        </visual>

      </link>
    </model>

    {''.join(cells)}

    <model name="test_box">

      <pose>
        {box_x:.6f} {box_y:.6f} 0.14 0 0 {box_yaw}
      </pose>

      <allow_auto_disable>true</allow_auto_disable>

      <link name="base_link">

        <inertial>
          <mass>1.5</mass>

          <inertia>
            <ixx>0.01305</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>

            <iyy>0.0096125</iyy>
            <iyz>0</iyz>

            <izz>0.0190625</izz>
          </inertia>
        </inertial>

        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.25 0.30 0.12</size>
            </box>
          </geometry>

          <surface>
            <friction>
              <ode>
                <mu>0.7</mu>
                <mu2>0.7</mu2>
              </ode>
            </friction>
          </surface>
        </collision>

        <visual name="box_visual">
          <geometry>
            <box>
              <size>0.25 0.30 0.12</size>
            </box>
          </geometry>

          <material>
            <ambient>0.55 0.22 0.06 1</ambient>
            <diffuse>0.80 0.35 0.08 1</diffuse>
          </material>
        </visual>

      </link>
    </model>

    <gui fullscreen="0">

      <plugin filename="MinimalScene" name="3D View">

        <gz-gui>
          <title>3D View</title>

          <property
            type="bool"
            key="showTitleBar">
            false
          </property>

          <property
            type="string"
            key="state">
            docked
          </property>
        </gz-gui>

        <engine>ogre2</engine>
        <scene>scene</scene>

        <ambient_light>0.6 0.6 0.6</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>

        <camera_pose>
          -1.8 0 1.6 0 0.65 0
        </camera_pose>

      </plugin>

      <plugin
        filename="GzSceneManager"
        name="Scene Manager">

        <gz-gui>
          <property key="resizable" type="bool">
            false
          </property>

          <property key="width" type="double">
            5
          </property>

          <property key="height" type="double">
            5
          </property>

          <property key="state" type="string">
            floating
          </property>

          <property key="showTitleBar" type="bool">
            false
          </property>
        </gz-gui>

      </plugin>

      <plugin filename="EntityTree" name="Entity tree"/>
      <plugin filename="WorldControl" name="World control"/>
      <plugin filename="WorldStats" name="World stats"/>

    </gui>

  </world>

</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--rows",
        type=int,
        required=True,
    )

    parser.add_argument(
        "--cols",
        type=int,
        required=True,
    )

    parser.add_argument(
        "--output",
        type=Path,
        required=True,
    )

    args = parser.parse_args()

    world = generate_world(
        rows=args.rows,
        cols=args.cols,
    )

    args.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    args.output.write_text(
        world,
        encoding="utf-8",
    )

    print(f"Создан мир: {args.output}")
    print(f"Размер матрицы: {args.rows}x{args.cols}")
    print(f"Количество приводов: {args.rows * args.cols}")


if __name__ == "__main__":
    main()
