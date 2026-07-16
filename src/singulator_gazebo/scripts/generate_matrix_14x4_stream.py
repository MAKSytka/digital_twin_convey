#!/usr/bin/env python3

from pathlib import Path


ROWS = 14
COLS = 4

# Размер активной поверхности одной ячейки.
CELL_X = 0.360
CELL_Y = 0.175

# Новые зазоры.
GAP_X = 0.020
GAP_Y = 0.020

PITCH_X = CELL_X + GAP_X
PITCH_Y = CELL_Y + GAP_Y

CELL_HEIGHT = 0.080
BELT_TOP_Z = CELL_HEIGHT

MU_LONGITUDINAL = 0.8
MU_TRANSVERSE = 0.2

MAX_SPEED = 2.0
MAX_COMMAND_AGE = 2.0

INFEED_LENGTH = 1.20
OUTFEED_LENGTH = 1.00

MATRIX_LENGTH = ROWS * CELL_X + (ROWS - 1) * GAP_X
MATRIX_WIDTH = COLS * CELL_Y + (COLS - 1) * GAP_Y

MATRIX_MIN_X = -MATRIX_LENGTH / 2.0
MATRIX_MAX_X = MATRIX_LENGTH / 2.0

INFEED_CENTER_X = (
    MATRIX_MIN_X
    - GAP_X
    - INFEED_LENGTH / 2.0
)

OUTFEED_CENTER_X = (
    MATRIX_MAX_X
    + GAP_X
    + OUTFEED_LENGTH / 2.0
)


def track_controller(
    velocity_topic: str,
    odometry_topic: str,
) -> str:
    return f"""
      <plugin
        filename="gz-sim-track-controller-system"
        name="gz::sim::systems::TrackController">

        <link>belt</link>

        <!-- Положительное направление скорости: глобальная ось +X -->
        <track_orientation>0 0 0</track_orientation>

        <velocity_topic>{velocity_topic}</velocity_topic>
        <odometry_topic>{odometry_topic}</odometry_topic>

        <odometry_publish_frequency>
          1
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
"""


def belt_model(
    name: str,
    x: float,
    y: float,
    size_x: float,
    size_y: float,
    velocity_topic: str,
    odometry_topic: str,
    color: tuple[float, float, float],
) -> str:
    red, green, blue = color

    return f"""
    <model name="{name}">
      <static>true</static>

      <pose>{x:.6f} {y:.6f} 0 0 0 0</pose>

      <link name="belt">
        <pose>0 0 {CELL_HEIGHT / 2.0:.6f} 0 0 0</pose>

        <collision name="belt_collision">
	<max_contacts>4</max_contacts>
          <geometry>
            <box>
              <size>{size_x:.6f} {size_y:.6f} {CELL_HEIGHT:.6f}</size>
            </box>
          </geometry>

          <surface>
            <friction>
              <ode>
                <!-- Продольное сцепление вдоль X -->
                <mu>{MU_LONGITUDINAL}</mu>

                <!-- Поперечное сцепление вдоль Y -->
                <mu2>{MU_TRANSVERSE}</mu2>

                <fdir1>1 0 0</fdir1>
              </ode>
            </friction>
          </surface>
        </collision>

        <visual name="belt_visual">
          <geometry>
            <box>
              <size>{size_x:.6f} {size_y:.6f} {CELL_HEIGHT:.6f}</size>
            </box>
          </geometry>

          <material>
            <ambient>{red:.3f} {green:.3f} {blue:.3f} 1</ambient>
            <diffuse>{red:.3f} {green:.3f} {blue:.3f} 1</diffuse>
          </material>
        </visual>

      </link>

      {track_controller(velocity_topic, odometry_topic)}

    </model>
"""


def matrix_cells() -> str:
    result = []

    # r12 и r13 создаются первыми.
    # Это предотвращает потерю контактной скорости
    # у последних TrackController.
    row_order = [12, 13] + list(range(0, 12))

    for row in row_order:
        x = (
            row - (ROWS - 1) / 2.0
        ) * PITCH_X

        for col in range(COLS):
            y = (
                col - (COLS - 1) / 2.0
            ) * PITCH_Y

            cell_id = f"r{row:02d}_c{col:02d}"

            # Небольшое чередование оттенков помогает видеть границы.
            if (row + col) % 2 == 0:
                color = (0.05, 0.25, 0.95)
            else:
                color = (0.04, 0.18, 0.75)

            result.append(
                belt_model(
                    name=f"cell_{cell_id}",
                    x=x,
                    y=y,
                    size_x=CELL_X,
                    size_y=CELL_Y,
                    velocity_topic=(
                        f"/singulator/cell/{cell_id}/cmd_vel"
                    ),
                    odometry_topic=(
                        f"/singulator/cell/{cell_id}/odometry"
                    ),
                    color=color,
                )
            )

    return "\n".join(result)


def generate_world() -> str:
    complete_length = (
        INFEED_LENGTH
        + GAP_X
        + MATRIX_LENGTH
        + GAP_X
        + OUTFEED_LENGTH
    )

    platform_length = complete_length + 0.6
    platform_width = MATRIX_WIDTH + 0.4

    infeed = belt_model(
        name="infeed_conveyor",
        x=INFEED_CENTER_X,
        y=0.0,
        size_x=INFEED_LENGTH,
        size_y=MATRIX_WIDTH,
        velocity_topic="/singulator/infeed/cmd_vel",
        odometry_topic="/singulator/infeed/odometry",
        color=(0.18, 0.18, 0.18),
    )

    outfeed = belt_model(
        name="outfeed_conveyor",
        x=OUTFEED_CENTER_X,
        y=0.0,
        size_x=OUTFEED_LENGTH,
        size_y=MATRIX_WIDTH,
        velocity_topic="/singulator/outfeed/cmd_vel",
        odometry_topic="/singulator/outfeed/odometry",
        color=(0.18, 0.18, 0.18),
    )

    return f"""<?xml version="1.0"?>
<sdf version="1.10">

  <world name="matrix_14x4_stream">

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
      <pose>0 0 6 0 0 0</pose>
      <cast_shadows>false</cast_shadows>

      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>

      <direction>-0.4 0.2 -1.0</direction>
    </light>

    <!-- Основание находится ниже рабочих поверхностей. -->
    <model name="platform">
      <static>true</static>

      <pose>0 0 -0.025 0 0 0</pose>

      <link name="platform_link">

        <collision name="platform_collision">
          <geometry>
            <box>
              <size>
                {platform_length:.6f}
                {platform_width:.6f}
                0.05
              </size>
            </box>
          </geometry>
        </collision>

        <visual name="platform_visual">
          <geometry>
            <box>
              <size>
                {platform_length:.6f}
                {platform_width:.6f}
                0.05
              </size>
            </box>
          </geometry>

          <material>
            <ambient>0.20 0.20 0.20 1</ambient>
            <diffuse>0.30 0.30 0.30 1</diffuse>
          </material>
        </visual>

      </link>
    </model>

    {infeed}

    {matrix_cells()}

    {outfeed}

    <gui fullscreen="0">

      <plugin filename="MinimalScene" name="3D View">

        <gz-gui>
          <title>3D View</title>

          <property type="bool" key="showTitleBar">
            false
          </property>

          <property type="string" key="state">
            docked
          </property>
        </gz-gui>

        <engine>ogre2</engine>
        <scene>scene</scene>

        <ambient_light>0.6 0.6 0.6</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>

        <camera_pose>
          -6.5 0 4.2 0 0.48 0
        </camera_pose>

      </plugin>

      <plugin filename="GzSceneManager" name="Scene Manager">

        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
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
    output = (
        Path.home()
        / "singulator_digital_twin"
        / "src"
        / "singulator_gazebo"
        / "worlds"
        / "matrix_14x4_stream.sdf"
    )

    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    output.write_text(
        generate_world(),
        encoding="utf-8",
    )

    print(f"Создан мир: {output}")
    print(f"Матрица: {ROWS}x{COLS}")
    print(f"Количество ячеек: {ROWS * COLS}")
    print(f"Активный размер ячейки: {CELL_X:.3f} x {CELL_Y:.3f} м")
    print(f"Зазоры: {GAP_X:.3f} x {GAP_Y:.3f} м")
    print(f"Размер матрицы: {MATRIX_LENGTH:.3f} x {MATRIX_WIDTH:.3f} м")
    print(f"Центр входного конвейера X: {INFEED_CENTER_X:.3f} м")
    print(f"Центр выходного конвейера X: {OUTFEED_CENTER_X:.3f} м")


if __name__ == "__main__":
    main()
