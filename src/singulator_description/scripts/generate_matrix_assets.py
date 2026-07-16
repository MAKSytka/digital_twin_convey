#!/usr/bin/env python3
"""Generate matrix SDF and ros_gz bridge YAML from matrix.yaml."""

from pathlib import Path
import yaml

SCRIPT = Path(__file__).resolve()
DESC_DIR = SCRIPT.parents[1]
SRC_DIR = SCRIPT.parents[2]
CONFIG = DESC_DIR / "config" / "matrix.yaml"
MODEL_OUT = DESC_DIR / "models" / "matrix_14x4" / "model.sdf"
BRIDGE_OUT = SRC_DIR / "singulator_bringup" / "config" / "matrix_bridge.yaml"


def cell_name(row: int, col: int) -> str:
    return f"r{row:02d}_c{col:02d}"


def generate_model(cfg: dict) -> str:
    m = cfg["matrix"]
    rows, cols = int(m["rows"]), int(m["cols"])
    px, py = float(m["pitch_x_m"]), float(m["pitch_y_m"])
    length, width = float(m["active_length_m"]), float(m["active_width_m"])
    height = float(m["belt_height_m"])
    min_v, max_v = float(m["min_velocity_mps"]), float(m["max_velocity_mps"])
    max_a, max_j = float(m["max_acceleration_mps2"]), float(m["max_jerk_mps3"])
    timeout = float(m["command_timeout_s"])
    mu_lat = float(m["friction"]["lateral_mu"])
    mu_long = float(m["friction"]["longitudinal_mu"])
    ambient = m["visual"]["ambient"]
    diffuse = m["visual"]["diffuse"]

    parts = [
        '<?xml version="1.0"?>',
        '<sdf version="1.10">',
        '  <model name="matrix_14x4">',
        '    <static>true</static>',
    ]

    for row in range(rows):
        x = (row + 0.5) * px
        for col in range(cols):
            y = (col - (cols - 1) / 2.0) * py
            name = cell_name(row, col)
            parts += [
                f'    <link name="cell_{name}">',
                f'      <pose>{x:.6f} {y:.6f} {height/2:.6f} 0 0 0</pose>',
                f'      <collision name="collision_{name}">',
                f'        <geometry><box><size>{length:.6f} {width:.6f} {height:.6f}</size></box></geometry>',
                '        <surface><friction><ode>',
                f'          <mu>{mu_lat:.6f}</mu>',
                f'          <mu2>{mu_long:.6f}</mu2>',
                '          <fdir1>0 1 0</fdir1>',
                '        </ode></friction></surface>',
                '      </collision>',
                f'      <visual name="visual_{name}">',
                f'        <geometry><box><size>{length:.6f} {width:.6f} {height:.6f}</size></box></geometry>',
                f'        <material><ambient>{ambient}</ambient><diffuse>{diffuse}</diffuse></material>',
                '      </visual>',
                '    </link>',
            ]

    for row in range(rows):
        for col in range(cols):
            name = cell_name(row, col)
            topic = f"/singulator/cell/{name}/cmd_vel"
            parts += [
                '    <plugin filename="gz-sim-track-controller-system"',
                '            name="gz::sim::systems::TrackController">',
                f'      <link>cell_{name}</link>',
                f'      <velocity_topic>{topic}</velocity_topic>',
                f'      <min_velocity>{min_v:.6f}</min_velocity>',
                f'      <max_velocity>{max_v:.6f}</max_velocity>',
                f'      <min_acceleration>{-max_a:.6f}</min_acceleration>',
                f'      <max_acceleration>{max_a:.6f}</max_acceleration>',
                f'      <min_jerk>{-max_j:.6f}</min_jerk>',
                f'      <max_jerk>{max_j:.6f}</max_jerk>',
                f'      <max_command_age>{timeout:.6f}</max_command_age>',
                '      <odometry_publish_frequency>20</odometry_publish_frequency>',
                '    </plugin>',
            ]

    parts += ['  </model>', '</sdf>', '']
    return "\n".join(parts)


def generate_bridge(cfg: dict) -> str:
    m = cfg["matrix"]
    rows, cols = int(m["rows"]), int(m["cols"])
    lines = [
        '- ros_topic_name: "/clock"',
        '  gz_topic_name: "/clock"',
        '  ros_type_name: "rosgraph_msgs/msg/Clock"',
        '  gz_type_name: "gz.msgs.Clock"',
        '  direction: GZ_TO_ROS',
        '',
    ]
    for row in range(rows):
        for col in range(cols):
            name = cell_name(row, col)
            topic = f"/singulator/cell/{name}/cmd_vel"
            lines += [
                f'- ros_topic_name: "{topic}"',
                f'  gz_topic_name: "{topic}"',
                '  ros_type_name: "std_msgs/msg/Float64"',
                '  gz_type_name: "gz.msgs.Double"',
                '  direction: ROS_TO_GZ',
                '',
            ]
    return "\n".join(lines)


def main() -> None:
    cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    MODEL_OUT.write_text(generate_model(cfg), encoding="utf-8")
    BRIDGE_OUT.write_text(generate_bridge(cfg), encoding="utf-8")
    print(f"Generated: {MODEL_OUT}")
    print(f"Generated: {BRIDGE_OUT}")


if __name__ == "__main__":
    main()
