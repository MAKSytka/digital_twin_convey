#!/usr/bin/env python3
"""Static validation that does not require ROS 2 or Gazebo to be running."""

from __future__ import annotations

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "src/singulator_gazebo/worlds/matrix_14x4_stream.sdf"
MATRIX_CONFIG = ROOT / "src/singulator_description/config/matrix.yaml"
BRIDGE_DIR = ROOT / "src/singulator_bringup/config"
LAUNCH_FILE = ROOT / "src/singulator_bringup/launch/matrix_stream.launch.py"
RUN_DEMO = ROOT / "scripts/run_demo.sh"
UNIFORM_CONTROLLER = ROOT / "src/singulator_control/singulator_control/uniform_matrix_controller.py"

ROWS = 14
COLS = 4
EXPECTED_CELLS = {
    f"cell_r{row:02d}_c{col:02d}"
    for row in range(ROWS)
    for col in range(COLS)
}
EXPECTED_COMMAND_TOPICS = {
    f"/singulator/cell/r{row:02d}_c{col:02d}/cmd_vel"
    for row in range(ROWS)
    for col in range(COLS)
}


def fail(message: str) -> None:
    raise AssertionError(message)


def validate_world() -> None:
    world = ET.parse(WORLD).getroot().find("world")
    if world is None:
        fail("SDF does not contain a <world> element")
    if world.get("name") != "matrix_14x4_stream":
        fail("Unexpected Gazebo world name")

    models = world.findall("model")
    model_names = {model.get("name") for model in models}
    cell_names = {name for name in model_names if name and name.startswith("cell_")}

    if cell_names != EXPECTED_CELLS:
        missing = sorted(EXPECTED_CELLS - cell_names)
        extra = sorted(cell_names - EXPECTED_CELLS)
        fail(f"Cell model mismatch; missing={missing}, extra={extra}")

    for required in ("infeed_conveyor", "outfeed_conveyor", "platform"):
        if required not in model_names:
            fail(f"Missing model: {required}")

    topics = {
        element.text.strip()
        for element in world.findall(".//velocity_topic")
        if element.text
    }
    if not EXPECTED_COMMAND_TOPICS.issubset(topics):
        fail("Not all 56 cell command topics exist in the SDF")
    if "/singulator/infeed/cmd_vel" not in topics:
        fail("Missing infeed command topic in SDF")
    if "/singulator/outfeed/cmd_vel" not in topics:
        fail("Missing outfeed command topic in SDF")

    friction_pairs = {
        (
            float(ode.findtext("mu")),
            float(ode.findtext("mu2")),
        )
        for ode in world.findall(".//surface/friction/ode")
    }
    if friction_pairs != {(0.8, 0.2)}:
        fail(f"Unexpected conveyor friction values: {friction_pairs}")


def validate_matrix_config() -> None:
    config = yaml.safe_load(MATRIX_CONFIG.read_text(encoding="utf-8"))["matrix"]
    expected = {
        "rows": 14,
        "cols": 4,
        "active_cell_length_m": 0.360,
        "active_cell_width_m": 0.175,
        "longitudinal_gap_m": 0.020,
        "transverse_gap_m": 0.020,
        "pitch_x_m": 0.380,
        "pitch_y_m": 0.195,
        "total_length_m": 5.300,
        "total_width_m": 0.760,
    }
    for key, value in expected.items():
        if config.get(key) != value:
            fail(f"matrix.yaml: {key}={config.get(key)!r}, expected {value!r}")


def validate_bridges() -> None:
    files = [
        BRIDGE_DIR / "bridge_rows_00_03.yaml",
        BRIDGE_DIR / "bridge_rows_04_07.yaml",
        BRIDGE_DIR / "bridge_rows_08_11.yaml",
        BRIDGE_DIR / "bridge_rows_12_13.yaml",
        BRIDGE_DIR / "bridge_aux.yaml",
    ]
    entries: list[dict] = []
    for path in files:
        entries.extend(yaml.safe_load(path.read_text(encoding="utf-8")))

    ros_topics = {entry["ros_topic_name"] for entry in entries}
    cell_topics = {topic for topic in ros_topics if "/cell/" in topic}
    if cell_topics != EXPECTED_COMMAND_TOPICS:
        fail("Split bridge configs do not cover exactly 56 cell command topics")

    for topic in ("/clock", "/singulator/infeed/cmd_vel", "/singulator/outfeed/cmd_vel"):
        if topic not in ros_topics:
            fail(f"Bridge config misses {topic}")



def validate_speed_defaults() -> None:
    root = ET.parse(WORLD).getroot()
    controllers = root.findall(".//plugin[@name='gz::sim::systems::TrackController']")
    if len(controllers) != 58:
        fail(f"Expected 58 TrackController plugins, got {len(controllers)}")
    for controller in controllers:
        if float(controller.findtext("max_velocity")) != 2.0:
            fail("A TrackController max_velocity is not 2.0 m/s")
        if float(controller.findtext("min_velocity")) != -2.0:
            fail("A TrackController min_velocity is not -2.0 m/s")

    launch_text = LAUNCH_FILE.read_text(encoding="utf-8")
    if 'DeclareLaunchArgument("demo_speed_mps", default_value="2.0")' not in launch_text:
        fail("matrix_stream.launch.py demo speed default is not 2.0 m/s")

    demo_text = RUN_DEMO.read_text(encoding="utf-8")
    if 'CONVEYOR_SPEED_MPS="${CONVEYOR_SPEED_MPS:-2.0}"' not in demo_text:
        fail("run_demo.sh does not use one 2.0 m/s default for all conveyors")

    controller_text = UNIFORM_CONTROLLER.read_text(encoding="utf-8")
    if 'declare_parameter("speed_mps",2.0)' not in controller_text:
        fail("uniform_matrix_controller default is not 2.0 m/s")


def validate_python_package_manifests() -> None:
    packages = (
        ROOT / "src/singulator_bringup/package.xml",
        ROOT / "src/singulator_control/package.xml",
        ROOT / "src/singulator_sim/package.xml",
    )
    for path in packages:
        package_root = ET.parse(path).getroot()
        build_type = package_root.findtext("./export/build_type")
        if build_type != "ament_python":
            fail(f"{path}: build_type must be ament_python")
        invalid = [
            element.text
            for element in package_root.findall("buildtool_depend")
            if element.text == "ament_python"
        ]
        if invalid:
            fail(
                f"{path}: ament_python is a build type, not a rosdep dependency"
            )

def validate_interfaces() -> None:
    matrix_command = (
        ROOT / "src/singulator_interfaces/msg/MatrixCommand.msg"
    ).read_text(encoding="utf-8")
    required = (
        "uint16 rows",
        "uint16 cols",
        "float32[] target_speed_mps",
    )
    for line in required:
        if line not in matrix_command:
            fail(f"MatrixCommand.msg misses: {line}")


def validate_python() -> None:
    paths = list(ROOT.glob("src/**/*.py")) + list(ROOT.glob("examples/*.py"))
    for path in paths:
        source = path.read_text(encoding="utf-8")
        compile(source, str(path), "exec")


def validate_clean_tree() -> None:
    forbidden_dirs = ("build", "install", "log", "src_before_rtf_optimization")
    for name in forbidden_dirs:
        if (ROOT / name).exists():
            fail(f"Forbidden generated or backup directory is present: {name}")

    backups = list(ROOT.rglob("*.before_*")) + list(ROOT.rglob("*.backup"))
    if backups:
        fail(f"Backup files are present: {backups}")


def main() -> int:
    checks = (
        ("world", validate_world),
        ("matrix config", validate_matrix_config),
        ("bridge coverage", validate_bridges),
        ("2 m/s defaults", validate_speed_defaults),
        ("Python package manifests", validate_python_package_manifests),
        ("interfaces", validate_interfaces),
        ("Python syntax", validate_python),
        ("clean repository tree", validate_clean_tree),
    )

    for label, check in checks:
        check()
        print(f"[OK] {label}")

    print("Static project validation passed.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        raise
