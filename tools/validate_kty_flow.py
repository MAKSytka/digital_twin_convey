#!/usr/bin/env python3
"""Static validation for the stage-2 deterministic KTY flow scenario."""

from __future__ import annotations

from pathlib import Path
import py_compile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "kty_station_sim"
WORLD = PACKAGE / "worlds" / "kty_flow.sdf"
LAUNCH = PACKAGE / "launch" / "kty_flow.launch.py"
CONTROLLER = PACKAGE / "kty_station_sim" / "flow_cycle.py"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_world() -> None:
    root = ET.parse(WORLD).getroot()
    require(root.tag == "sdf", "Flow world root must be <sdf>")
    world = root.find("world")
    require(world is not None, "Flow world is missing <world>")
    require(world.attrib.get("name") == "kty_flow", "Unexpected flow world name")

    models = {model.attrib.get("name") for model in world.findall("model")}
    required = {
        "floor",
        "kty_flow_infeed",
        "kty_flow_platform",
        "kty_flow_outfeed",
        "kty_flow_side_guides",
        "kty_flow_product_chute",
    }
    require(required <= models, f"Missing flow world models: {required - models}")
    require(
        "kty_flow_container" not in models,
        "Dynamic KTY must be created by flow_cycle, not embedded in the world",
    )

    text = read(WORLD)
    for fragment in (
        'name="kty_flow"',
        'gz-sim-user-commands-system',
        'gz-sim-scene-broadcaster-system',
        '<max_step_size>0.001</max_step_size>',
        '<real_time_factor>1.0</real_time_factor>',
        '<pose>-0.730 0 1.185 0 0.5585053606 0</pose>',
        '<mu>0.20</mu>',
        '<plugin filename="MinimalScene" name="3D View">',
        '<plugin filename="InteractiveViewControl" name="Interactive view control">',
        '<plugin filename="CameraTracking" name="Camera Tracking">',
        '<plugin name="World control" filename="WorldControl">',
        '<start_paused>0</start_paused>',
    ):
        require(fragment in text, f"Missing world fragment: {fragment}")


def validate_python() -> None:
    py_compile.compile(str(CONTROLLER), doraise=True)
    py_compile.compile(str(LAUNCH), doraise=True)

    controller = read(CONTROLLER)
    for fragment in (
        'super().__init__("kty_flow_cycle")',
        '"/kty/flow/state"',
        '"/kty/flow/heartbeat"',
        '"/kty/flow/restart"',
        'make_kty_sdf(kty_name)',
        'make_flow_product_sdf(name, profile)',
        'self._create_service',
        'self._set_pose_service',
        'self._remove_service',
        'self._pose_topic',
        '"APPROACH"',
        '"LOAD"',
        '"SETTLE"',
        '"OUTFEED"',
        '"DESPAWN"',
        '"COMPLETE"',
        'inside == self.product_count',
        'removed_total=self._removed_models',
        'time.monotonic()',
    ):
        require(fragment in controller, f"Missing controller behavior: {fragment}")

    launch = read(LAUNCH)
    require(
        '"use_sim_time": False' in launch,
        "Flow launch must explicitly use wall time",
    )
    for fragment in (
        'worlds" / "kty_flow.sdf"',
        'executable="flow_cycle"',
        'DeclareLaunchArgument("product_count"',
        'DeclareLaunchArgument("auto_repeat"',
    ):
        require(fragment in launch, f"Missing launch wiring: {fragment}")


def validate_package_and_scripts() -> None:
    setup = read(PACKAGE / "setup.py")
    require(
        "flow_cycle = kty_station_sim.flow_cycle:main" in setup,
        "flow_cycle console entry point is missing",
    )

    package_xml = read(PACKAGE / "package.xml")
    require("<version>0.3.0</version>" in package_xml, "Expected package version 0.3.0")
    require(
        "<buildtool_depend>ament_python</buildtool_depend>" not in package_xml,
        "ament_python must remain a build type only",
    )

    scripts = (
        "scripts/build_kty_flow.sh",
        "scripts/run_kty_flow.sh",
        "scripts/check_kty_flow.sh",
        "scripts/stop_kty_flow.sh",
    )
    for relative in scripts:
        text = read(ROOT / relative)
        require(text.startswith("#!/usr/bin/env bash"), f"Missing shebang: {relative}")

    diagnostic = read(ROOT / "scripts/check_kty_flow.sh")
    for fragment in (
        "/world/kty_flow/control",
        "/world/kty_flow/create",
        "/world/kty_flow/remove",
        "/world/kty_flow/set_pose",
        "/kty/flow/state",
        "complete cycle observed",
        "KTY and products were despawned",
    ):
        require(fragment in diagnostic, f"Missing diagnostic behavior: {fragment}")


def main() -> None:
    validate_world()
    validate_python()
    validate_package_and_scripts()
    print("KTY flow static validation: OK")


if __name__ == "__main__":
    main()
