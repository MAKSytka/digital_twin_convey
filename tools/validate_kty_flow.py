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
BASE_CONTROLLER = PACKAGE / "kty_station_sim" / "flow_cycle.py"
SMOOTH_CONTROLLER = PACKAGE / "kty_station_sim" / "flow_cycle_smooth.py"


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
        "gz-sim-user-commands-system",
        "gz-sim-scene-broadcaster-system",
        "<max_step_size>0.001</max_step_size>",
        "<real_time_factor>1.0</real_time_factor>",
        "<pose>-0.730 0 1.185 0 0.5585053606 0</pose>",
        "<mu>0.20</mu>",
        '<plugin filename="MinimalScene" name="3D View">',
        '<plugin filename="InteractiveViewControl" name="Interactive view control">',
        '<plugin filename="CameraTracking" name="Camera Tracking">',
        '<plugin name="World control" filename="WorldControl">',
        "<start_paused>0</start_paused>",
    ):
        require(fragment in text, f"Missing world fragment: {fragment}")


def validate_python() -> None:
    for path in (BASE_CONTROLLER, SMOOTH_CONTROLLER, LAUNCH):
        py_compile.compile(str(path), doraise=True)

    base = read(BASE_CONTROLLER)
    for fragment in (
        'super().__init__("kty_flow_cycle")',
        '"/kty/flow/state"',
        '"/kty/flow/heartbeat"',
        '"/kty/flow/restart"',
        "make_kty_sdf(kty_name)",
        "make_flow_product_sdf(name, profile)",
        "self._create_service",
        "self._set_pose_service",
        "self._remove_service",
        "self._pose_topic",
        "inside == self.product_count",
        "time.monotonic()",
    ):
        require(fragment in base, f"Missing base controller behavior: {fragment}")

    smooth = read(SMOOTH_CONTROLLER)
    for fragment in (
        "class SmoothKtyFlowCycle(KtyFlowCycle)",
        "VIBRATION_DURATION_S = 5.0",
        "VIBRATION_FREQUENCY_HZ = 5.0",
        "VIBRATION_AMPLITUDE_M = 0.0020",
        "PRODUCT_UPDATE_HZ = 8.0",
        "6.0 * ratio**5 - 15.0 * ratio**4 + 10.0 * ratio**3",
        '"APPROACH"',
        '"LOAD"',
        '"SETTLE"',
        '"VIBRATE"',
        '"OUTFEED"',
        '"DESPAWN"',
        '"COMPLETE"',
        "self._vibrate_kty(kty_name, captured_poses[kty_name])",
        "removed_total=self._removed_models",
    ):
        require(fragment in smooth, f"Missing smooth controller behavior: {fragment}")

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
        'DeclareLaunchArgument("pose_update_hz", default_value="20.0")',
        '"pose_update_hz": ParameterValue(',
    ):
        require(fragment in launch, f"Missing launch wiring: {fragment}")


def validate_package_and_scripts() -> None:
    setup = read(PACKAGE / "setup.py")
    require(
        "flow_cycle = kty_station_sim.flow_cycle_smooth:main" in setup,
        "smooth flow_cycle console entry point is missing",
    )

    package_xml = read(PACKAGE / "package.xml")
    supported_versions = (
        "<version>0.3.0</version>",
        "<version>0.4.0</version>",
        "<version>0.5.0</version>",
    )
    require(
        any(version in package_xml for version in supported_versions),
        "Expected compatible KTY package version 0.3.0, 0.4.0 or 0.5.0",
    )
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
