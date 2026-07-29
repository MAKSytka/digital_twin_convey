#!/usr/bin/env python3
"""Static and cross-file checks for the KTY station.

The validator does not pretend to replace a Gazebo runtime test.  It verifies
that the generated models, ROS interfaces, entry points, bridges and fallback
controller are wired consistently before the user starts the simulator.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import py_compile
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "kty_station_sim"
INTERFACES = ROOT / "src" / "singulator_interfaces"
WORLD = PACKAGE / "worlds" / "kty_station.sdf"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_world() -> None:
    tree = ET.parse(WORLD)
    root = tree.getroot()
    require(root.tag == "sdf", "Root element must be sdf")
    world = root.find("world")
    require(world is not None, "World element is missing")
    require(world.attrib.get("name") == "kty_station", "Unexpected world name")

    text = read(WORLD)
    required_fragments = (
        "<max_step_size>0.0005</max_step_size>",
        "<real_time_factor>1.0</real_time_factor>",
        "<size>1.000 0.600 0.020</size>",
        "0.5585053606",
        "/kty/platform/cmd_pos",
        "/kty/shutter/cmd_pos",
        "/kty/camera",
        "<mu>0.75</mu>",
        "<upper>0.0032</upper>",
        '<plugin filename="MinimalScene" name="3D View">',
        "<start_paused>false</start_paused>",
        "<use_event>true</use_event>",
        "/world/kty_station/control",
    )
    for fragment in required_fragments:
        require(fragment in text, f"Missing world fragment: {fragment}")

    track_controllers = world.findall(
        ".//plugin[@filename='gz-sim-track-controller-system']"
    )
    require(len(track_controllers) == 3, "Expected three contact-surface drives")
    topics = {
        controller.findtext("velocity_topic", "")
        for controller in track_controllers
    }
    require(
        topics
        == {
            "/kty/infeed/cmd_vel",
            "/kty/platform/cmd_vel",
            "/kty/outfeed/cmd_vel",
        },
        f"Unexpected TrackController topics: {topics}",
    )

    models = {model.attrib["name"] for model in world.findall("model")}
    required_models = {
        "kty_infeed",
        "kty_vibration_platform",
        "kty_outfeed",
        "product_chute",
        "product_shutter",
        "kty_vision_station",
    }
    require(required_models <= models, f"Missing models: {required_models - models}")


def validate_launch() -> None:
    text = read(PACKAGE / "launch" / "kty_station.launch.py")
    for fragment in (
        '"-r -v 3',
        "pause: false",
        "delayed_unpause",
        'executable="registry_json_mirror"',
        'LaunchConfiguration("vision_gui")',
        'default_value="false"',
    ):
        require(fragment in text, f"Launch wiring is missing: {fragment}")


def validate_python() -> None:
    python_files = sorted(PACKAGE.rglob("*.py"))
    require(python_files, "No Python files found")
    for path in python_files:
        py_compile.compile(str(path), doraise=True)


def validate_factories() -> None:
    module_path = PACKAGE / "kty_station_sim" / "model_factory.py"
    spec = importlib.util.spec_from_file_location("kty_model_factory", module_path)
    require(spec is not None and spec.loader is not None, "Cannot load model_factory")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    kty_sdf = module.make_kty_sdf("kty_test")
    kty_root = ET.fromstring(kty_sdf)
    require(kty_sdf.count("<collision") == 5, "KTY must have bottom and four walls")
    require("0.003000000" in kty_sdf, "3 mm KTY wall is missing")
    require(
        "gz-sim-velocity-control-system" in kty_sdf,
        "Spawned KTY has no deterministic carrier actuator",
    )
    require(
        "/kty/carrier/cmd_vel" in kty_sdf,
        "Spawned KTY listens on the wrong carrier command topic",
    )
    plugin = kty_root.find(".//plugin[@filename='gz-sim-velocity-control-system']")
    require(plugin is not None, "VelocityControl plugin is missing from KTY SDF")

    rng = __import__("random").Random(42)
    for index in range(100):
        product = module.ProductSpec.random(rng)
        require(0.010 <= product.mass <= 5.0, "Product mass outside source data")
        require(0.010 <= product.size_z <= 0.280, "Product Z outside source data")
        require(product.size_x <= 0.400, "Product X outside source data")
        require(product.size_y <= 0.320, "Product Y outside source data")
        ET.fromstring(product.to_sdf(f"product_{index}"))


def validate_interfaces() -> None:
    required = {
        "KtyProductContour.msg",
        "KtyProductContourArray.msg",
        "KtyGroundTruth.msg",
        "KtyGroundTruthArray.msg",
        "KtyStationState.msg",
        "KtyFault.msg",
    }
    message_dir = INTERFACES / "msg"
    found = {path.name for path in message_dir.glob("Kty*.msg")}
    require(required <= found, f"Missing interface files: {required - found}")

    cmake = read(INTERFACES / "CMakeLists.txt")
    for name in required:
        require(f'"msg/{name}"' in cmake, f"{name} is not registered in CMake")

    package_xml = read(PACKAGE / "package.xml")
    require(
        "<buildtool_depend>ament_python</buildtool_depend>" in package_xml,
        "ament_python build dependency was incorrectly removed",
    )


def validate_runtime_wiring() -> None:
    setup = read(PACKAGE / "setup.py")
    require(
        "station_controller = kty_station_sim.station_controller_v2:main" in setup,
        "station_controller entry point does not use runtime v2",
    )
    require(
        "registry_json_mirror = kty_station_sim.registry_json_mirror:main" in setup,
        "JSON registry mirror entry point is missing",
    )

    controller = read(PACKAGE / "kty_station_sim" / "station_controller_v2.py")
    for fragment in (
        'Twist, "/kty/carrier/cmd_vel"',
        "carrier.linear.x = carrier_x",
        "deterministic 2 s carrier transfer",
        "pose feedback unavailable",
        "scan_failures_before_fault",
        "surface_command_sign * infeed",
    ):
        require(fragment in controller, f"Controller v2 is missing: {fragment}")

    safety = read(PACKAGE / "kty_station_sim" / "safety_monitor.py")
    for fragment in (
        "pose_stream_alive",
        "pose-dependent safety",
        "KtyFault.WARNING",
        "StationControllerV2 owns the retry counter",
    ):
        require(fragment in safety, f"Safety fallback is missing: {fragment}")

    bridge = read(PACKAGE / "config" / "bridge.yaml")
    for fragment in (
        "/kty/carrier/cmd_vel",
        "geometry_msgs/msg/Twist",
        "gz.msgs.Twist",
    ):
        require(fragment in bridge, f"Carrier bridge is missing: {fragment}")

    run_script = read(ROOT / "scripts" / "run_kty_station.sh")
    for fragment in (
        "registry_json_mirror",
        "KtyGroundTruthArray",
        "KtyStationState",
        "build_kty_station.sh",
    ):
        require(fragment in run_script, f"Launcher preflight is missing: {fragment}")

    targeted_build = read(ROOT / "scripts" / "build_kty_station.sh")
    for fragment in (
        "build/singulator_interfaces",
        "install/singulator_interfaces",
        "--packages-select singulator_interfaces kty_station_sim",
        "ros2 interface show",
        "--skip-keys ament_python",
    ):
        require(fragment in targeted_build, f"Targeted build is missing: {fragment}")

    diagnostic = read(ROOT / "scripts" / "check_kty_station.sh")
    require(
        "/kty/ground_truth/registry_json" in diagnostic,
        "Diagnostic script does not use the standard-message JSON mirror",
    )


def main() -> None:
    validate_world()
    validate_launch()
    validate_python()
    validate_factories()
    validate_interfaces()
    validate_runtime_wiring()
    print("KTY station static validation: OK")


if __name__ == "__main__":
    main()
