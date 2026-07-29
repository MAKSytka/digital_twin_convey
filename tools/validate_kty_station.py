#!/usr/bin/env python3
"""Static checks for the KTY station overlay, without ROS or Gazebo."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import py_compile
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "kty_station_sim"
WORLD = PACKAGE / "worlds" / "kty_station.sdf"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def validate_world() -> None:
    tree = ET.parse(WORLD)
    root = tree.getroot()
    require(root.tag == "sdf", "Root element must be sdf")
    world = root.find("world")
    require(world is not None, "World element is missing")
    require(world.attrib.get("name") == "kty_station", "Unexpected world name")

    text = WORLD.read_text(encoding="utf-8")
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
    require(len(track_controllers) == 3, "Expected three driven contact surfaces")
    for controller in track_controllers:
        orientation = controller.findtext("track_orientation", "").split()
        require(
            orientation == ["0", "0", "0"],
            "Positive contact-surface commands must transport payloads toward +X",
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
    launch_file = PACKAGE / "launch" / "kty_station.launch.py"
    text = launch_file.read_text(encoding="utf-8")
    require('"-r -v 3' in text, "Gazebo must be launched in running mode")
    require("pause: false" in text, "Launch file must explicitly unpause the world")
    require("delayed_unpause" in text, "Unpause retry action is missing")


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
    ET.fromstring(kty_sdf)
    require(kty_sdf.count("<collision") == 5, "KTY must have bottom and four walls")
    require("0.003000000" in kty_sdf, "3 mm KTY wall is missing")

    rng = __import__("random").Random(42)
    for index in range(100):
        product = module.ProductSpec.random(rng)
        require(0.010 <= product.mass <= 5.0, "Product mass outside TZ")
        require(product.size_x <= 0.400, "Product X outside TZ")
        require(product.size_y <= 0.320, "Product Y outside TZ")
        require(product.size_z <= 0.280, "Product Z outside TZ")
        ET.fromstring(product.to_sdf(f"product_{index}"))


def validate_interfaces() -> None:
    message_dir = ROOT / "src" / "singulator_interfaces" / "msg"
    required = {
        "KtyProductContour.msg",
        "KtyProductContourArray.msg",
        "KtyGroundTruth.msg",
        "KtyGroundTruthArray.msg",
        "KtyStationState.msg",
        "KtyFault.msg",
    }
    found = {path.name for path in message_dir.glob("Kty*.msg")}
    require(required <= found, f"Missing interface files: {required - found}")
    cmake = (ROOT / "src" / "singulator_interfaces" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    for name in required:
        require(f'"msg/{name}"' in cmake, f"{name} is not registered in CMake")


def validate_launcher() -> None:
    run_script = (ROOT / "scripts" / "run_kty_station.sh").read_text(
        encoding="utf-8"
    )
    for executable in (
        "station_controller",
        "product_spawner",
        "depth_perception",
        "safety_monitor",
        "metrics_node",
    ):
        require(executable in run_script, f"Launcher does not check {executable}")

    controller = (
        PACKAGE / "kty_station_sim" / "station_controller.py"
    ).read_text(encoding="utf-8")
    for fragment in (
        '"/kty/world/poses"',
        "abs(self.active_kty_x) <= self.position_tolerance",
        "KTY positioning timeout; x=",
    ):
        require(fragment in controller, f"Missing pose-driven positioning: {fragment}")


def main() -> None:
    validate_world()
    validate_launch()
    validate_python()
    validate_factories()
    validate_interfaces()
    validate_launcher()
    print("KTY station static validation: OK")


if __name__ == "__main__":
    main()
