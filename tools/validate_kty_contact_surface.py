#!/usr/bin/env python3
"""Static validation for the roller-free KTY contact-surface runtime."""

from __future__ import annotations

from pathlib import Path
import py_compile
import sys
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src" / "kty_station_sim"
PLUGIN = ROOT / "src" / "kty_conveyor_surface"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_plugin_package() -> None:
    package_xml = read(PLUGIN / "package.xml")
    cmake = read(PLUGIN / "CMakeLists.txt")
    source = read(PLUGIN / "src" / "KtyConveyorSurfaceSystem.cc")
    for fragment in (
        "<name>kty_conveyor_surface</name>",
        "<build_type>ament_cmake</build_type>",
        "gz-sim8",
        "gz-transport13",
    ):
        require(fragment in package_xml, f"Missing plugin package metadata: {fragment}")
    for fragment in (
        "add_library(KtyConveyorSurfaceSystem SHARED",
        "gz-sim8::gz-sim8",
        "gz-plugin2::gz-plugin2",
        "LIBRARY DESTINATION lib",
    ):
        require(fragment in cmake, f"Missing plugin build wiring: {fragment}")
    for fragment in (
        "class KtyConveyorSurfaceSystem",
        "ISystemPreUpdate",
        "Link link(linkEntity)",
        "link.AddWorldForce",
        "targetVelocity - velocity->X()",
        "modelPrefix",
        "GZ_ADD_PLUGIN",
    ):
        require(fragment in source, f"Missing contact-surface behavior: {fragment}")
    require("LinearVelocityCmd" not in source, "Transport must preserve vertical physics")


def validate_generated_world() -> None:
    sys.path.insert(0, str(SIM))
    from kty_station_sim.world_patch_v3 import build_surface_world

    with tempfile.TemporaryDirectory() as directory:
        destination = Path(directory) / "surface.sdf"
        build_surface_world(
            SIM / "worlds" / "kty_mechatronics.sdf",
            destination,
        )
        root = ET.parse(destination).getroot()

    world = root.find("world")
    require(world is not None, "Generated world is missing")
    require(
        world.attrib.get("name") == "kty_mechatronics_surface",
        "Unexpected contact-surface world name",
    )
    machine = world.find("model[@name='kty_mechatronics_machine']")
    require(machine is not None, "Machine model is missing")
    names = [element.attrib.get("name", "") for element in machine.findall("link")]
    joints = [element.attrib.get("name", "") for element in machine.findall("joint")]
    require(not any("roller" in name for name in names), "Roller links remain in runtime world")
    require(not any("roller" in name for name in joints), "Roller joints remain in runtime world")
    require("infeed_surface" in names, "Infeed contact plate is missing")
    require("outfeed_surface" in names, "Outfeed contact plate is missing")
    require(
        machine.find("link[@name='vibration_deck']/collision[@name='active_contact_surface']")
        is not None,
        "Active contact plate is missing from vibration deck",
    )
    plugin = world.find(
        "plugin[@name='kty_conveyor_surface::KtyConveyorSurfaceSystem']"
    )
    require(plugin is not None, "Contact-surface Gazebo plugin is missing")
    topics = {zone.findtext("topic") for zone in plugin.findall("zone")}
    require(
        topics
        == {
            "/kty/mech/infeed_surface/cmd_vel",
            "/kty/mech/active_surface/cmd_vel",
            "/kty/mech/outfeed_surface/cmd_vel",
        },
        f"Unexpected surface command topics: {sorted(topics)}",
    )


def validate_python_and_launch() -> None:
    paths = (
        SIM / "kty_station_sim" / "world_patch_v3.py",
        SIM / "kty_station_sim" / "mechatronics_cycle_v3.py",
        SIM / "launch" / "kty_mechatronics_surface.launch.py",
    )
    for path in paths:
        py_compile.compile(str(path), doraise=True)

    controller = read(paths[1])
    for fragment in (
        'super().__init__()',
        '"/kty/mech/infeed_surface/cmd_vel"',
        '"/kty/mech/active_surface/cmd_vel"',
        '"/kty/mech/outfeed_surface/cmd_vel"',
        'payload["transport"] = "flat_contact_surface"',
    ):
        require(fragment in controller, f"Missing surface controller behavior: {fragment}")

    launch = read(paths[2])
    for fragment in (
        "build_surface_world",
        'get_package_prefix("kty_conveyor_surface")',
        'name="GZ_SIM_SYSTEM_PLUGIN_PATH"',
        'executable="mechatronics_cycle_v3"',
        '"world_name": "kty_mechatronics_surface"',
        'default_value="1.15"',
    ):
        require(fragment in launch, f"Missing surface launch wiring: {fragment}")
    require("_rollers/cmd_vel" not in launch, "Launch still bridges roller commands")


def validate_package_and_scripts() -> None:
    setup = read(SIM / "setup.py")
    package_xml = read(SIM / "package.xml")
    require('version="0.6.0"' in setup, "Expected kty_station_sim version 0.6.0")
    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v3:main" in setup,
        "Missing v3 controller entry point",
    )
    require("<version>0.6.0</version>" in package_xml, "package.xml version mismatch")
    require(
        "<exec_depend>kty_conveyor_surface</exec_depend>" in package_xml,
        "Missing runtime dependency on contact-surface plugin",
    )

    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")
    run = read(ROOT / "scripts" / "run_kty_perception_3d.sh")
    for fragment in (
        "kty_conveyor_surface",
        "mechatronics_cycle_v3",
        "libKtyConveyorSurfaceSystem.so",
        "validate_kty_contact_surface.py",
    ):
        require(fragment in build, f"Missing build behavior: {fragment}")
    require(
        "kty_mechatronics_surface.launch.py" in run,
        "Run script does not use surface launch",
    )


def main() -> None:
    validate_plugin_package()
    validate_generated_world()
    validate_python_and_launch()
    validate_package_and_scripts()
    print("KTY roller-free contact-surface static validation: OK")


if __name__ == "__main__":
    main()
