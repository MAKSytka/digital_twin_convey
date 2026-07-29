#!/usr/bin/env python3
"""Static validation for deterministic KTY transport and effective vibration."""

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


def _floats(text: str | None) -> list[float]:
    require(bool(text), "Expected numeric SDF text")
    return [float(value) for value in str(text).split()]


def _top_z(link: ET.Element) -> float:
    pose = _floats(link.findtext("pose"))
    size = _floats(link.findtext("collision/geometry/box/size"))
    require(len(pose) == 6 and len(size) == 3, f"Invalid surface geometry: {link.attrib}")
    return pose[2] + 0.5 * size[2]


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
        "link.SetLinearVelocity",
        "link.SetAngularVelocity",
        "link.AddWorldForce",
        "targetVelocity = commands[index]",
        "modelPrefix",
        "GZ_ADD_PLUGIN",
        "const_cast<sdf::Element *>(_sdf.get())",
        'mutableSdf->GetElement("zone")',
    ):
        require(fragment in source, f"Missing deterministic surface behavior: {fragment}")
    require(
        '_sdf->GetElement("zone")' not in source,
        "sdformat14 traversal must not call non-const GetElement through _sdf",
    )
    overlap_loop = source[source.index("for (std::size_t index = 0;"):source.index("if (!insideZone)")]
    require("break;" not in overlap_loop, "Later zones must win in overlap regions")


def validate_generated_world() -> None:
    sys.path.insert(0, str(SIM))
    from kty_station_sim.world_patch_v3 import build_surface_world

    with tempfile.TemporaryDirectory() as directory:
        destination = Path(directory) / "surface.sdf"
        build_surface_world(SIM / "worlds" / "kty_mechatronics.sdf", destination)
        root = ET.parse(destination).getroot()

    world = root.find("world")
    require(world is not None, "Generated world is missing")
    require(world.attrib.get("name") == "kty_mechatronics_surface", "Wrong world name")
    require(world.findtext("physics/max_step_size") == "0.004", "Expected 4 ms step")
    require(world.findtext("physics/real_time_update_rate") == "250", "Expected 250 Hz")
    sensor = world.find(".//sensor[@name='overhead_rgbd']")
    require(sensor is not None, "RGB-D sensor missing")
    require(sensor.findtext("update_rate") == "6", "Expected 6 Hz RGB-D")
    require(sensor.findtext("camera/image/width") == "512", "Expected 512 px")
    require(sensor.findtext("camera/image/height") == "384", "Expected 384 px")

    machine = world.find("model[@name='kty_mechatronics_machine']")
    require(machine is not None, "Machine model is missing")
    names = [element.attrib.get("name", "") for element in machine.findall("link")]
    joints = [element.attrib.get("name", "") for element in machine.findall("joint")]
    require(not any("roller" in name for name in names), "Roller links remain")
    require(not any("roller" in name for name in joints), "Roller joints remain")
    require("locator_stop" not in names, "Joint locator link remains")
    require("locator_stop_joint" not in joints, "Joint locator remains")
    for expected in ("infeed_surface", "outfeed_transfer_bridge", "outfeed_surface"):
        require(expected in names, f"Missing surface: {expected}")

    vibration_joint = machine.find("joint[@name='vibration_joint']")
    require(vibration_joint is not None, "Vibration joint missing")
    require(vibration_joint.findtext("axis/limit/lower") == "-0.006", "Wrong lower stroke")
    require(vibration_joint.findtext("axis/limit/upper") == "0.006", "Wrong upper stroke")
    require(vibration_joint.findtext("axis/limit/effort") == "12000", "Wrong effort limit")
    require(vibration_joint.findtext("axis/limit/velocity") == "2.0", "Wrong velocity limit")
    require(vibration_joint.findtext("axis/dynamics/damping") == "80", "Wrong deck damping")
    require(vibration_joint.findtext("axis/dynamics/friction") == "3", "Wrong deck friction")

    vibration_controller = next(
        (
            plugin
            for plugin in machine.findall("plugin")
            if plugin.findtext("joint_name", default="") == "vibration_joint"
        ),
        None,
    )
    require(vibration_controller is not None, "Vibration controller missing")
    require(vibration_controller.findtext("p_gain") == "320000", "Wrong vibration p_gain")
    require(vibration_controller.findtext("d_gain") == "2400", "Wrong vibration d_gain")
    require(vibration_controller.findtext("cmd_max") == "12000", "Wrong positive force cap")
    require(vibration_controller.findtext("cmd_min") == "-12000", "Wrong negative force cap")

    bridge = machine.find("link[@name='outfeed_transfer_bridge']")
    outfeed = machine.find("link[@name='outfeed_surface']")
    active = machine.find("link[@name='vibration_deck']/collision[@name='active_contact_surface']")
    require(bridge is not None and outfeed is not None, "Outfeed geometry missing")
    require(active is not None, "Active plate missing")
    require(abs(_top_z(bridge) - 0.498) < 1.0e-9, "Bridge top must be 498 mm")
    require(abs(_top_z(outfeed) - 0.496) < 1.0e-9, "Outfeed top must be 496 mm")
    require(_top_z(bridge) > _top_z(outfeed), "Transfer must descend")

    collisions = (
        machine.find("link[@name='infeed_surface']/collision"),
        active,
        machine.find("link[@name='outfeed_transfer_bridge']/collision"),
        machine.find("link[@name='outfeed_surface']/collision"),
    )
    for collision in collisions:
        require(collision is not None, "A transport collision is missing")
        require(collision.findtext("surface/friction/ode/mu") == "0.08", "X friction")
        require(collision.findtext("surface/friction/ode/mu2") == "1.15", "Y friction")
        require(collision.findtext("surface/friction/ode/fdir1") == "1 0 0", "fdir1")

    plugin = world.find("plugin[@name='kty_conveyor_surface::KtyConveyorSurfaceSystem']")
    require(plugin is not None, "Contact-surface plugin missing")
    topics = {zone.findtext("topic") for zone in plugin.findall("zone")}
    require(
        topics == {
            "/kty/mech/infeed_surface/cmd_vel",
            "/kty/mech/active_surface/cmd_vel",
            "/kty/mech/outfeed_surface/cmd_vel",
        },
        f"Unexpected topics: {sorted(topics)}",
    )


def validate_python_and_launch() -> None:
    paths = (
        SIM / "kty_station_sim" / "world_patch_v3.py",
        SIM / "kty_station_sim" / "mechatronics_cycle_v3.py",
        SIM / "kty_station_sim" / "mechatronics_cycle_v10.py",
        SIM / "launch" / "kty_mechatronics_surface.launch.py",
    )
    for path in paths:
        py_compile.compile(str(path), doraise=True)

    transport = read(paths[1])
    for fragment in (
        "self._v3_ready.wait()",
        'LOCATOR_NAME = "kty_mech_runtime_locator"',
        "_spawn_locator_model",
        "_remove_locator_model",
        '"/kty/mech/infeed_surface/cmd_vel"',
        '"/kty/mech/active_surface/cmd_vel"',
        '"/kty/mech/outfeed_surface/cmd_vel"',
        'payload["transport"] = "flat_contact_surface_velocity"',
        'payload["last_nonzero_outfeed_mps"]',
    ):
        require(fragment in transport, f"Missing v9 transport behavior: {fragment}")

    vibration = read(paths[2])
    for fragment in (
        "class KtyMechatronicsCycleV10",
        'self.declare_parameter("strong_vibration_sweep_hz", 2.0)',
        'self.declare_parameter("strong_vibration_modulation_hz", 0.35)',
        '"vibration_profile": "vertical_frequency_sweep_v10"',
        '"last_compaction": dict(self._last_compaction)',
        "2.0 * math.pi * self.strong_frequency * elapsed",
        "self.strong_amplitude * envelope * math.sin(phase)",
        "height_before - height_after",
    ):
        require(fragment in vibration, f"Missing v10 vibration behavior: {fragment}")

    launch = read(paths[3])
    for fragment in (
        "build_surface_world",
        'get_package_prefix("kty_conveyor_surface")',
        'name="GZ_SIM_SYSTEM_PLUGIN_PATH"',
        'executable="mechatronics_cycle_v3"',
        '"world_name": "kty_mechatronics_surface"',
        'default_value="1.15"',
        '"roller_linear_speed_mps": 0.65',
        '"weak_vibration_frequency_hz": 6.0',
        '"weak_vibration_amplitude_m": 0.0012',
        '"strong_vibration_frequency_hz": 10.0',
        '"strong_vibration_sweep_hz": 2.0',
        '"strong_vibration_modulation_hz": 0.35',
        '"strong_vibration_amplitude_m": 0.0050',
        '"strong_vibration_duration_s": 12.0',
        '"strong_vibration_ramp_s": 1.5',
        '"processing_hz": 3.0',
        '"refresh_hz": 4.0',
    ):
        require(fragment in launch, f"Missing launch wiring: {fragment}")
    require("_rollers/cmd_vel" not in launch, "Launch still bridges rollers")


def validate_package_and_scripts() -> None:
    setup = read(SIM / "setup.py")
    package_xml = read(SIM / "package.xml")
    require('version="0.5.0"' in setup, "Expected version 0.5.0")
    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v10:main" in setup,
        "Accepted v3 executable must route to the v10 vibration runtime",
    )
    require(
        "mechatronics_cycle_v10 = kty_station_sim.mechatronics_cycle_v10:main" in setup,
        "Missing explicit v10 entry point",
    )
    require("<version>0.5.0</version>" in package_xml, "package.xml mismatch")
    require("<exec_depend>kty_conveyor_surface</exec_depend>" in package_xml, "Missing dependency")

    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")
    run = read(ROOT / "scripts" / "run_kty_perception_3d.sh")
    for fragment in (
        "kty_conveyor_surface",
        "mechatronics_cycle_v3",
        "libKtyConveyorSurfaceSystem.so",
        "validate_kty_contact_surface.py",
    ):
        require(fragment in build, f"Missing build behavior: {fragment}")
    require("kty_mechatronics_surface.launch.py" in run, "Wrong run launch")


def main() -> None:
    validate_plugin_package()
    validate_generated_world()
    validate_python_and_launch()
    validate_package_and_scripts()
    print("KTY deterministic transport and effective vibration validation: OK")


if __name__ == "__main__":
    main()
