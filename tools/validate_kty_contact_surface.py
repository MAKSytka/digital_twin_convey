#!/usr/bin/env python3
"""Static validation for deterministic KTY transport and runtime-v12 guards."""

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


def floats(text: str | None) -> list[float]:
    require(bool(text), "Expected numeric SDF text")
    return [float(value) for value in str(text).split()]


def top_z(link: ET.Element) -> float:
    pose = floats(link.findtext("pose"))
    size = floats(link.findtext("collision/geometry/box/size"))
    return pose[2] + 0.5 * size[2]


def validate_plugin() -> None:
    package_xml = read(PLUGIN / "package.xml")
    cmake = read(PLUGIN / "CMakeLists.txt")
    source = read(PLUGIN / "src" / "KtyConveyorSurfaceSystem.cc")
    for fragment in (
        "<name>kty_conveyor_surface</name>",
        "<build_type>ament_cmake</build_type>",
        "gz-sim8",
        "add_library(KtyConveyorSurfaceSystem SHARED",
        "LIBRARY DESTINATION lib",
        "class KtyConveyorSurfaceSystem",
        "link.SetLinearVelocity",
        "link.SetAngularVelocity",
        "targetVelocity = commands[index]",
        "GZ_ADD_PLUGIN",
        'mutableSdf->GetElement("zone")',
    ):
        require(
            fragment in package_xml or fragment in cmake or fragment in source,
            f"Missing contact-surface fragment: {fragment}",
        )
    require(
        '_sdf->GetElement("zone")' not in source,
        "sdformat traversal regressed to non-const call",
    )


def validate_world() -> None:
    sys.path.insert(0, str(SIM))
    from kty_station_sim.world_patch_v3 import build_surface_world

    with tempfile.TemporaryDirectory() as directory:
        destination = Path(directory) / "surface.sdf"
        build_surface_world(SIM / "worlds" / "kty_mechatronics.sdf", destination)
        root = ET.parse(destination).getroot()

    world = root.find("world")
    require(world is not None, "Generated world missing")
    require(world.attrib.get("name") == "kty_mechatronics_surface", "Wrong world")
    require(world.findtext("physics/max_step_size") == "0.004", "Wrong physics step")
    require(world.findtext("physics/real_time_update_rate") == "250", "Wrong update rate")

    machine = world.find("model[@name='kty_mechatronics_machine']")
    require(machine is not None, "Machine missing")
    links = {item.attrib.get("name", "") for item in machine.findall("link")}
    joints = {item.attrib.get("name", "") for item in machine.findall("joint")}
    require(not any("roller" in name for name in links | joints), "Rollers remain")
    for name in ("infeed_surface", "outfeed_transfer_bridge", "outfeed_surface"):
        require(name in links, f"Missing flat surface: {name}")

    vibration = machine.find("joint[@name='vibration_joint']")
    require(vibration is not None, "Vibration joint missing")
    expected = {
        "axis/limit/lower": "-0.010",
        "axis/limit/upper": "0.010",
        "axis/limit/effort": "16000",
        "axis/limit/velocity": "2.0",
        "axis/dynamics/damping": "60",
        "axis/dynamics/friction": "2",
    }
    for path, value in expected.items():
        require(vibration.findtext(path) == value, f"Wrong vibration setting {path}")

    controller = next(
        (
            plugin
            for plugin in machine.findall("plugin")
            if plugin.findtext("joint_name", default="") == "vibration_joint"
        ),
        None,
    )
    require(controller is not None, "Vibration controller missing")
    for path, value in {
        "p_gain": "420000",
        "d_gain": "2800",
        "cmd_max": "16000",
        "cmd_min": "-16000",
    }.items():
        require(controller.findtext(path) == value, f"Wrong controller {path}")

    bridge = machine.find("link[@name='outfeed_transfer_bridge']")
    outfeed = machine.find("link[@name='outfeed_surface']")
    require(bridge is not None and outfeed is not None, "Outfeed geometry missing")
    require(abs(top_z(bridge) - 0.498) < 1.0e-9, "Bridge top must be 498 mm")
    require(abs(top_z(outfeed) - 0.496) < 1.0e-9, "Outfeed top must be 496 mm")

    plugin = world.find("plugin[@name='kty_conveyor_surface::KtyConveyorSurfaceSystem']")
    require(plugin is not None, "Surface plugin missing")
    topics = {zone.findtext("topic") for zone in plugin.findall("zone")}
    require(
        topics
        == {
            "/kty/mech/infeed_surface/cmd_vel",
            "/kty/mech/active_surface/cmd_vel",
            "/kty/mech/outfeed_surface/cmd_vel",
        },
        f"Unexpected surface topics: {topics}",
    )


def validate_runtime() -> None:
    paths = {
        "v3": SIM / "kty_station_sim" / "mechatronics_cycle_v3.py",
        "v10": SIM / "kty_station_sim" / "mechatronics_cycle_v10.py",
        "v11": SIM / "kty_station_sim" / "mechatronics_cycle_v11.py",
        "v12": SIM / "kty_station_sim" / "mechatronics_cycle_v12.py",
        "launch": SIM / "launch" / "kty_mechatronics_surface.launch.py",
    }
    for path in paths.values():
        py_compile.compile(str(path), doraise=True)

    v11 = read(paths["v11"])
    for fragment in (
        "class KtyMechatronicsCycleV11",
        "self.strong_amplitude = 0.0080",
        '"DESPAWN_ACTIVE"',
        "_remove_model_confirmed",
        "_despawn_loaded_kty(old_kty, old_products)",
        '"changeover_order": "eject_despawn_position_next"',
    ):
        require(fragment in v11, f"Missing v11 behavior: {fragment}")
    require(
        v11.index("_despawn_loaded_kty(old_kty, old_products)")
        < v11.index('"POSITION_NEXT"'),
        "Old KTY must despawn before POSITION_NEXT",
    )

    v12 = read(paths["v12"])
    for fragment in (
        "class KtyMechatronicsCycleV12",
        '"minimum_load_duration_s": 4.0',
        '"minimum_products_for_close": 3',
        '"height_guard_min_fill_ratio": 0.10',
        '"height_guard_min_occupied_ratio": 0.18',
        '"position_next_timeout_s": 60.0',
        '"readiness_timeout_s": 30.0',
        "volume_reached = (",
        "height_reached = (",
        "enough_products",
        "fill_fresh",
        "_ensure_gate_open",
        "_position_recovery_pulses",
        '"runtime_profile": "kty_mechatronics_v12"',
        '"load_threshold_policy": "guarded_volume_or_supported_height"',
    ):
        require(fragment in v12, f"Missing v12 behavior: {fragment}")
    require(
        "maximum_height >= self.height_threshold\n                and fill_ratio >= self.height_guard_fill"
        in v12,
        "Height threshold lacks volume support guard",
    )

    launch = read(paths["launch"])
    for fragment in (
        'executable="mechatronics_cycle_v3"',
        '"weak_vibration_amplitude_m": 0.0018',
        '"strong_vibration_amplitude_m": 0.0080',
        '"minimum_load_duration_s": 4.0',
        '"minimum_products_for_close": 3',
        '"position_next_timeout_s": 60.0',
        '"readiness_timeout_s": 30.0',
        '"empty_kty_fill_limit": 0.22',
    ):
        require(fragment in launch, f"Missing launch v12 setting: {fragment}")
    require("_rollers/cmd_vel" not in launch, "Launch still bridges rollers")

    setup = read(SIM / "setup.py")
    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v12:main" in setup,
        "Accepted entry point must route to v12",
    )
    require(
        "mechatronics_cycle_v12 = kty_station_sim.mechatronics_cycle_v12:main" in setup,
        "Explicit v12 entry point missing",
    )


def validate_scripts() -> None:
    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")
    run = read(ROOT / "scripts" / "run_kty_perception_3d.sh")
    runtime_check = read(ROOT / "scripts" / "check_kty_runtime_v7.sh")
    for fragment in (
        "kty_conveyor_surface",
        "mechatronics_cycle_v3",
        "libKtyConveyorSurfaceSystem.so",
        "validate_kty_contact_surface.py",
    ):
        require(fragment in build, f"Missing build behavior: {fragment}")
    require("kty_mechatronics_surface.launch.py" in run, "Wrong run launch")
    require("DESPAWN_ACTIVE" in runtime_check, "Runtime check must require despawn")
    require("second_load" in runtime_check, "Runtime check must require second LOAD")


def main() -> None:
    validate_plugin()
    validate_world()
    validate_runtime()
    validate_scripts()
    print("KTY deterministic transport and runtime-v12 validation: OK")


if __name__ == "__main__":
    main()
