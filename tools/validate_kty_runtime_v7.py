#!/usr/bin/env python3
"""Static validation for KTY runtime-v7 compatibility and its v8 successor."""

from __future__ import annotations

from pathlib import Path
import py_compile
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "kty_station_sim"
SOURCE_WORLD = PACKAGE / "worlds" / "kty_mechatronics.sdf"
PATCHER = PACKAGE / "kty_station_sim" / "world_patch_v2.py"
CONTROLLER = PACKAGE / "kty_station_sim" / "mechatronics_cycle_v2.py"
FILL = PACKAGE / "kty_station_sim" / "fill_estimator_v2.py"
PERCEPTION = PACKAGE / "kty_station_sim" / "depth_perception_3d_v2.py"
LAUNCH = PACKAGE / "launch" / "kty_mechatronics_v2.launch.py"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_python() -> None:
    for path in (PATCHER, CONTROLLER, FILL, PERCEPTION, LAUNCH):
        py_compile.compile(str(path), doraise=True)


def validate_generated_world() -> None:
    import importlib.util

    spec = importlib.util.spec_from_file_location("world_patch_v2", PATCHER)
    require(spec is not None and spec.loader is not None, "Cannot load world patch")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "world.sdf"
        module.build_balanced_world(SOURCE_WORLD, output)
        root = ET.parse(output).getroot()

    world = root.find("world")
    require(world is not None, "Generated world is missing")
    require(world.attrib.get("name") == "kty_mechatronics_v2", "Wrong world name")
    require(world.findtext("physics/max_step_size") == "0.002", "Expected 2 ms step")
    require(
        world.findtext("physics/real_time_update_rate") == "500",
        "Expected 500 Hz physics target",
    )
    sensor = world.find(".//sensor[@name='overhead_rgbd']")
    require(sensor is not None, "RGB-D sensor missing")
    require(sensor.findtext("update_rate") == "8", "Expected balanced 8 Hz camera")
    require(sensor.findtext("camera/image/width") == "640", "Expected 640 px RGB-D")
    require(sensor.findtext("camera/image/height") == "480", "Expected 480 px RGB-D")
    require(sensor.findtext("visualize") == "false", "Sensor visualization must be off")

    machine = world.find("model[@name='kty_mechatronics_machine']")
    require(machine is not None, "Machine missing")
    require(machine.find("link[@name='gate']") is None, "Obsolete hinged gate remains")
    require(machine.find("joint[@name='gate_joint']") is None, "Obsolete gate joint remains")

    # The v7 generator remains in the repository as a reproducible legacy
    # baseline even though the default runtime now uses flat contact surfaces.
    roller_links = [
        link for link in machine.findall("link") if "_roller_" in link.attrib.get("name", "")
    ]
    require(len(roller_links) == 17, f"Expected 17 corrected legacy rollers, found {len(roller_links)}")
    for link in roller_links:
        name = link.attrib["name"]
        pose = [float(value) for value in link.findtext("pose", "").split()]
        require(len(pose) == 6 and pose[3:] == [0.0, 0.0, 0.0], f"Link frame rotated: {name}")
        collision_pose = [
            float(value) for value in link.findtext("collision/pose", "").split()
        ]
        require(
            len(collision_pose) == 6 and abs(collision_pose[3] - 1.57079632679) < 1e-8,
            f"Cylinder geometry is not aligned across Y: {name}",
        )
        joint = machine.find(f"joint[@name='{name}_joint']")
        require(joint is not None, f"Missing roller joint: {name}")
        joint_pose = [float(value) for value in joint.findtext("pose", "").split()]
        require(
            len(joint_pose) == 6 and abs(joint_pose[0] - pose[0]) < 1e-8,
            f"Roller joint not anchored at link centre: {name}",
        )
        require(joint.findtext("axis/xyz") == "0 1 0", f"Wrong axle: {name}")

    roller_plugins = [
        plugin
        for plugin in machine.findall("plugin")
        if plugin.attrib.get("filename") == "gz-sim-joint-controller-system"
        and "_rollers/cmd_vel" in plugin.findtext("topic", "")
    ]
    require(
        len(roller_plugins) == 17,
        f"Expected one velocity controller per legacy roller, found {len(roller_plugins)}",
    )
    require(
        all(len(plugin.findall("joint_name")) == 1 for plugin in roller_plugins),
        "Grouped legacy roller controller remains",
    )


def validate_runtime_wiring() -> None:
    controller = read(CONTROLLER)
    for fragment in (
        'GATE_NAME = "kty_mech_chute_gate"',
        "_spawn_gate_model",
        "_remove_gate_model",
        '"gate_mechanism"] = "spawned_static_slide"',
        "boosting active/outfeed surfaces",
        "max(18.0, timeout_s)",
        "retract locator and open clamps before energising contact surfaces",
        "self._interruptible_sleep(2.5)",
        "restraints clear; moving loaded KTY",
    ):
        require(fragment in controller, f"Missing controller correction: {fragment}")
    require("/set_pose" not in controller, "Runtime v7 must not teleport KTY")

    fill = read(FILL)
    for fragment in (
        'super().__init__("kty_fill_estimator_v2")',
        '"wall_exclusion_margin_m": 0.040',
        "maximum_product_height_m",
        "area_scale = (self.length * self.width) / (core_length * core_width)",
        '"schema": "kty_fill_state/v2"',
    ):
        require(fragment in fill, f"Missing fill correction: {fragment}")

    perception = read(PERCEPTION)
    for fragment in (
        "class KtyClassical3DPerceptionV2",
        '"/kty/perception/fault"',
        "except Exception as error",
        "rclpy.try_shutdown()",
        'if item.state == "OCCLUDED"',
    ):
        require(fragment in perception, f"Missing perception guard: {fragment}")

    launch = read(LAUNCH)
    for fragment in (
        "build_balanced_world(source_world, generated_world)",
        'executable="mechatronics_cycle_v2"',
        'executable="fill_estimator_v2"',
        'executable="depth_perception_3d_v2"',
        'default_value="1.15"',
        'DeclareLaunchArgument("show_dashboard", default_value="false")',
        '"world_name": "kty_mechatronics_v2"',
    ):
        require(fragment in launch, f"Missing launch correction: {fragment}")
    require("/kty/mech/gate/cmd_pos@" not in launch, "Obsolete gate bridge remains")

    setup = read(PACKAGE / "setup.py")
    for executable in (
        "mechatronics_cycle_v2",
        "fill_estimator_v2",
        "depth_perception_3d_v2",
    ):
        require(f'"{executable} = ' in setup, f"Missing entry point: {executable}")


def validate_scripts() -> None:
    for relative in (
        "scripts/run_kty_mechatronics.sh",
        "scripts/run_kty_perception_3d.sh",
        "scripts/build_kty_perception_3d.sh",
    ):
        text = read(ROOT / relative)
        require(text.startswith("#!/usr/bin/env bash"), f"Missing shebang: {relative}")

    run_mechatronics = read(ROOT / "scripts/run_kty_mechatronics.sh")
    run_perception = read(ROOT / "scripts/run_kty_perception_3d.sh")
    accepted_launches = (
        "kty_mechatronics_v2.launch.py",
        "kty_mechatronics_surface.launch.py",
    )
    require(
        any(name in run_mechatronics for name in accepted_launches),
        "Mechatronics script does not run a supported v7/v8 launch",
    )
    require(
        any(name in run_perception for name in accepted_launches),
        "Perception script does not run a supported v7/v8 launch",
    )


def main() -> None:
    validate_python()
    validate_generated_world()
    validate_runtime_wiring()
    validate_scripts()
    print("KTY runtime v7/v8 compatibility validation: OK")


if __name__ == "__main__":
    main()
