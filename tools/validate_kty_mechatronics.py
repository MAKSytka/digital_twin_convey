#!/usr/bin/env python3
"""Static validation for physical KTY mechatronics stage."""

from __future__ import annotations

from pathlib import Path
import py_compile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "kty_station_sim"
WORLD = PACKAGE / "worlds" / "kty_mechatronics.sdf"
LAUNCH = PACKAGE / "launch" / "kty_mechatronics.launch.py"
CONTROLLER = PACKAGE / "kty_station_sim" / "mechatronics_cycle.py"
FILL = PACKAGE / "kty_station_sim" / "fill_estimator.py"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_world() -> None:
    root = ET.parse(WORLD).getroot()
    require(root.tag == "sdf", "World root must be <sdf>")
    world = root.find("world")
    require(world is not None, "World element is missing")
    require(
        world.attrib.get("name") == "kty_mechatronics",
        "Unexpected mechatronics world name",
    )

    machine = world.find("model[@name='kty_mechatronics_machine']")
    require(machine is not None, "Mechatronics machine model is missing")
    links = {link.attrib.get("name") for link in machine.findall("link")}
    joints = {joint.attrib.get("name") for joint in machine.findall("joint")}

    for required_link in (
        "base",
        "vibration_deck",
        "pusher",
        "clamp_neg_y",
        "clamp_pos_y",
        "gate",
        "locator_stop",
    ):
        require(required_link in links, f"Missing machine link: {required_link}")

    for required_joint in (
        "vibration_joint",
        "pusher_joint",
        "clamp_neg_y_joint",
        "clamp_pos_y_joint",
        "gate_joint",
        "locator_stop_joint",
    ):
        require(required_joint in joints, f"Missing machine joint: {required_joint}")

    roller_links = [name for name in links if name and "_roller_" in name]
    require(
        len(roller_links) == 17,
        f"Expected 17 physical rollers, found {len(roller_links)}",
    )

    text = read(WORLD)
    for fragment in (
        "gz-sim-joint-controller-system",
        "gz-sim-joint-position-controller-system",
        "gz-sim-joint-state-publisher-system",
        "<topic>/kty/mech/infeed_rollers/cmd_vel</topic>",
        "<topic>/kty/mech/active_rollers/cmd_vel</topic>",
        "<topic>/kty/mech/outfeed_rollers/cmd_vel</topic>",
        "<topic>/kty/mech/pusher/cmd_pos</topic>",
        "<topic>/kty/mech/clamps/cmd_pos</topic>",
        "<topic>/kty/mech/gate/cmd_pos</topic>",
        "<topic>/kty/mech/vibration/cmd_pos</topic>",
        "<topic>/kty/mech/locator_stop/cmd_pos</topic>",
        "<lower>-0.004</lower><upper>0.004</upper>",
        "<upper>1.35</upper>",
        '<sensor name="overhead_rgbd" type="rgbd_camera">',
        "<update_rate>15</update_rate>",
    ):
        require(fragment in text, f"Missing world behavior: {fragment}")


def validate_python() -> None:
    for path in (CONTROLLER, FILL, LAUNCH):
        py_compile.compile(str(path), doraise=True)

    controller = read(CONTROLLER)
    for fragment in (
        'super().__init__("kty_mechatronics_cycle")',
        '"fill_ratio_threshold": 0.70',
        '"max_height_threshold_m": 0.280',
        '"weak_vibration_frequency_hz": 8.0',
        '"weak_vibration_amplitude_m": 0.0005',
        '"strong_vibration_frequency_hz": 18.0',
        '"strong_vibration_amplitude_m": 0.0030',
        '"strong_vibration_duration_s": 8.0',
        '"/kty/mech/infeed_rollers/cmd_vel"',
        '"/kty/mech/pusher/cmd_pos"',
        '"/kty/mech/clamps/cmd_pos"',
        '"/kty/mech/gate/cmd_pos"',
        '"/kty/mech/vibration/cmd_pos"',
        '"/kty/mech/locator_stop/cmd_pos"',
        '"CLOSE_GATE"',
        '"COMPACT"',
        '"EJECT_ACTIVE"',
        '"POSITION_NEXT"',
        '"VERIFY_READY"',
        '"OPEN_GATE"',
        "self._spawn_kty(self._active_kty",
        "self._spawn_kty(self._queue_kty",
        "abs(pose.x - self.active_target_x) <= self.position_tolerance",
        "velocity <= self.velocity_tolerance",
        "fill_ratio <= 0.15",
    ):
        require(fragment in controller, f"Missing controller behavior: {fragment}")

    require(
        "_set_pose" not in controller and "/set_pose" not in controller,
        "Physical controller must not teleport moving KTY models",
    )

    fill = read(FILL)
    for fragment in (
        'super().__init__("kty_fill_estimator")',
        '"/kty/fill/state"',
        "estimated_volume = float(np.sum(height_map[roi_pixels]) * pixel_area)",
        '"fill_ratio": fill_ratio',
        '"maximum_height_m": maximum_height',
        '"camera_ok": valid_fraction >= self.minimum_valid_fraction',
    ):
        require(fragment in fill, f"Missing fill estimator behavior: {fragment}")

    launch = read(LAUNCH)
    for fragment in (
        'worlds" / "kty_mechatronics.sdf"',
        'executable="parameter_bridge"',
        'executable="mechatronics_cycle"',
        'executable="fill_estimator"',
        'executable="depth_perception"',
        'executable="vision_dashboard"',
        '"/kty/mech/infeed_rollers/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double"',
        '"fill_ratio_threshold"',
        'default_value="0.70"',
        'default_value="0.280"',
    ):
        require(fragment in launch, f"Missing launch wiring: {fragment}")


def validate_package_and_scripts() -> None:
    setup = read(PACKAGE / "setup.py")
    require('version="0.5.0"' in setup, "Expected setup.py version 0.5.0")
    for fragment in (
        "mechatronics_cycle = kty_station_sim.mechatronics_cycle:main",
        "fill_estimator = kty_station_sim.fill_estimator:main",
        "depth_perception = kty_station_sim.depth_perception:main",
        "vision_dashboard = kty_station_sim.vision_dashboard:main",
    ):
        require(fragment in setup, f"Missing entry point: {fragment}")

    package_xml = read(PACKAGE / "package.xml")
    require(
        "<version>0.5.0</version>" in package_xml,
        "Expected package.xml version 0.5.0",
    )
    require(
        "<buildtool_depend>ament_python</buildtool_depend>" not in package_xml,
        "ament_python must remain a build type only",
    )

    scripts = (
        "scripts/build_kty_mechatronics.sh",
        "scripts/run_kty_mechatronics.sh",
        "scripts/check_kty_mechatronics.sh",
        "scripts/stop_kty_mechatronics.sh",
    )
    for relative in scripts:
        text = read(ROOT / relative)
        require(
            text.startswith("#!/usr/bin/env bash"),
            f"Missing shell shebang: {relative}",
        )

    diagnostic = read(ROOT / "scripts/check_kty_mechatronics.sh")
    for fragment in (
        "/kty_mechatronics_cycle",
        "/kty_fill_estimator",
        "complete physical KTY changeover observed",
        "at least two KTY models coexist",
        "fill >= 70% OR maximum height >= 280 mm",
        "KTY mechatronics diagnostics: OK",
    ):
        require(fragment in diagnostic, f"Missing diagnostic behavior: {fragment}")


def main() -> None:
    validate_world()
    validate_python()
    validate_package_and_scripts()
    print("KTY mechatronics static validation: OK")


if __name__ == "__main__":
    main()
