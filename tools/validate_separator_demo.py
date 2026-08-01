#!/usr/bin/env python3
"""Static validation for the continuous-roller infeed separator demo."""

from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "src/singulator_description/models"
SEPARATOR = MODEL_DIR / "infeed_size_separator/model.sdf"
ROLLER = MODEL_DIR / "separator_star_shaft/model.sdf"
WORLD = ROOT / "src/singulator_gazebo/worlds/infeed_size_separator_demo.sdf"
LAUNCH = ROOT / "src/singulator_bringup/launch/infeed_size_separator_demo.launch.py"
BRIDGE = ROOT / "src/singulator_bringup/config/bridge_separator_demo.yaml"
CONTROLLER = ROOT / "src/singulator_control/singulator_control/separator_demo_controller.py"
SPAWNER_BASE = ROOT / "src/singulator_sim/singulator_sim/separator_demo_spawner.py"
SPAWNER = ROOT / "src/singulator_sim/singulator_sim/separator_continuous_roller_spawner.py"
CLEANUP = ROOT / "src/singulator_sim/singulator_sim/separator_demo_cleanup.py"
SIM_SETUP = ROOT / "src/singulator_sim/setup.py"

EXPECTED_ROLLERS = 11
EXPECTED_PITCH_M = 0.150
EXPECTED_RADIUS_M = 0.025
EXPECTED_OPENING_M = 0.100
EXPECTED_GRAVITY_Z_MPS2 = -12.0


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def values(text: str | None) -> list[float]:
    require(text is not None, "Numeric field is missing")
    return [float(value) for value in text.split()]


def named_link(root: ET.Element, name: str) -> ET.Element:
    node = root.find(f".//link[@name='{name}']")
    require(node is not None, f"Missing link: {name}")
    return node


def box_size(link: ET.Element, collision: str) -> list[float]:
    return values(
        link.findtext(
            f"./collision[@name='{collision}']/geometry/box/size"
        )
    )


def link_pose(link: ET.Element) -> list[float]:
    return values(link.findtext("pose"))


def main() -> None:
    separator = ET.parse(SEPARATOR).getroot()
    roller = ET.parse(ROLLER).getroot()
    world = ET.parse(WORLD).getroot()

    world_node = world.find(".//world[@name='infeed_size_separator_demo']")
    require(world_node is not None, "Unexpected separator world name")
    gravity = values(world_node.findtext("gravity"))
    require(
        gravity == [0.0, 0.0, EXPECTED_GRAVITY_Z_MPS2],
        "Separator demo gravity must be 0 0 -12.0 m/s^2",
    )

    includes = [
        node
        for node in world.findall(".//include")
        if node.findtext("uri") == "model://separator_star_shaft"
    ]
    require(
        len(includes) == EXPECTED_ROLLERS,
        f"Expected {EXPECTED_ROLLERS} continuous rollers",
    )
    xs = sorted(values(node.findtext("pose"))[0] for node in includes)
    pitches = [right - left for left, right in zip(xs, xs[1:])]
    require(pitches, "Roller pitch list is empty")
    require(
        all(abs(pitch - EXPECTED_PITCH_M) < 1e-6 for pitch in pitches),
        "Roller pitch must be 150 mm for every adjacent pair",
    )
    pitch = pitches[0]

    rotor = roller.find(".//link[@name='rotor']")
    require(rotor is not None, "Rotor link is missing")
    axis_z = values(rotor.findtext("pose"))[2]
    require(abs(axis_z - 0.055) < 1e-9, "Roller axis Z must be 55 mm")

    collision = rotor.find("./collision[@name='roller_collision']")
    visual = rotor.find("./visual[@name='roller_visual']")
    require(collision is not None, "Continuous roller collision is missing")
    require(visual is not None, "Continuous roller visual is missing")
    radius = float(collision.findtext("./geometry/cylinder/radius"))
    length = float(collision.findtext("./geometry/cylinder/length"))
    visual_radius = float(visual.findtext("./geometry/cylinder/radius"))
    visual_length = float(visual.findtext("./geometry/cylinder/length"))
    require(abs(radius - EXPECTED_RADIUS_M) < 1e-9, "Roller radius must be 25 mm")
    require(abs(length - 2.480) < 1e-9, "Roller length must be 2.480 m")
    require(
        radius == visual_radius and length == visual_length,
        "Roller visual and collision differ",
    )
    require(
        not any(
            node.get("name", "").startswith("disc_collision_")
            for node in rotor.findall("./collision")
        ),
        "Legacy segmented discs are still present",
    )
    opening = pitch - 2.0 * radius
    require(
        abs(opening - EXPECTED_OPENING_M) < 1e-6,
        "Longitudinal opening must be 100 mm",
    )

    joint = roller.find(".//joint[@name='shaft_joint'][@type='revolute']")
    require(joint is not None, "Roller revolute joint is missing")
    require(
        values(joint.findtext("./axis/xyz")) == [0.0, 1.0, 0.0],
        "Roller axis must be Y",
    )
    require(
        roller.find(".//plugin[@filename='gz-sim-joint-controller-system']")
        is not None,
        "Roller JointController is missing",
    )

    entry = named_link(separator, "entry_belt")
    accepted = named_link(separator, "accepted_belt")
    reject = named_link(separator, "reject_belt")
    reject_transfer = named_link(separator, "reject_transfer_belt")
    frame = named_link(separator, "frame")
    for name, link, collision_name in (
        ("entry", entry, "entry_belt_collision"),
        ("accepted", accepted, "accepted_belt_collision"),
        ("reject", reject, "reject_belt_collision"),
    ):
        size = box_size(link, collision_name)
        require(abs(size[0] - 3.0) < 1e-9, f"{name} length must be 3 m")
        require(abs(size[1] - 2.5) < 1e-9, f"{name} width must be 2.5 m")
    transfer_size = box_size(
        reject_transfer,
        "reject_transfer_belt_collision",
    )
    require(
        abs(transfer_size[0] - 1.7) < 1e-9,
        "Lower transfer belt must be 1.7 m long for the wider screen",
    )
    require(
        abs(transfer_size[1] - 2.5) < 1e-9,
        "Lower transfer belt must cover 2.5 m",
    )
    require(
        abs(box_size(frame, "left_guard_collision")[0] - 7.65) < 1e-9,
        "Side guards must cover the widened separator",
    )
    require(
        abs(box_size(frame, "left_screen_cheek_collision")[0] - 1.62) < 1e-9,
        "Screen cheeks must cover the widened roller bank",
    )

    entry_x, _, entry_z, *_ = link_pose(entry)
    accepted_x, _, accepted_z, *_ = link_pose(accepted)
    reject_x, _, reject_z, *_ = link_pose(reject)
    transfer_x, _, transfer_z, *_ = link_pose(reject_transfer)

    first_roller_left = xs[0] - radius
    last_roller_right = xs[-1] + radius
    entry_right = entry_x + 1.5
    accepted_left = accepted_x - 1.5
    transfer_right = transfer_x + transfer_size[0] / 2.0
    reject_left = reject_x - 1.5

    require(
        abs(first_roller_left - entry_right - 0.009) < 1e-6,
        "Entry-to-screen gap must remain 9 mm",
    )
    require(
        abs(accepted_left - last_roller_right - 0.001) < 1e-6,
        "Screen-to-upper-output gap must remain 1 mm",
    )
    require(
        abs(reject_left - transfer_right - 0.001) < 1e-6,
        "Lower transfer-to-output gap must remain 1 mm",
    )

    roller_crest_z = axis_z + radius
    accepted_top_z = accepted_z + 0.040
    transfer_top_z = transfer_z + 0.040
    reject_top_z = reject_z + 0.040
    require(
        abs(roller_crest_z - accepted_top_z - 0.004) < 1e-6,
        "Upper output must remain 4 mm below the roller crest",
    )
    require(
        abs(transfer_top_z - reject_top_z - 0.004) < 1e-6,
        "Lower output must remain 4 mm below the transfer belt",
    )

    launch = LAUNCH.read_text(encoding="utf-8")
    for token in (
        'default_value="continuous"',
        'default_value="4.0"',
        'default_value="0.70"',
        '"upper_safety_projection_m": 0.110',
        '"box_restitution",\n                default_value="0.0"',
        '"bounce_capture_velocity_mps",\n                default_value="1.0"',
        '"linear_velocity_decay",\n                default_value="0.12"',
        '"angular_velocity_decay",\n                default_value="0.60"',
        '"contact_max_correcting_velocity_mps",\n                default_value="0.02"',
        '"spawn_clearance_m",\n                default_value="0.001"',
        '"remove_retries": 3',
    ):
        require(token in launch, f"Launch token is missing: {token}")

    bridge = BRIDGE.read_text(encoding="utf-8")
    for suffix in (
        "infeed",
        "screen",
        "accepted",
        "reject_transfer",
        "reject",
    ):
        require(
            f"/singulator/separator/{suffix}/cmd_vel" in bridge,
            f"Bridge topic is missing: {suffix}",
        )

    controller = CONTROLLER.read_text(encoding="utf-8")
    require(
        "angular_speed = surface_speed / radius" in controller,
        "Surface-speed calculation is missing",
    )
    require(
        "rpm = angular_speed * 60.0 / (2.0 * math.pi)" in controller,
        "RPM calculation is missing",
    )
    require(abs(2.0 / radius - 80.0) < 1e-9, "2 m/s must produce 80 rad/s")

    base_spawner = SPAWNER_BASE.read_text(encoding="utf-8")
    require(
        re.search(
            r"declare_parameter\(\s*[\"']small_item_probability[\"']",
            base_spawner,
        )
        is not None,
        "small_item_probability is missing",
    )
    for token in (
        "<restitution_coefficient>",
        "<velocity_decay>",
        "contact_max_correcting_velocity_mps",
    ):
        require(token in base_spawner, f"Impact-model token is missing: {token}")

    spawner = SPAWNER.read_text(encoding="utf-8")
    require(
        "classified_lower = projection_x < self.cutoff" in spawner,
        "Lower route must use longitudinal projection_x",
    )
    for profile in (
        "micro_parcel",
        "long_narrow",
        "flat_strip",
        "tall_slender",
        "near_cutoff",
    ):
        require(f'"{profile}"' in spawner, f"Lower profile is missing: {profile}")

    setup = SIM_SETUP.read_text(encoding="utf-8")
    require(
        "separator_demo_spawner = "
        "singulator_sim.separator_continuous_roller_spawner:main"
        in setup,
        "Console entry point does not use continuous-roller spawner",
    )

    cleanup = CLEANUP.read_text(encoding="utf-8")
    for token in ("remove_retries", "monitor_restarts", "Separator statistics"):
        require(token in cleanup, f"Cleanup token is missing: {token}")

    rpm = 80.0 * 60.0 / (2.0 * 3.141592653589793)
    print(
        "Separator demo static validation passed: "
        f"rollers={len(includes)}, roller_length={length:.3f} m, "
        f"roller_pitch={pitch:.3f} m, "
        f"longitudinal_opening={opening:.3f} m, "
        f"gravity_z={gravity[2]:.1f} m/s^2, "
        f"omega_2mps=80.0 rad/s, rpm_2mps={rpm:.1f}, "
        "lower_cutoff=0.070 m, upper_safety=0.110 m, "
        "restitution=0.00, bounce_threshold=1.00 m/s, "
        "velocity_decay=0.12/0.60, contact_max_vel=0.02 m/s, "
        "lower_probability=0.70"
    )


if __name__ == "__main__":
    main()
