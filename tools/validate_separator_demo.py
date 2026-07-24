#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "src" / "singulator_description" / "models"
MODEL = MODEL_DIR / "infeed_size_separator" / "model.sdf"
SHAFT = MODEL_DIR / "separator_star_shaft" / "model.sdf"
WORLD = (
    ROOT
    / "src"
    / "singulator_gazebo"
    / "worlds"
    / "infeed_size_separator_demo.sdf"
)
BRIDGE = (
    ROOT
    / "src"
    / "singulator_bringup"
    / "config"
    / "bridge_separator_demo.yaml"
)
LAUNCH = (
    ROOT
    / "src"
    / "singulator_bringup"
    / "launch"
    / "infeed_size_separator_demo.launch.py"
)
CONTROLLER = (
    ROOT
    / "src"
    / "singulator_control"
    / "singulator_control"
    / "separator_demo_controller.py"
)
SPAWNER = (
    ROOT
    / "src"
    / "singulator_sim"
    / "singulator_sim"
    / "separator_demo_spawner.py"
)
CLEANUP = (
    ROOT
    / "src"
    / "singulator_sim"
    / "singulator_sim"
    / "separator_demo_cleanup.py"
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def vector(text: str | None) -> list[float]:
    require(text is not None, "Expected numeric vector is missing")
    return [float(value) for value in text.split()]


def belt_size(
    root: ET.Element,
    link_name: str,
    collision_name: str,
) -> list[float]:
    link = root.find(f".//link[@name='{link_name}']")
    require(link is not None, f"{link_name} is missing")
    return vector(
        link.findtext(
            f"./collision[@name='{collision_name}']/geometry/box/size"
        )
    )


def main() -> None:
    model_root = ET.parse(MODEL).getroot()
    shaft_root = ET.parse(SHAFT).getroot()
    world_root = ET.parse(WORLD).getroot()

    require(
        world_root.find(".//world[@name='infeed_size_separator_demo']")
        is not None,
        "Demo world name is missing",
    )

    separator_includes = [
        node
        for node in world_root.findall(".//include")
        if node.findtext("uri") == "model://infeed_size_separator"
    ]
    require(len(separator_includes) == 1, "Separator body include is missing")

    shaft_includes = [
        node
        for node in world_root.findall(".//include")
        if node.findtext("uri") == "model://separator_star_shaft"
    ]
    require(len(shaft_includes) == 11, "Expected 11 physical shafts")
    require(
        not any(
            node.findtext("uri") == "model://separator_star_shaft_odd"
            for node in world_root.findall(".//include")
        ),
        "Legacy nested odd-shaft model is still used",
    )

    x_values = sorted(
        vector(node.findtext("pose"))[0]
        for node in shaft_includes
    )
    shaft_pitch = min(
        right - left
        for left, right in zip(x_values, x_values[1:])
    )
    require(
        abs(shaft_pitch - 0.120) < 1e-6,
        "Longitudinal shaft pitch must be 120 mm",
    )

    entry = belt_size(model_root, "entry_belt", "entry_belt_collision")
    accepted = belt_size(
        model_root,
        "accepted_belt",
        "accepted_belt_collision",
    )
    reject = belt_size(model_root, "reject_belt", "reject_belt_collision")
    reject_transfer = belt_size(
        model_root,
        "reject_transfer_belt",
        "reject_transfer_belt_collision",
    )

    for name, size in (
        ("entry", entry),
        ("accepted", accepted),
        ("reject", reject),
    ):
        require(abs(size[0] - 3.0) < 1e-9, f"{name} length must be 3 m")
        require(abs(size[1] - 2.5) < 1e-9, f"{name} width must be 2.5 m")
    require(
        abs(reject_transfer[1] - 2.5) < 1e-9,
        "Lower catch belt must cover the full 2.5 m width",
    )

    require(
        not shaft_root.findall(".//include"),
        "A shaft must be one model, not nested per-disc models",
    )
    rotor = shaft_root.find(".//link[@name='rotor']")
    require(rotor is not None, "Consolidated rotor link is missing")
    rotor_pose = vector(rotor.findtext("pose"))
    require(
        abs(rotor_pose[2] - 0.055) < 1e-9,
        "Rotor axis must be at z=55 mm",
    )

    disc_collisions = [
        node
        for node in rotor.findall("./collision")
        if node.get("name", "").startswith("disc_collision_")
    ]
    require(
        len(disc_collisions) == 25,
        "A shaft must contain 25 disc collisions",
    )
    disc_y = sorted(
        vector(node.findtext("pose"))[1]
        for node in disc_collisions
    )
    transverse_pitch = min(
        right - left
        for left, right in zip(disc_y, disc_y[1:])
    )

    first_disc = disc_collisions[0]
    radius_text = first_disc.findtext("./geometry/cylinder/radius")
    thickness_text = first_disc.findtext("./geometry/cylinder/length")
    require(radius_text is not None, "Disc radius is missing")
    require(thickness_text is not None, "Disc thickness is missing")
    disc_radius = float(radius_text)
    disc_thickness = float(thickness_text)
    require(abs(disc_radius - 0.025) < 1e-9, "Disc radius must be 25 mm")
    require(
        abs(disc_thickness - 0.030) < 1e-9,
        "Disc thickness must be 30 mm",
    )

    longitudinal_gap = shaft_pitch - 2.0 * disc_radius
    transverse_gap = transverse_pitch - disc_thickness
    require(
        abs(longitudinal_gap - 0.070) < 1e-6,
        "Longitudinal clear opening must be 70 mm",
    )
    require(
        abs(transverse_gap - 0.070) < 1e-6,
        "Transverse clear opening must be 70 mm",
    )

    joints = shaft_root.findall(
        ".//joint[@name='shaft_joint'][@type='revolute']"
    )
    require(
        len(joints) == 1,
        "Each shaft must have exactly one revolute joint",
    )
    require(
        vector(joints[0].findtext("./axis/xyz")) == [0.0, 1.0, 0.0],
        "Shaft must rotate around Y",
    )
    controllers = shaft_root.findall(
        ".//plugin[@filename='gz-sim-joint-controller-system']"
    )
    require(
        len(controllers) == 1,
        "Each shaft must have exactly one JointController",
    )
    require(
        controllers[0].findtext("topic")
        == "/singulator/separator/screen/cmd_vel",
        "Shaft command topic is incorrect",
    )

    require(
        world_root.find(".//gui/plugin[@filename='CameraTracking']")
        is not None,
        "CameraTracking GUI plugin is missing",
    )
    require(
        world_root.find(".//gui/plugin[@filename='GzSceneManager']")
        is not None,
        "GzSceneManager GUI plugin is missing",
    )
    require(
        world_root.findtext(
            ".//gui/plugin[@filename='MinimalScene']/camera_pose"
        )
        is not None,
        "Initial camera pose is missing",
    )

    bridge_text = BRIDGE.read_text(encoding="utf-8")
    for suffix in (
        "infeed",
        "screen",
        "accepted",
        "reject_transfer",
        "reject",
    ):
        require(
            f"/singulator/separator/{suffix}/cmd_vel" in bridge_text,
            f"Missing bridge topic for {suffix}",
        )

    launch_text = LAUNCH.read_text(encoding="utf-8")
    for token in (
        'default_value="continuous"',
        'default_value="4.0"',
        'default_value="0.20"',
        'default_value="2.0"',
        'default_value="0.002"',
        'default_value="0.02"',
        'default_value="0.35"',
        'default_value="0.30"',
        "separator_demo_controller",
        "separator_demo_spawner",
        "separator_demo_cleanup",
    ):
        require(token in launch_text, f"Launch token is missing: {token}")

    controller_text = CONTROLLER.read_text(encoding="utf-8")
    for token in (
        'self.declare_parameter("shaft_collision_radius_m", 0.025)',
        "angular_speed = surface_speed / radius",
        "rpm = angular_speed * 60.0 / (2.0 * math.pi)",
        "disc_contact_radius",
    ):
        require(
            token in controller_text,
            f"Roller speed token missing: {token}",
        )

    expected_angular_speed = 2.0 / disc_radius
    require(
        abs(expected_angular_speed - 80.0) < 1e-9,
        "A 25 mm radius must require 80 rad/s for 2 m/s",
    )

    spawner_text = SPAWNER.read_text(encoding="utf-8")
    require(
        "range(10)" in spawner_text,
        "Spawner does not define ten fixed spots",
    )
    require(
        "minimum_projection < self.cutoff" in spawner_text,
        "Spawner does not use the two-projection cutoff rule",
    )
    for profile in (
        "micro_parcel",
        "long_narrow",
        "flat_strip",
        "tall_slender",
        "near_cutoff",
        "medium_carton",
        "large_carton",
        "long_parcel",
        "flat_panel",
        "tall_carton",
        "square_carton",
    ):
        require(
            f'"{profile}"' in spawner_text,
            f"Box profile is missing: {profile}",
        )
    for token in (
        'self.declare_parameter("small_item_probability", 0.20)',
        'self.declare_parameter("spawn_clearance_m", 0.002)',
        'self.declare_parameter("box_restitution", 0.02)',
        'self.declare_parameter("angular_velocity_decay", 0.30)',
        "cardboard_areal_density",
        "profile.minimum_mass_kg",
        "<velocity_decay>",
        "<restitution_coefficient>",
        "<max_vel>",
        "<allow_auto_disable>true</allow_auto_disable>",
    ):
        require(token in spawner_text, f"Spawner dynamics token missing: {token}")
    require(
        "transport_assist" not in spawner_text,
        "Artificial persistent-force transport must not be used",
    )

    cleanup_text = CLEANUP.read_text(encoding="utf-8")
    for token in (
        "upper_exit_x_m",
        "lower_exit_x_m",
        "mismatch_count",
        "Separator statistics",
    ):
        require(
            token in cleanup_text,
            f"Cleanup/statistics token missing: {token}",
        )

    control_setup = (
        ROOT / "src" / "singulator_control" / "setup.py"
    ).read_text(encoding="utf-8")
    sim_setup = (
        ROOT / "src" / "singulator_sim" / "setup.py"
    ).read_text(encoding="utf-8")
    require(
        "separator_demo_controller" in control_setup,
        "Controller entry point is missing",
    )
    require(
        "separator_demo_spawner" in sim_setup,
        "Spawner entry point is missing",
    )
    require(
        "separator_demo_cleanup" in sim_setup,
        "Cleanup entry point is missing",
    )

    rpm = expected_angular_speed * 60.0 / (2.0 * 3.141592653589793)
    print(
        "Separator demo static validation passed: "
        f"width={entry[1]:.3f} m, "
        f"output_length={accepted[0]:.3f} m, "
        f"shafts={len(shaft_includes)}, "
        f"discs_per_shaft={len(disc_collisions)}, "
        f"openings={longitudinal_gap:.3f}/{transverse_gap:.3f} m, "
        f"roller_radius={disc_radius:.3f} m, "
        f"omega_2mps={expected_angular_speed:.1f} rad/s, "
        f"rpm_2mps={rpm:.1f}, profiles=11"
    )


if __name__ == "__main__":
    main()
