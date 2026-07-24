#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = (
    ROOT / "src" / "singulator_description" / "models"
)
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


def link(
    root: ET.Element,
    name: str,
) -> ET.Element:
    node = root.find(f".//link[@name='{name}']")
    require(node is not None, f"{name} link is missing")
    return node


def box_size(
    node: ET.Element,
    collision_name: str,
) -> list[float]:
    return vector(
        node.findtext(
            f"./collision[@name='{collision_name}']"
            "/geometry/box/size"
        )
    )


def pose(node: ET.Element) -> list[float]:
    return vector(node.findtext("pose"))


def main() -> None:
    model_root = ET.parse(MODEL).getroot()
    shaft_root = ET.parse(SHAFT).getroot()
    world_root = ET.parse(WORLD).getroot()

    require(
        world_root.find(
            ".//world[@name='infeed_size_separator_demo']"
        )
        is not None,
        "Demo world name is missing",
    )

    separator_includes = [
        node
        for node in world_root.findall(".//include")
        if node.findtext("uri")
        == "model://infeed_size_separator"
    ]
    require(
        len(separator_includes) == 1,
        "Separator body include is missing",
    )

    shaft_includes = [
        node
        for node in world_root.findall(".//include")
        if node.findtext("uri")
        == "model://separator_star_shaft"
    ]
    require(
        len(shaft_includes) == 11,
        "Expected 11 physical shafts",
    )
    x_values = sorted(
        vector(node.findtext("pose"))[0]
        for node in shaft_includes
    )
    shaft_pitch = min(
        right - left
        for left, right in zip(
            x_values,
            x_values[1:],
        )
    )
    require(
        abs(shaft_pitch - 0.120) < 1e-6,
        "Longitudinal shaft pitch must be 120 mm",
    )

    entry = link(model_root, "entry_belt")
    accepted = link(model_root, "accepted_belt")
    reject_transfer = link(
        model_root,
        "reject_transfer_belt",
    )
    reject = link(model_root, "reject_belt")

    for name, node, collision_name in (
        ("entry", entry, "entry_belt_collision"),
        (
            "accepted",
            accepted,
            "accepted_belt_collision",
        ),
        ("reject", reject, "reject_belt_collision"),
    ):
        size = box_size(node, collision_name)
        require(
            abs(size[0] - 3.0) < 1e-9,
            f"{name} length must be 3 m",
        )
        require(
            abs(size[1] - 2.5) < 1e-9,
            f"{name} width must be 2.5 m",
        )

    reject_transfer_size = box_size(
        reject_transfer,
        "reject_transfer_belt_collision",
    )
    require(
        abs(reject_transfer_size[1] - 2.5) < 1e-9,
        "Lower catch belt must cover 2.5 m",
    )

    rotor = shaft_root.find(".//link[@name='rotor']")
    require(
        rotor is not None,
        "Consolidated rotor link is missing",
    )
    rotor_pose = pose(rotor)
    require(
        abs(rotor_pose[2] - 0.055) < 1e-9,
        "Rotor axis must be at z=55 mm",
    )

    axle_collision = rotor.find(
        "./collision[@name='axle_collision']"
    )
    axle_visual = rotor.find(
        "./visual[@name='axle_visual']"
    )
    require(
        axle_collision is not None,
        "Physical axle collision is missing",
    )
    require(
        axle_visual is not None,
        "Axle visual is missing",
    )
    axle_radius = float(
        axle_collision.findtext(
            "./geometry/cylinder/radius"
        )
    )
    axle_length = float(
        axle_collision.findtext(
            "./geometry/cylinder/length"
        )
    )
    visual_radius = float(
        axle_visual.findtext(
            "./geometry/cylinder/radius"
        )
    )
    visual_length = float(
        axle_visual.findtext(
            "./geometry/cylinder/length"
        )
    )
    require(
        abs(axle_radius - 0.008) < 1e-9,
        "Axle collision radius must be 8 mm",
    )
    require(
        abs(axle_length - 2.480) < 1e-9,
        "Axle collision length must be 2.480 m",
    )
    require(
        abs(axle_radius - visual_radius) < 1e-12
        and abs(axle_length - visual_length) < 1e-12,
        "Axle collision must match the visible axle",
    )

    disc_collisions = [
        node
        for node in rotor.findall("./collision")
        if node.get("name", "").startswith(
            "disc_collision_"
        )
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
        for left, right in zip(
            disc_y,
            disc_y[1:],
        )
    )
    first_disc = disc_collisions[0]
    disc_radius = float(
        first_disc.findtext(
            "./geometry/cylinder/radius"
        )
    )
    disc_thickness = float(
        first_disc.findtext(
            "./geometry/cylinder/length"
        )
    )
    require(
        abs(disc_radius - 0.025) < 1e-9,
        "Disc radius must be 25 mm",
    )
    require(
        abs(disc_thickness - 0.030) < 1e-9,
        "Disc thickness must be 30 mm",
    )
    longitudinal_gap = (
        shaft_pitch - 2.0 * disc_radius
    )
    transverse_gap = (
        transverse_pitch - disc_thickness
    )
    require(
        abs(longitudinal_gap - 0.070) < 1e-6,
        "Longitudinal opening must be 70 mm",
    )
    require(
        abs(transverse_gap - 0.070) < 1e-6,
        "Transverse opening must be 70 mm",
    )

    joints = shaft_root.findall(
        ".//joint[@name='shaft_joint']"
        "[@type='revolute']"
    )
    require(
        len(joints) == 1,
        "Each shaft needs one revolute joint",
    )
    require(
        vector(joints[0].findtext("./axis/xyz"))
        == [0.0, 1.0, 0.0],
        "Shaft must rotate around Y",
    )
    controllers = shaft_root.findall(
        ".//plugin"
        "[@filename='gz-sim-joint-controller-system']"
    )
    require(
        len(controllers) == 1,
        "Each shaft needs one JointController",
    )

    last_shaft_right = (
        max(x_values) + disc_radius
    )
    accepted_pose = pose(accepted)
    accepted_size = box_size(
        accepted,
        "accepted_belt_collision",
    )
    accepted_left = (
        accepted_pose[0] - accepted_size[0] / 2.0
    )
    accepted_top = (
        accepted_pose[2] + accepted_size[2] / 2.0
    )
    roller_top = rotor_pose[2] + disc_radius
    upper_gap = accepted_left - last_shaft_right
    upper_drop = roller_top - accepted_top
    require(
        0.0005 <= upper_gap <= 0.0020,
        "Upper transfer gap must be about 1 mm",
    )
    require(
        0.003 <= upper_drop <= 0.006,
        "Upper output must be 3-6 mm lower",
    )

    reject_transfer_pose = pose(reject_transfer)
    reject_transfer_right = (
        reject_transfer_pose[0]
        + reject_transfer_size[0] / 2.0
    )
    reject_pose = pose(reject)
    reject_size = box_size(
        reject,
        "reject_belt_collision",
    )
    reject_left = (
        reject_pose[0] - reject_size[0] / 2.0
    )
    reject_transfer_top = (
        reject_transfer_pose[2]
        + reject_transfer_size[2] / 2.0
    )
    reject_top = (
        reject_pose[2] + reject_size[2] / 2.0
    )
    lower_gap = reject_left - reject_transfer_right
    lower_drop = reject_transfer_top - reject_top
    require(
        0.0005 <= lower_gap <= 0.0020,
        "Lower transfer gap must be about 1 mm",
    )
    require(
        0.003 <= lower_drop <= 0.006,
        "Lower output must be 3-6 mm lower",
    )

    for plugin in (
        "CameraTracking",
        "GzSceneManager",
    ):
        require(
            world_root.find(
                f".//gui/plugin[@filename='{plugin}']"
            )
            is not None,
            f"{plugin} GUI plugin is missing",
        )

    bridge_text = BRIDGE.read_text(
        encoding="utf-8"
    )
    for suffix in (
        "infeed",
        "screen",
        "accepted",
        "reject_transfer",
        "reject",
    ):
        require(
            f"/singulator/separator/{suffix}/cmd_vel"
            in bridge_text,
            f"Missing bridge topic for {suffix}",
        )

    launch_text = LAUNCH.read_text(
        encoding="utf-8"
    )
    for token in (
        'default_value="continuous"',
        'default_value="4.0"',
        'default_value="0.50"',
        'default_value="2.0"',
        '"remove_retries": 3',
        "separator_demo_controller",
        "separator_demo_spawner",
        "separator_demo_cleanup",
    ):
        require(
            token in launch_text,
            f"Launch token is missing: {token}",
        )

    controller_text = CONTROLLER.read_text(
        encoding="utf-8"
    )
    for token in (
        'self.declare_parameter("shaft_collision_radius_m", 0.025)',
        "angular_speed = surface_speed / radius",
        "rpm = angular_speed * 60.0 / (2.0 * math.pi)",
    ):
        require(
            token in controller_text,
            f"Roller-speed token is missing: {token}",
        )

    expected_angular_speed = 2.0 / disc_radius
    require(
        abs(expected_angular_speed - 80.0) < 1e-9,
        "25 mm radius must use 80 rad/s at 2 m/s",
    )

    spawner_text = SPAWNER.read_text(
        encoding="utf-8"
    )
    require(
        'self.declare_parameter("small_item_probability"'
        in spawner_text,
        "Spawner lower-class probability parameter is missing",
    )
    require(
        "minimum_projection < self.cutoff"
        in spawner_text,
        "Spawner does not use both base projections",
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

    cleanup_text = CLEANUP.read_text(
        encoding="utf-8"
    )
    for token in (
        r"(?:_[A-Za-z0-9_]+)?_spot\d+$",
        "remove_retries",
        "_monitor_loop",
        "monitor_restarts",
        "Separator statistics",
    ):
        require(
            token in cleanup_text,
            f"Cleanup robustness token missing: {token}",
        )

    test_names = (
        "box_separator_123_n0000001_exp_lower_"
        "long_narrow_spot04",
        "box_separator_123_n0000002_exp_upper_"
        "large_carton_spot07",
        "box_separator_123_n0000003_exp_lower_spot02",
    )
    pattern = re.compile(
        r"^box_separator_\d+_n\d+_exp_(upper|lower)"
        r"(?:_[A-Za-z0-9_]+)?_spot\d+$"
    )
    require(
        all(pattern.fullmatch(name) for name in test_names),
        "Cleanup name pattern does not accept parcel profiles",
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

    rpm = (
        expected_angular_speed
        * 60.0
        / (2.0 * 3.141592653589793)
    )
    print(
        "Separator demo static validation passed: "
        f"shafts={len(shaft_includes)}, "
        f"discs_per_shaft={len(disc_collisions)}, "
        f"axle_radius={axle_radius:.3f} m, "
        f"openings={longitudinal_gap:.3f}/"
        f"{transverse_gap:.3f} m, "
        f"upper_transfer={upper_gap:.3f} m/"
        f"{upper_drop:.3f} m, "
        f"lower_transfer={lower_gap:.3f} m/"
        f"{lower_drop:.3f} m, "
        f"omega_2mps={expected_angular_speed:.1f} rad/s, "
        f"rpm_2mps={rpm:.1f}, "
        "lower_probability=0.50"
    )


if __name__ == "__main__":
    main()
