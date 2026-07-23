#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = (
    ROOT
    / "src"
    / "singulator_description"
    / "models"
)
MODEL = MODEL_DIR / "infeed_size_separator" / "model.sdf"
SHAFT_EVEN = MODEL_DIR / "separator_star_shaft" / "model.sdf"
SHAFT_ODD = MODEL_DIR / "separator_star_shaft_odd" / "model.sdf"
DISC = MODEL_DIR / "separator_star_disc" / "model.sdf"
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
    size = link.findtext(
        f"./collision[@name='{collision_name}']/geometry/box/size"
    )
    return vector(size)


def disc_positions(shaft_root: ET.Element) -> list[float]:
    includes = [
        node
        for node in shaft_root.findall(".//include")
        if node.findtext("uri") == "model://separator_star_disc"
    ]
    return sorted(
        vector(node.findtext("pose"))[1]
        for node in includes
    )


def main() -> None:
    model_root = ET.parse(MODEL).getroot()
    even_root = ET.parse(SHAFT_EVEN).getroot()
    odd_root = ET.parse(SHAFT_ODD).getroot()
    disc_root = ET.parse(DISC).getroot()
    world_root = ET.parse(WORLD).getroot()

    require(
        world_root.find(
            ".//world[@name='infeed_size_separator_demo']"
        )
        is not None,
        "Demo world name is missing",
    )

    main_include = [
        node
        for node in world_root.findall(".//include")
        if node.findtext("uri") == "model://infeed_size_separator"
    ]
    require(len(main_include) == 1, "Separator body include is missing")

    shaft_includes = [
        node
        for node in world_root.findall(".//include")
        if node.findtext("uri")
        in {
            "model://separator_star_shaft",
            "model://separator_star_shaft_odd",
        }
    ]
    require(len(shaft_includes) == 11, "Expected 11 physical shafts")

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

    entry = belt_size(
        model_root,
        "entry_belt",
        "entry_belt_collision",
    )
    accepted = belt_size(
        model_root,
        "accepted_belt",
        "accepted_belt_collision",
    )
    reject = belt_size(
        model_root,
        "reject_belt",
        "reject_belt_collision",
    )
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
        require(
            abs(size[0] - 3.0) < 1e-9,
            f"{name} output length must be 3 m",
        )
        require(
            abs(size[1] - 2.5) < 1e-9,
            f"{name} width must be 2.5 m",
        )
    require(
        abs(reject_transfer[1] - 2.5) < 1e-9,
        "Lower catch belt must cover the full 2.5 m width",
    )

    even_y = disc_positions(even_root)
    odd_y = disc_positions(odd_root)
    require(len(even_y) == 25, "Even shaft must contain 25 discs")
    require(len(odd_y) == 24, "Odd shaft must contain 24 discs")

    disc_collision = disc_root.find(
        ".//collision[@name='disc_collision']"
    )
    require(disc_collision is not None, "Disc collision is missing")
    radius_text = disc_collision.findtext("./geometry/cylinder/radius")
    thickness_text = disc_collision.findtext("./geometry/cylinder/length")
    require(radius_text is not None, "Disc radius is missing")
    require(thickness_text is not None, "Disc thickness is missing")
    disc_radius = float(radius_text)
    disc_thickness = float(thickness_text)
    require(
        abs(disc_radius - 0.025) < 1e-9,
        "Disc collision radius must be 25 mm",
    )
    require(
        abs(disc_thickness - 0.030) < 1e-9,
        "Disc collision thickness must be 30 mm",
    )

    transverse_pitch = min(
        right - left
        for left, right in zip(even_y, even_y[1:])
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

    joint = disc_root.find(
        ".//joint[@name='shaft_joint'][@type='revolute']"
    )
    require(joint is not None, "Disc is not revolute")
    require(
        vector(joint.findtext("./axis/xyz")) == [0.0, 1.0, 0.0],
        "Disc must rotate around Y",
    )
    plugin = disc_root.find(
        ".//plugin[@filename='gz-sim-joint-controller-system']"
    )
    require(plugin is not None, "Disc JointController is missing")
    require(
        plugin.findtext("topic")
        == "/singulator/separator/screen/cmd_vel",
        "Disc command topic is incorrect",
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
            f"/singulator/separator/{suffix}/cmd_vel"
            in bridge_text,
            f"Missing bridge topic for {suffix}",
        )

    launch_text = LAUNCH.read_text(encoding="utf-8")
    for token in (
        'default_value="continuous"',
        'default_value="4.0"',
        'default_value="0.20"',
        'default_value="2.0"',
        "separator_demo_controller",
        "separator_demo_spawner",
        "separator_demo_cleanup",
    ):
        require(token in launch_text, f"Launch token is missing: {token}")

    spawner_text = SPAWNER.read_text(encoding="utf-8")
    require(
        "range(10)" in spawner_text,
        "Spawner does not define ten fixed spots",
    )
    require(
        "min(projection_x, projection_y) < self.cutoff"
        in spawner_text,
        "Spawner does not use the two-projection cutoff rule",
    )
    require(
        'self.declare_parameter("spawn_mode", "continuous")'
        in spawner_text,
        "Continuous mode is not the spawner default",
    )
    require(
        'self.declare_parameter("small_item_probability", 0.20)'
        in spawner_text,
        "Small-item default probability is not 20%",
    )
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
        require(token in cleanup_text, f"Cleanup/statistics token missing: {token}")

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

    print(
        "Separator demo static validation passed: "
        f"width={entry[1]:.3f} m, "
        f"output_length={accepted[0]:.3f} m, "
        f"shafts={len(shaft_includes)}, "
        f"discs={len(even_y)}/{len(odd_y)}, "
        f"openings={longitudinal_gap:.3f}/{transverse_gap:.3f} m"
    )


if __name__ == "__main__":
    main()
