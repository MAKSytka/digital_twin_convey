#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = (
    ROOT
    / "src"
    / "singulator_description"
    / "models"
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


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    model_root = ET.parse(MODEL).getroot()
    shaft_root = ET.parse(SHAFT).getroot()
    world_root = ET.parse(WORLD).getroot()

    require(
        world_root.find(".//world[@name='infeed_size_separator_demo']")
        is not None,
        "Demo world name is missing",
    )
    require(
        world_root.find(".//include/uri").text
        == "model://infeed_size_separator",
        "Demo world does not include the separator model",
    )

    shaft_includes = [
        node
        for node in model_root.findall(".//include")
        if node.findtext("uri") == "model://separator_star_shaft"
    ]
    require(len(shaft_includes) == 11, "Expected 11 star shafts")

    x_values = sorted(
        float(node.findtext("pose").split()[0])
        for node in shaft_includes
    )
    pitch = min(
        right - left
        for left, right in zip(x_values, x_values[1:])
    )

    support_size = shaft_root.find(
        ".//collision[@name='support_collision']/geometry/box/size"
    )
    require(support_size is not None, "Shaft support collision is missing")
    support_width = float(support_size.text.split()[0])
    clear_gap = pitch - support_width
    require(0.085 <= clear_gap <= 0.091, "Unexpected clear gap")

    entry_link = model_root.find(".//link[@name='entry_belt']")
    accepted_link = model_root.find(".//link[@name='accepted_belt']")
    require(entry_link is not None, "Entry belt is missing")
    require(accepted_link is not None, "Accepted belt is missing")

    entry_center = float(entry_link.findtext("pose").split()[0])
    entry_length = float(
        entry_link.findtext(
            "./collision[@name='entry_belt_collision']/geometry/box/size"
        ).split()[0]
    )
    accepted_center = float(accepted_link.findtext("pose").split()[0])
    accepted_length = float(
        accepted_link.findtext(
            "./collision[@name='accepted_belt_collision']/geometry/box/size"
        ).split()[0]
    )

    first_shaft_left = x_values[0] - support_width / 2.0
    last_shaft_right = x_values[-1] + support_width / 2.0
    entry_gap = first_shaft_left - (entry_center + entry_length / 2.0)
    exit_gap = (accepted_center - accepted_length / 2.0) - last_shaft_right
    require(0.0 <= entry_gap <= 0.015, "Entry transfer gap is unsafe")
    require(0.0 <= exit_gap <= 0.015, "Accepted transfer gap is unsafe")

    star_visuals = shaft_root.findall(".//visual")
    require(len(star_visuals) >= 3, "Shaft does not look toothed")

    bridge_text = BRIDGE.read_text(encoding="utf-8")
    for suffix in ("infeed", "screen", "accepted", "reject"):
        require(
            f"/singulator/separator/{suffix}/cmd_vel" in bridge_text,
            f"Missing bridge topic for {suffix}",
        )

    launch_text = LAUNCH.read_text(encoding="utf-8")
    require(
        "separator_demo_controller" in launch_text,
        "Controller is missing from launch",
    )
    require(
        "separator_demo_spawner" in launch_text,
        "Spawner is missing from launch",
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

    print(
        "Separator demo static validation passed: "
        f"shafts={len(shaft_includes)}, "
        f"pitch={pitch:.3f} m, clear_gap={clear_gap:.3f} m, "
        f"transfer_gaps={entry_gap:.3f}/{exit_gap:.3f} m"
    )


if __name__ == "__main__":
    main()
