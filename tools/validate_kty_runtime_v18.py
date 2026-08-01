#!/usr/bin/env python3
"""Static acceptance for runtime-v18 continuous KTY cycling and release defaults."""

from __future__ import annotations

from pathlib import Path
import py_compile
import sys
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src" / "kty_station_sim"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_generated_chute(source_world: Path) -> None:
    """Build the release world and inspect the physical funnel geometry."""
    sys.path.insert(0, str(SIM))
    try:
        from kty_station_sim.world_patch_v4 import build_runtime_v13_world
    finally:
        sys.path.pop(0)

    with tempfile.TemporaryDirectory(prefix="kty_v18_validate_") as temp_dir:
        destination = Path(temp_dir) / "kty_runtime_v18.sdf"
        build_runtime_v13_world(source_world, destination)
        root = ET.parse(destination).getroot()
        chute = root.find(
            "world/model[@name='kty_product_chute']/link[@name='chute']"
        )
        require(chute is not None, "Generated KTY chute is missing")

        expected = {
            "funnel_guide_neg_y": "0 -0.255 0.120 0 0 0.079830",
            "funnel_guide_pos_y": "0 0.255 0.120 0 0 -0.079830",
        }
        for name, expected_pose in expected.items():
            collision = chute.find(f"collision[@name='{name}_collision']")
            visual = chute.find(f"visual[@name='{name}_visual']")
            require(collision is not None, f"Generated chute misses {name} collision")
            require(visual is not None, f"Generated chute misses {name} visual")
            require(
                collision.findtext("pose") == expected_pose,
                f"Unexpected pose for {name}",
            )
            require(
                collision.findtext("geometry/box/size") == "1.010 0.030 0.240",
                f"Unexpected dimensions for {name}",
            )
            require(
                collision.findtext("surface/bounce/restitution_coefficient") == "0.0",
                f"{name} must use an inelastic contact",
            )


def main() -> None:
    runtime_path = SIM / "kty_station_sim" / "mechatronics_cycle_v18.py"
    world_path = SIM / "kty_station_sim" / "world_patch_v4.py"
    source_world_path = SIM / "worlds" / "kty_mechatronics.sdf"
    perception_launch_path = SIM / "launch" / "kty_perception_3d.launch.py"
    v15_launch_path = SIM / "launch" / "kty_mechatronics_v15.launch.py"
    v13_launch_path = SIM / "launch" / "kty_mechatronics_v13.launch.py"
    setup_path = SIM / "setup.py"
    build_path = ROOT / "scripts" / "build_kty_perception_3d.sh"
    readme_path = ROOT / "README.md"
    handoff_path = ROOT / "docs" / "KTY_RUNTIME_V18_HANDOFF.md"
    commands_path = ROOT / "docs" / "KTY_RUNTIME_COMMANDS.md"

    for path in (
        runtime_path,
        world_path,
        perception_launch_path,
        v15_launch_path,
        v13_launch_path,
    ):
        py_compile.compile(str(path), doraise=True)

    runtime = read(runtime_path)
    world = read(world_path)
    perception_launch = read(perception_launch_path)
    v15_launch = read(v15_launch_path)
    v13_launch = read(v13_launch_path)
    setup = read(setup_path)
    build = read(build_path)
    readme = read(readme_path)
    handoff = read(handoff_path)
    commands = read(commands_path)

    for fragment in (
        "class KtyMechatronicsCycleV18",
        "MEDIUM_PRODUCT_PROFILES",
        "ProductProfile(0.280, 0.190, 0.145",
        "position_recovery_after_stalls",
        "position_recovery_spawn_x_m",
        "_recover_empty_queue_kty",
        "physical_then_bounded_empty_kty_respawn",
        '"runtime_profile": "kty_mechatronics_v18"',
        '"product_size_max_m": [0.280, 0.190, 0.145]',
        '"product_spawn_center_half_width_m": 0.090',
        '"product_landing_half_width_m": 0.170',
        "_transverse_half_projection",
        "self.product_landing_half_width - projected_half_y",
        '"chute_guidance_policy": "tapered_side_guides_to_kty_opening"',
    ):
        require(fragment in runtime, f"Missing runtime-v18 behavior: {fragment}")

    require("ProductProfile(0.400" not in runtime, "Giant 400 mm profile returned")
    require("ProductProfile(0.360" not in runtime, "Giant 360 mm profile returned")
    require('"contact_tolerance", "0.300"' in world, "Transport Z envelope not widened")
    require('"min_x", "-0.800"' in world, "Active-zone overlap not widened")
    require('"max_x", "-0.100"' in world, "Infeed-zone overlap not widened")
    require("_append_chute_guide" in world, "Chute guide generator is missing")
    require("funnel_guide_neg_y" in world, "Negative-Y chute guide is missing")
    require("funnel_guide_pos_y" in world, "Positive-Y chute guide is missing")
    require(
        '"0 0 1.60 0 1.57079632679 1.57079632679"' in world,
        "RGB-D camera was not lowered to Z=1.60 m",
    )
    validate_generated_chute(source_world_path)

    for name, launch in (
        ("kty_perception_3d.launch.py", perception_launch),
        ("kty_mechatronics_v15.launch.py", v15_launch),
        ("kty_mechatronics_v13.launch.py", v13_launch),
    ):
        require(
            'DeclareLaunchArgument("fill_ratio_threshold", default_value="0.82")'
            in launch,
            f"{name} does not use the 82% fill default",
        )
        require(
            'default_value="0.340"' in launch,
            f"{name} does not use the 340 mm height default",
        )

    require(
        '"camera_to_bottom_m": 1.10' in v13_launch,
        "Fill estimator camera distance is not 1.10 m",
    )
    require(
        '"camera_to_kty_bottom_m": 1.10' in v13_launch,
        "3-D perception camera distance is not 1.10 m",
    )

    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v18:main" in setup,
        "Accepted alias is not routed to runtime v18",
    )
    require(
        "mechatronics_cycle_v18 = kty_station_sim.mechatronics_cycle_v18:main" in setup,
        "Explicit runtime-v18 executable is missing",
    )
    require("validate_kty_runtime_v18.py" in build, "Build script skips v18 validation")
    require("mechatronics_cycle_v18" in build, "Build script does not verify v18 executable")

    require("82%" in readme and "1,10 м" in readme, "README misses release fill settings")
    require(
        "docs/KTY_RUNTIME_V18_HANDOFF.md" in readme,
        "README does not link the v18 handoff",
    )
    require(
        "docs/KTY_RUNTIME_COMMANDS.md" in readme,
        "README does not link the command runbook",
    )
    require("82%" in handoff and "340 мм" in handoff, "Handoff misses fill limits")
    require("check_kty_runtime_v18.sh" in commands, "Command runbook misses continuity test")

    print(
        "KTY runtime v18 continuous-cycle, guided-chute and release-default "
        "validation: OK"
    )


if __name__ == "__main__":
    main()
