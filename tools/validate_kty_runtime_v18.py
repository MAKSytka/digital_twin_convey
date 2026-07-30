#!/usr/bin/env python3
"""Static acceptance for runtime-v18 continuous KTY cycling and release defaults."""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src" / "kty_station_sim"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def main() -> None:
    runtime_path = SIM / "kty_station_sim" / "mechatronics_cycle_v18.py"
    world_path = SIM / "kty_station_sim" / "world_patch_v4.py"
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
    ):
        require(fragment in runtime, f"Missing runtime-v18 behavior: {fragment}")

    require("ProductProfile(0.400" not in runtime, "Giant 400 mm profile returned")
    require("ProductProfile(0.360" not in runtime, "Giant 360 mm profile returned")
    require('"contact_tolerance", "0.300"' in world, "Transport Z envelope not widened")
    require('"min_x", "-0.800"' in world, "Active-zone overlap not widened")
    require('"max_x", "-0.100"' in world, "Infeed-zone overlap not widened")
    require(
        '"0 0 1.60 0 1.57079632679 1.57079632679"' in world,
        "RGB-D camera was not lowered to Z=1.60 m",
    )

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

    print("KTY runtime v18 continuous-cycle and release-default validation: OK")


if __name__ == "__main__":
    main()
