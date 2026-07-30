#!/usr/bin/env python3
"""Static acceptance for runtime-v18 continuous KTY cycling."""

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
    setup_path = SIM / "setup.py"
    build_path = ROOT / "scripts" / "build_kty_perception_3d.sh"

    py_compile.compile(str(runtime_path), doraise=True)
    py_compile.compile(str(world_path), doraise=True)
    runtime = read(runtime_path)
    world = read(world_path)
    setup = read(setup_path)
    build = read(build_path)

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
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v18:main" in setup,
        "Accepted alias is not routed to runtime v18",
    )
    require(
        "mechatronics_cycle_v18 = kty_station_sim.mechatronics_cycle_v18:main" in setup,
        "Explicit runtime-v18 executable is missing",
    )
    require("validate_kty_runtime_v18.py" in build, "Build script skips v18 validation")
    require("mechatronics_cycle_v18" in build, "Build script does not verify v18 executable")

    print("KTY runtime v18 medium-product continuous-cycle validation: OK")


if __name__ == "__main__":
    main()
