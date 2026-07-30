#!/usr/bin/env python3
"""Static acceptance for runtime-v16 lifecycle and long-run feeder recovery."""

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


def validate_runtime() -> None:
    path = SIM / "kty_station_sim" / "mechatronics_cycle_v16.py"
    py_compile.compile(str(path), doraise=True)
    source = read(path)
    for fragment in (
        "class KtyMechatronicsCycleV16",
        "KtyMechatronicsCycleV15",
        "gazebo_json_registry_confirmed",
        "Slide gate OPEN and registry-confirmed",
        "Slide gate CLOSED and registry-confirmed",
        "product_spawn_max_attempts",
        "maximum_products_per_load",
        "Feeder back-pressure",
        "Gate appeared in LOAD",
        "product_count_safety_cap",
        '"runtime_profile": "kty_mechatronics_v16"',
        '"gate_registry_present"',
        '"product_spawn_failures"',
        '"registry_live_products"',
    ):
        require(fragment in source, f"Missing runtime-v16 behavior: {fragment}")
    require('"gz", "topic"' not in source, "Runtime v16 must not invoke gz topic")


def validate_wiring() -> None:
    setup = read(SIM / "setup.py")
    launch_path = SIM / "launch" / "kty_mechatronics_v16.launch.py"
    py_compile.compile(str(launch_path), doraise=True)
    launch = read(launch_path)
    run = read(ROOT / "scripts" / "run_kty_perception_3d.sh")
    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")

    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v16:main" in setup,
        "Accepted entry point does not route to runtime v16",
    )
    require(
        "mechatronics_cycle_v16 = kty_station_sim.mechatronics_cycle_v16:main" in setup,
        "Explicit runtime v16 entry point missing",
    )
    require("kty_mechatronics_v15.launch.py" in launch, "V16 launch must retain V15 registry bridge")
    require("kty_mechatronics_v16.launch.py" in run, "Run script does not launch v16")
    require("validate_kty_runtime_v16.py" in build, "Build script does not validate v16")


def validate_clean_shutdown() -> None:
    for relative in (
        "kty_station_sim/contour_recorder_3d.py",
        "kty_station_sim/vision_dashboard_3d.py",
    ):
        source = read(SIM / relative)
        require("rclpy.try_shutdown()" in source, f"{relative} still double-shuts down rclpy")
        require("rclpy.shutdown()" not in source, f"{relative} still calls rclpy.shutdown()")


def main() -> None:
    validate_runtime()
    validate_wiring()
    validate_clean_shutdown()
    print("KTY runtime v16 registry lifecycle and feeder recovery validation: OK")


if __name__ == "__main__":
    main()
