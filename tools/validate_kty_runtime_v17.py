#!/usr/bin/env python3
"""Static acceptance for runtime-v17 empty-registry startup recovery."""

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
    runtime_path = SIM / "kty_station_sim" / "mechatronics_cycle_v17.py"
    py_compile.compile(str(runtime_path), doraise=True)
    runtime = read(runtime_path)
    setup = read(SIM / "setup.py")
    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")

    for fragment in (
        "class KtyMechatronicsCycleV17",
        "KtyMechatronicsCycleV16",
        "self._registry_sequence > 0",
        "sequence > baseline_sequence",
        "registry_empty_frame_is_valid",
        "KtyMechatronicsCycle._set_commands",
        '"runtime_profile": "kty_mechatronics_v17"',
    ):
        require(fragment in runtime, f"Missing runtime-v17 behavior: {fragment}")

    require(
        "bool(self._pose_cache)" not in runtime,
        "Runtime v17 must not reject fresh empty registry frames",
    )
    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v17:main" in setup,
        "Accepted executable is not routed to runtime v17",
    )
    require(
        "mechatronics_cycle_v17 = kty_station_sim.mechatronics_cycle_v17:main" in setup,
        "Explicit runtime v17 executable missing",
    )
    require(
        "validate_kty_runtime_v17.py" in build,
        "Build script does not validate runtime v17",
    )
    print("KTY runtime v17 fresh-empty registry validation: OK")


if __name__ == "__main__":
    main()
