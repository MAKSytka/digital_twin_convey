#!/usr/bin/env python3
"""Validate the jury-facing release contract without running ROS or Gazebo."""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]

README = ROOT / "README.md"
ARCHITECTURE = ROOT / "docs/ARCHITECTURE.md"
INTERFACES = ROOT / "docs/INTERFACES.md"
PARAMETERS = ROOT / "docs/SIMULATION_PARAMETERS.md"
DEMO_SCENARIOS = ROOT / "docs/DEMO_SCENARIOS.md"
CLEANUP = ROOT / "docs/REPOSITORY_CLEANUP.md"
ROLLER_LAUNCH = ROOT / "src/singulator_bringup/launch/matrix_stream_roller.launch.py"
ROLLER_RUNNER = ROOT / "scripts/run_roller_demo.sh"
GITIGNORE = ROOT / ".gitignore"

REQUIRED_FILES = (
    README,
    ARCHITECTURE,
    INTERFACES,
    PARAMETERS,
    DEMO_SCENARIOS,
    CLEANUP,
    ROLLER_LAUNCH,
    ROLLER_RUNNER,
    GITIGNORE,
)


def fail(message: str) -> None:
    raise AssertionError(message)


def read(path: Path) -> str:
    if not path.is_file():
        fail(f"Required release file is missing: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_release_files() -> None:
    for path in REQUIRED_FILES:
        read(path)


def validate_matrix_contract() -> None:
    launch = read(ROLLER_LAUNCH)
    runner = read(ROLLER_RUNNER)
    docs = "\n".join(
        read(path) for path in (README, ARCHITECTURE, INTERFACES, PARAMETERS)
    )

    required_launch_fragments = (
        '"rows": 18',
        '"cols": 4',
        '"cell_length_m": 0.360',
        '"cell_width_m": 0.175',
        '"gap_x_m": 0.020',
        '"gap_y_m": 0.020',
        '"maximum_acceleration_mps2": 6.0',
        '"publish_rate_hz": 30.0',
        'SetLaunchConfiguration("matrix_rows", "18")',
        'bridge_rows_14_17.yaml',
    )
    for fragment in required_launch_fragments:
        if fragment not in launch:
            fail(f"Roller launch misses release fragment: {fragment}")

    if 'seed:="${SEED:-42}"' not in runner:
        fail("run_roller_demo.sh must default to deterministic seed 42")
    if 'target_rate_boxes_per_sec:="${TARGET_RATE_BOXES_PER_SEC:-4.0}"' not in runner:
        fail("run_roller_demo.sh must expose the nominal 4 items/s rate")

    required_doc_fragments = (
        "18×4",
        "72",
        "mu2",
        "0,2",
        "6,0 м/с²",
    )
    for fragment in required_doc_fragments:
        if fragment not in docs:
            fail(f"Jury documentation misses matrix contract: {fragment}")

    forbidden_doc_patterns = (
        r"mu2[^\n]{0,40}0[.,]8",
        r"rows\s*==\s*14",
        r"len\(target_speed_mps\)\s*==\s*56",
        r"56 ROS-топик",
        r"56 Gazebo",
    )
    for pattern in forbidden_doc_patterns:
        if re.search(pattern, docs, flags=re.IGNORECASE):
            fail(f"Outdated jury-facing matrix statement matched: {pattern}")


def validate_kty_release_contract() -> None:
    readme = read(README)
    architecture = read(ARCHITECTURE)
    interfaces = read(INTERFACES)
    combined = "\n".join((readme, architecture, interfaces))

    required = (
        "runtime v18",
        "LOAD",
        "CLOSE_GATE",
        "COMPACT",
        "EJECT_ACTIVE",
        "DESPAWN_ACTIVE",
        "POSITION_NEXT",
        "VERIFY_READY",
        "OPEN_GATE",
        "/kty/vision/image",
        "/kty/vision/depth_image",
        "/kty/perception/contours",
        "/kty/fill/state",
        "/kty/flow/state",
    )
    for fragment in required:
        if fragment not in combined:
            fail(f"KTY release documentation misses: {fragment}")

    obsolete_branch_commands = re.findall(
        r"git\s+switch[^\n]*(?:feat/kty|fix/kty|archive/kty)",
        combined,
        flags=re.IGNORECASE,
    )
    if obsolete_branch_commands:
        fail(f"Jury docs still require historical KTY branches: {obsolete_branch_commands}")


def validate_jury_entrypoint() -> None:
    readme = read(README)
    required_commands = (
        "bash ./scripts/run_roller_demo.sh",
        "infeed_size_separator_demo.launch.py",
        "bash ./scripts/run_kty_perception_3d.sh",
        "python3 tools/validate_project.py",
        "python3 tools/validate_release.py",
    )
    for command in required_commands:
        if command not in readme:
            fail(f"README jury entrypoint misses command: {command}")

    required_links = (
        "docs/ARCHITECTURE.md",
        "docs/INTERFACES.md",
        "docs/SIMULATION_PARAMETERS.md",
        "docs/DEMO_SCENARIOS.md",
        "docs/SINGULATION_CONTROL.md",
        "docs/TROUBLESHOOTING.md",
    )
    for link in required_links:
        if link not in readme:
            fail(f"README misses navigation link: {link}")


def validate_repository_hygiene() -> None:
    gitignore = read(GITIGNORE)
    required_patterns = (
        "build/",
        "install/",
        "log/",
        "__pycache__/",
        "*.before_*",
        "src_before_*/",
        "*.mp4",
        "*.step",
    )
    for pattern in required_patterns:
        if pattern not in gitignore:
            fail(f".gitignore misses required release pattern: {pattern}")

    forbidden_paths = []
    for pattern in (
        "**/*.before_*",
        "**/*.backup",
        "**/*.bak",
        "**/__pycache__",
        "src_before_*",
        "scripts_before_*",
    ):
        forbidden_paths.extend(ROOT.glob(pattern))
    if forbidden_paths:
        relative = sorted(str(path.relative_to(ROOT)) for path in forbidden_paths)
        fail(f"Backup/generated files are tracked in the release tree: {relative}")


def main() -> int:
    checks = (
        ("release files", validate_release_files),
        ("18x4 matrix contract", validate_matrix_contract),
        ("KTY v18 contract", validate_kty_release_contract),
        ("jury README entrypoint", validate_jury_entrypoint),
        ("repository hygiene", validate_repository_hygiene),
    )
    for label, check in checks:
        check()
        print(f"[OK] {label}")
    print("Release consistency validation passed.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        raise
