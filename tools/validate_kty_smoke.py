#!/usr/bin/env python3
"""Static checks for the isolated KTY smoke scenario.

The validator intentionally avoids importing ROS or Gazebo. Runtime behavior is
verified on the target workstation by scripts/check_kty_smoke.sh.
"""

from __future__ import annotations

from pathlib import Path
import py_compile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "kty_station_sim"
WORLD = PACKAGE / "worlds" / "kty_station_smoke.sdf"
LAUNCH = PACKAGE / "launch" / "kty_smoke.launch.py"
HEARTBEAT = PACKAGE / "kty_station_sim" / "smoke_heartbeat.py"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_world() -> None:
    root = ET.parse(WORLD).getroot()
    require(root.tag == "sdf", "Smoke world root must be <sdf>")

    world = root.find("world")
    require(world is not None, "Smoke world is missing <world>")
    require(
        world.attrib.get("name") == "kty_station_smoke",
        "Unexpected smoke world name",
    )

    models = {model.attrib.get("name") for model in world.findall("model")}
    required_models = {
        "kty_infeed_smoke",
        "kty_platform_smoke",
        "kty_outfeed_smoke",
        "product_chute_smoke",
        "kty_smoke_container",
    }
    require(required_models <= models, f"Missing smoke models: {required_models - models}")

    kty = world.find("model[@name='kty_smoke_container']")
    require(kty is not None, "Static KTY model is missing")
    require(kty.findtext("static") == "true", "Smoke KTY must be static in stage 1")
    collisions = kty.findall(".//collision")
    require(len(collisions) == 5, "Smoke KTY must have bottom and four wall collisions")

    text = read(WORLD)
    for fragment in (
        '<plugin filename="MinimalScene" name="3D View">',
        '<plugin filename="InteractiveViewControl" name="Interactive view control">',
        '<plugin filename="CameraTracking" name="Camera Tracking">',
        '<plugin name="World control" filename="WorldControl">',
        "<start_paused>0</start_paused>",
        "<play_pause>1</play_pause>",
        "<step>1</step>",
        "<engine>ogre</engine>",
    ):
        require(fragment in text, f"Missing GUI/world fragment: {fragment}")

    require("<max_step_size>0.001</max_step_size>" in text, "Expected 1 ms smoke step")
    require("<real_time_factor>1.0</real_time_factor>" in text, "Expected real-time smoke world")


def validate_python_and_wiring() -> None:
    py_compile.compile(str(LAUNCH), doraise=True)
    py_compile.compile(str(HEARTBEAT), doraise=True)

    launch = read(LAUNCH)
    for fragment in (
        'worlds" / "kty_station_smoke.sdf"',
        '"gz_args": f"-r -v 3 {world}"',
        'executable="smoke_heartbeat"',
        '"use_sim_time": False',
    ):
        require(fragment in launch, f"Missing smoke launch wiring: {fragment}")

    heartbeat = read(HEARTBEAT)
    for fragment in (
        'super().__init__("kty_smoke_heartbeat")',
        '"/kty/smoke/heartbeat"',
        "time.monotonic()",
        '"expected_model": "kty_smoke_container"',
    ):
        require(fragment in heartbeat, f"Missing heartbeat behavior: {fragment}")

    setup = read(PACKAGE / "setup.py")
    require(
        "smoke_heartbeat = kty_station_sim.smoke_heartbeat:main" in setup,
        "Smoke heartbeat console entry point is missing",
    )


def validate_scripts() -> None:
    required_scripts = (
        "scripts/build_kty_smoke.sh",
        "scripts/run_kty_smoke.sh",
        "scripts/check_kty_smoke.sh",
        "scripts/stop_kty_smoke.sh",
    )
    for relative in required_scripts:
        path = ROOT / relative
        text = read(path)
        require(text.startswith("#!/usr/bin/env bash"), f"Missing shebang: {relative}")

    diagnostic = read(ROOT / "scripts/check_kty_smoke.sh")
    for fragment in (
        "/world/kty_station_smoke/control",
        "/world/kty_station_smoke/create",
        "/world/kty_station_smoke/remove",
        "/world/kty_station_smoke/set_pose",
        "/gui/camera/view_control",
        "kty_smoke_container",
        "/kty/smoke/heartbeat",
        "check_gz_model()",
        "sed -E",
        "/world/kty_station_smoke/pose/info",
        "Raw output of 'gz model --list'",
    ):
        require(fragment in diagnostic, f"Smoke diagnostic is missing: {fragment}")

    require(
        "grep -Fxq 'kty_smoke_container'" not in diagnostic,
        "Smoke diagnostic must not depend on undecorated exact CLI output",
    )


def main() -> None:
    validate_world()
    validate_python_and_wiring()
    validate_scripts()
    print("KTY smoke static validation: OK")


if __name__ == "__main__":
    main()
