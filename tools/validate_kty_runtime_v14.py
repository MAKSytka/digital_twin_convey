#!/usr/bin/env python3
"""Static acceptance for the runtime-v14 startup recovery patch."""

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


def validate_world() -> None:
    sys.path.insert(0, str(SIM))
    from kty_station_sim.world_patch_v4 import build_runtime_v13_world

    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "runtime_v14.sdf"
        build_runtime_v13_world(SIM / "worlds" / "kty_mechatronics.sdf", output)
        root = ET.parse(output).getroot()

    world = root.find("world")
    require(world is not None, "Generated world missing")
    require(world.findtext("physics/max_step_size") == "0.005", "Expected 5 ms physics")
    require(world.findtext("physics/real_time_update_rate") == "200", "Expected 200 Hz physics")
    sensor = world.find(".//sensor[@name='overhead_rgbd']")
    require(sensor is not None, "RGB-D sensor missing")
    require(sensor.findtext("update_rate") == "5", "Expected 5 Hz RGB-D")
    require(sensor.findtext("camera/image/width") == "448", "Expected 448 px RGB-D")
    require(sensor.findtext("camera/image/height") == "336", "Expected 336 px RGB-D")

    scene = world.find("plugin[@name='gz::sim::systems::SceneBroadcaster']")
    require(scene is not None, "SceneBroadcaster required for dynamic_pose/info")
    require(
        world.find("plugin[@name='gz::sim::systems::PosePublisher']") is None,
        "Obsolete mismatched PosePublisher remains",
    )


def validate_python_and_launch() -> None:
    runtime13_path = SIM / "kty_station_sim" / "mechatronics_cycle_v13.py"
    runtime14_path = SIM / "kty_station_sim" / "mechatronics_cycle_v14.py"
    world_path = SIM / "kty_station_sim" / "world_patch_v4.py"
    launch13_path = SIM / "launch" / "kty_mechatronics_v13.launch.py"
    launch14_path = SIM / "launch" / "kty_mechatronics_v14.launch.py"
    for path in (runtime13_path, runtime14_path, world_path, launch13_path, launch14_path):
        py_compile.compile(str(path), doraise=True)

    runtime13 = read(runtime13_path)
    require("OZON_PRODUCT_PROFILES" in runtime13, "Ozon product profiles lost")
    require("_eject_and_prefeed" in runtime13, "Overlapped changeover lost")
    require('"runtime_profile": "kty_mechatronics_v13"' in runtime13, "V13 telemetry lost")

    runtime14 = read(runtime14_path)
    for fragment in (
        "class KtyMechatronicsCycleV14",
        "KtyMechatronicsCycle._wait_for_services(self, timeout_s)",
        "_wait_for_model_pose",
        "_spawn_kty",
        "_remove_static_model_best_effort",
        "service_ledger_not_pose_cache",
        '"runtime_profile": "kty_mechatronics_v14"',
        '"pose_feedback": "scene_broadcaster_dynamic_pose_info"',
        "Avoid leaving a gate-only world",
    ):
        require(fragment in runtime14, f"Missing runtime-v14 behavior: {fragment}")
    require('"gz", "topic"' not in runtime14, "Runtime v14 invokes gz topic")
    require("subprocess.run" not in runtime14, "Runtime v14 spawns CLI readers")

    launch14 = read(launch14_path)
    for fragment in (
        "kty_dynamic_model_pose_bridge",
        "/world/kty_mechatronics_surface/dynamic_pose/info",
        "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        '"/kty/mech/model_poses"',
        "kty_mechatronics_v13.launch.py",
    ):
        require(fragment in launch14, f"Missing runtime-v14 launch wiring: {fragment}")
    require(
        "/world/kty_mechatronics_surface/pose@" not in launch14,
        "V14 launch still bridges the wrong /pose topic",
    )


def validate_package_and_scripts() -> None:
    setup = read(SIM / "setup.py")
    run = read(ROOT / "scripts" / "run_kty_perception_3d.sh")
    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")
    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v14:main" in setup,
        "Accepted executable not routed to v14",
    )
    require(
        "mechatronics_cycle_v14 = kty_station_sim.mechatronics_cycle_v14:main" in setup,
        "Explicit v14 executable missing",
    )
    require("kty_mechatronics_v14.launch.py" in run, "Run script does not launch v14")
    require("validate_kty_runtime_v14.py" in build, "Build script does not validate v14")


def main() -> None:
    validate_world()
    validate_python_and_launch()
    validate_package_and_scripts()
    print("KTY runtime v14 startup recovery validation: OK")


if __name__ == "__main__":
    main()
