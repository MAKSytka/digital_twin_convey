#!/usr/bin/env python3
"""Static acceptance for runtime v13."""

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
        output = Path(directory) / "runtime_v13.sdf"
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

    pose = world.find("plugin[@name='gz::sim::systems::PosePublisher']")
    require(pose is not None, "PosePublisher system missing")
    require(pose.attrib.get("filename") == "gz-sim-pose-publisher-system", "Wrong PosePublisher filename")
    require(pose.findtext("publish_model_pose") == "true", "Model poses not enabled")
    require(pose.findtext("use_pose_vector_msg") == "true", "Pose_V not enabled")
    require(pose.findtext("update_frequency") == "20", "Pose stream must be 20 Hz")
    require(pose.findtext("static_publisher") == "false", "Static models must share model pose stream")


def validate_python_and_launch() -> None:
    runtime_path = SIM / "kty_station_sim" / "mechatronics_cycle_v13.py"
    world_path = SIM / "kty_station_sim" / "world_patch_v4.py"
    launch_path = SIM / "launch" / "kty_mechatronics_v13.launch.py"
    for path in (runtime_path, world_path, launch_path):
        py_compile.compile(str(path), doraise=True)

    runtime = read(runtime_path)
    for fragment in (
        "class KtyMechatronicsCycleV13",
        "TFMessage",
        'POSE_TOPIC = "/kty/mech/model_poses"',
        "persistent_ros_gz_pose_bridge",
        "OZON_PRODUCT_PROFILES",
        "ProductProfile(0.035, 0.015, 0.010",
        "ProductProfile(0.400, 0.320, 0.280",
        "closed_gate_spawn_interval_s",
        "closed_gate_max_products",
        "_eject_and_prefeed",
        "prefeed_target_x_m",
        '"runtime_profile": "kty_mechatronics_v13"',
    ):
        require(fragment in runtime, f"Missing runtime-v13 behavior: {fragment}")
    require('subprocess.run' not in runtime, "Runtime v13 must not spawn transport CLI processes")
    require('"gz", "topic"' not in runtime, "Runtime v13 still invokes gz topic")
    require(runtime.index("_eject_and_prefeed(old_kty, next_kty)") < runtime.index("_despawn_loaded_kty(old_kty, old_products)"), "Queued KTY must prefeed while old KTY exits")

    launch = read(launch_path)
    for fragment in (
        "build_runtime_v13_world",
        "kty_model_pose_bridge",
        "/world/kty_mechatronics_surface/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        '("/world/kty_mechatronics_surface/pose", "/kty/mech/model_poses")',
        'default_value="1.90"',
        '"roller_linear_speed_mps": 0.80',
        '"closed_gate_spawn_interval_s": 3.0',
        '"closed_gate_max_products": 5',
        '"prefeed_target_x_m": -0.50',
        '"processing_hz": 2.5',
        '"refresh_hz": 3.0',
    ):
        require(fragment in launch, f"Missing runtime-v13 launch wiring: {fragment}")


def validate_package_and_scripts() -> None:
    setup = read(SIM / "setup.py")
    package_xml = read(SIM / "package.xml")
    run = read(ROOT / "scripts" / "run_kty_perception_3d.sh")
    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")
    require("mechatronics_cycle_v13 = kty_station_sim.mechatronics_cycle_v13:main" in setup, "Missing v13 executable")
    require("mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v13:main" in setup, "Accepted executable not routed to v13")
    require("<exec_depend>tf2_msgs</exec_depend>" in package_xml, "tf2_msgs dependency missing")
    require("kty_mechatronics_v13.launch.py" in run, "Run script does not launch v13")
    require("validate_kty_runtime_v13.py" in build, "Build script does not validate v13")


def main() -> None:
    validate_world()
    validate_python_and_launch()
    validate_package_and_scripts()
    print("KTY runtime v13 persistent-pose validation: OK")


if __name__ == "__main__":
    main()
