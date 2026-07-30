#!/usr/bin/env python3
"""Static acceptance for runtime v15 JSON model-pose feedback."""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src" / "kty_station_sim"
PLUGIN = ROOT / "src" / "kty_conveyor_surface" / "src" / "KtyConveyorSurfaceSystem.cc"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_plugin() -> None:
    source = read(PLUGIN)
    for fragment in (
        "gz/msgs/stringmsg.pb.h",
        'poseRegistryTopic{"/kty/mech/model_pose_registry_json"}',
        'poseRegistryPrefix{"kty_mech_"}',
        "kty_model_pose_registry/v1",
        "registryPublisher.Publish(message)",
        "std::chrono::duration<double>(_info.simTime).count()",
    ):
        require(fragment in source, f"Missing plugin pose-registry behavior: {fragment}")


def validate_runtime_and_launch() -> None:
    runtime_path = SIM / "kty_station_sim" / "mechatronics_cycle_v15.py"
    launch_path = SIM / "launch" / "kty_mechatronics_v15.launch.py"
    for path in (runtime_path, launch_path):
        py_compile.compile(str(path), doraise=True)

    runtime = read(runtime_path)
    for fragment in (
        "class KtyMechatronicsCycleV15",
        "KtyMechatronicsCycleV14",
        "std_msgs.msg import String",
        'REGISTRY_TOPIC = "/kty/mech/model_pose_registry_json"',
        'payload.get("schema") != "kty_model_pose_registry/v1"',
        "self._pose_cache = poses",
        '"runtime_profile": "kty_mechatronics_v15"',
        '"pose_feedback": "gazebo_plugin_json_registry"',
    ):
        require(fragment in runtime, f"Missing runtime-v15 behavior: {fragment}")
    require("subprocess.run" not in runtime, "Runtime v15 must not spawn CLI readers")
    require('"gz", "topic"' not in runtime, "Runtime v15 still invokes gz topic")

    launch = read(launch_path)
    for fragment in (
        "kty_model_pose_registry_bridge",
        "/kty/mech/model_pose_registry_json",
        "@std_msgs/msg/String[gz.msgs.StringMsg",
        "kty_mechatronics_v13.launch.py",
    ):
        require(fragment in launch, f"Missing runtime-v15 launch wiring: {fragment}")


def validate_package_and_scripts() -> None:
    setup = read(SIM / "setup.py")
    run = read(ROOT / "scripts" / "run_kty_perception_3d.sh")
    build = read(ROOT / "scripts" / "build_kty_perception_3d.sh")
    require(
        "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v15:main" in setup,
        "Accepted entry point does not route to v15",
    )
    require(
        "mechatronics_cycle_v15 = kty_station_sim.mechatronics_cycle_v15:main" in setup,
        "Explicit v15 entry point missing",
    )
    require("kty_mechatronics_v15.launch.py" in run, "Run script does not launch v15")
    require("validate_kty_runtime_v15.py" in build, "Build script does not validate v15")


def main() -> None:
    validate_plugin()
    validate_runtime_and_launch()
    validate_package_and_scripts()
    print("KTY runtime v15 Gazebo JSON pose-registry validation: OK")


if __name__ == "__main__":
    main()
