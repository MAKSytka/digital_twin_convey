#!/usr/bin/env python3
"""Static validation for classical 3-D KTY perception."""
from pathlib import Path
import py_compile

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "kty_station_sim"
IFACE = ROOT / "src" / "singulator_interfaces"

def need(ok, message):
    if not ok:
        raise AssertionError(message)

def read(path):
    need(path.is_file(), f"Missing {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")

def main():
    paths = [
        PKG / "kty_station_sim" / "classical_3d_core.py",
        PKG / "kty_station_sim" / "depth_perception_3d.py",
        PKG / "kty_station_sim" / "contour_recorder_3d.py",
        PKG / "kty_station_sim" / "vision_dashboard_3d.py",
        PKG / "launch" / "kty_mechatronics.launch.py",
    ]
    for path in paths:
        py_compile.compile(str(path), doraise=True)

    grasp = read(IFACE / "msg" / "KtyGraspCandidate.msg")
    contour = read(IFACE / "msg" / "KtyProductContour.msg")
    cmake = read(IFACE / "CMakeLists.txt")
    for fragment in ("geometry_msgs/Pose pose", "geometry_msgs/Vector3 approach_vector", "float32 score", "string strategy"):
        need(fragment in grasp, f"Missing grasp field {fragment}")
    for fragment in ("STATE_VISIBLE", "STATE_OCCLUDED", "tracking_state", "oriented_rectangle", "surface_normal", "estimated_size", "yaw_rad", "occlusion_score", "top_accessible", "KtyGraspCandidate[] grasp_candidates"):
        need(fragment in contour, f"Missing 3-D contour field {fragment}")
    need('"msg/KtyGraspCandidate.msg"' in cmake, "Grasp message not registered")

    core = read(paths[0])
    wrapper = read(paths[1])
    for fragment in ("depth_edge", "normal_edge", "cv2.watershed", "seed_height_prominence", "class Tracker", 'state="OCCLUDED"', "cv2.minAreaRect"):
        need(fragment in core, f"Missing classical 3-D behavior {fragment}")
    for fragment in ("KtyClassical3DPerception", "grasp_candidates", "oriented_rectangle", "surface_normal", "estimated_size", "top_accessible", "VISIBLE", "COLORMAP_MAGMA"):
        need(fragment in wrapper, f"Missing ROS 3-D output {fragment}")

    setup = read(PKG / "setup.py")
    launch = read(PKG / "launch" / "kty_mechatronics.launch.py")
    for fragment in ("depth_perception_3d =", "contour_recorder_3d =", "vision_dashboard_3d ="):
        need(fragment in setup, f"Missing entry point {fragment}")
    for fragment in ('executable="depth_perception_3d"', 'executable="contour_recorder_3d"', 'executable="vision_dashboard_3d"'):
        need(fragment in launch, f"Missing launch executable {fragment}")

    recorder = read(paths[2])
    dashboard = read(paths[3])
    for fragment in ("kty_carton_instances_3d/v2", "tracking_state", "oriented_rectangle_m", "grasp_candidates"):
        need(fragment in recorder, f"Missing recorder field {fragment}")
    for fragment in ("VISIBLE / OCCLUDED", "Grasp candidates", "oriented_rectangle", "arrowedLine"):
        need(fragment in dashboard, f"Missing dashboard behavior {fragment}")

    for relative in ("scripts/build_kty_perception_3d.sh", "scripts/run_kty_perception_3d.sh", "scripts/check_kty_perception_3d.sh"):
        need(read(ROOT / relative).startswith("#!/usr/bin/env bash"), f"Missing shell header {relative}")
    print("KTY classical 3-D perception static validation: OK")

if __name__ == "__main__":
    main()
