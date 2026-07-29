#!/usr/bin/env python3
"""Static cross-file validation for the KTY RGB-D perception stage."""

from __future__ import annotations

from pathlib import Path
import py_compile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "kty_station_sim"
INTERFACES = ROOT / "src" / "singulator_interfaces"
WORLD = PACKAGE / "worlds" / "kty_flow.sdf"
LAUNCH = PACKAGE / "launch" / "kty_vision.launch.py"
PERCEPTION = PACKAGE / "kty_station_sim" / "depth_perception.py"
DASHBOARD = PACKAGE / "kty_station_sim" / "vision_dashboard.py"
RECORDER = PACKAGE / "kty_station_sim" / "contour_recorder.py"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    require(path.is_file(), f"Missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def validate_world() -> None:
    root = ET.parse(WORLD).getroot()
    world = root.find("world")
    require(world is not None, "KTY flow world is missing")
    require(world.attrib.get("name") == "kty_flow", "Unexpected world name")

    camera_model = world.find("model[@name='kty_flow_vision_station']")
    require(camera_model is not None, "Overhead camera model is missing")
    require(camera_model.findtext("static") == "true", "Camera must be static")
    sensor = camera_model.find(".//sensor[@name='overhead_rgbd']")
    require(sensor is not None, "RGB-D sensor is missing")
    require(sensor.attrib.get("type") == "rgbd_camera", "Sensor must be RGB-D")
    require(sensor.findtext("topic") == "/kty/vision", "Unexpected camera topic")
    require(sensor.findtext("update_rate") == "15", "Expected 15 Hz RGB-D camera")
    require(sensor.findtext("camera/image/width") == "800", "Expected 800 px width")
    require(sensor.findtext("camera/image/height") == "600", "Expected 600 px height")

    text = read(WORLD)
    for fragment in (
        "gz-sim-sensors-system",
        "<render_engine>ogre2</render_engine>",
        "<topic>/kty/vision</topic>",
        "<horizontal_fov>1.05</horizontal_fov>",
        "<near>0.20</near>",
        "<far>3.0</far>",
    ):
        require(fragment in text, f"Missing RGB-D world fragment: {fragment}")


def validate_interfaces() -> None:
    contour = read(INTERFACES / "msg" / "KtyProductContour.msg")
    array = read(INTERFACES / "msg" / "KtyProductContourArray.msg")
    for fragment in (
        "uint32 track_id",
        "geometry_msgs/Polygon polygon",
        "geometry_msgs/Point32 centroid",
        "float32 top_height_m",
        "bool side_neg_x_accessible",
        "bool side_pos_x_accessible",
        "bool side_neg_y_accessible",
        "bool side_pos_y_accessible",
    ):
        require(fragment in contour, f"Contour interface is missing: {fragment}")
    for fragment in (
        "uint32 frame_sequence",
        "bool camera_ok",
        "float32 valid_depth_fraction",
        "KtyProductContour[] products",
    ):
        require(fragment in array, f"Contour array interface is missing: {fragment}")

    cmake = read(INTERFACES / "CMakeLists.txt")
    require(
        '"msg/KtyProductContour.msg"' in cmake,
        "KtyProductContour is not registered",
    )
    require(
        '"msg/KtyProductContourArray.msg"' in cmake,
        "KtyProductContourArray is not registered",
    )


def validate_python() -> None:
    for path in (LAUNCH, PERCEPTION, DASHBOARD, RECORDER):
        py_compile.compile(str(path), doraise=True)

    perception = read(PERCEPTION)
    for fragment in (
        'super().__init__("kty_depth_perception")',
        '"/kty/perception/contours"',
        '"/kty/perception/debug_image"',
        "class GreedyTracker",
        "cv2.approxPolyDP",
        "contour.polygon.points.append(point)",
        "contour.track_id = detection.track_id",
        "side_neg_x_accessible",
        "side_pos_y_accessible",
    ):
        require(fragment in perception, f"Missing perception behavior: {fragment}")

    recorder = read(RECORDER)
    for fragment in (
        'super().__init__("kty_contour_recorder")',
        '"/kty/vision/polygons_json"',
        '"polygons.jsonl"',
        '"polygons_latest.json"',
        '"polygon_m"',
        '"accessible_sides"',
        "tempfile.mkstemp",
    ):
        require(fragment in recorder, f"Missing recorder behavior: {fragment}")

    dashboard = read(DASHBOARD)
    for fragment in (
        'super().__init__("kty_vision_dashboard")',
        '"/kty/vision/dashboard"',
        "cv2.COLORMAP_TURBO",
        "KTY top-view polygons",
        "Tracked objects",
        "cv2.imshow",
        "self.dashboard_pub.publish(output)",
    ):
        require(fragment in dashboard, f"Missing dashboard behavior: {fragment}")


def validate_launch_and_package() -> None:
    launch = read(LAUNCH)
    for fragment in (
        "kty_flow.launch.py",
        'arguments=["/kty/vision/image"]',
        'arguments=["/kty/vision/depth_image"]',
        'executable="depth_perception"',
        'executable="contour_recorder"',
        'executable="vision_dashboard"',
        '"camera_to_kty_bottom_m": 1.25',
        '"show_window": ParameterValue(show_dashboard, value_type=bool)',
        'DeclareLaunchArgument("show_dashboard", default_value="true")',
        'default_value="~/.ros/kty_vision"',
    ):
        require(fragment in launch, f"Missing vision launch wiring: {fragment}")

    setup = read(PACKAGE / "setup.py")
    require(
        'version="0.4.0"' in setup or 'version="0.5.0"' in setup,
        "Expected vision-compatible setup.py version 0.4.0 or 0.5.0",
    )
    for fragment in (
        "depth_perception = kty_station_sim.depth_perception:main",
        "contour_recorder = kty_station_sim.contour_recorder:main",
        "vision_dashboard = kty_station_sim.vision_dashboard:main",
    ):
        require(fragment in setup, f"Missing entry point: {fragment}")

    package_xml = read(PACKAGE / "package.xml")
    require(
        "<version>0.4.0</version>" in package_xml
        or "<version>0.5.0</version>" in package_xml,
        "Expected vision-compatible package version 0.4.0 or 0.5.0",
    )
    for dependency in (
        "cv_bridge",
        "sensor_msgs",
        "singulator_interfaces",
        "ros_gz_image",
        "python3-numpy",
        "python3-opencv",
    ):
        require(
            f"<exec_depend>{dependency}</exec_depend>" in package_xml,
            f"Missing package dependency: {dependency}",
        )


def validate_scripts() -> None:
    scripts = (
        "scripts/build_kty_vision.sh",
        "scripts/run_kty_vision.sh",
        "scripts/check_kty_vision.sh",
        "scripts/stop_kty_vision.sh",
    )
    for relative in scripts:
        text = read(ROOT / relative)
        require(text.startswith("#!/usr/bin/env bash"), f"Missing shebang: {relative}")

    diagnostic = read(ROOT / "scripts/check_kty_vision.sh")
    for fragment in (
        "/kty_depth_perception",
        "/kty_contour_recorder",
        "/kty_vision_dashboard",
        "/kty/vision/image",
        "/kty/vision/depth_image",
        "/kty/perception/contours",
        "/kty/vision/dashboard",
        "/kty/vision/polygons_json",
        "non-empty product polygons",
        "polygons_latest.json",
        "KTY vision diagnostics: OK",
    ):
        require(fragment in diagnostic, f"Missing diagnostic behavior: {fragment}")


def main() -> None:
    validate_world()
    validate_interfaces()
    validate_python()
    validate_launch_and_package()
    validate_scripts()
    print("KTY vision static validation: OK")


if __name__ == "__main__":
    main()
