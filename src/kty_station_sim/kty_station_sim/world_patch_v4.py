"""Build the runtime-v13 KTY world with a persistent, rate-limited pose stream."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from .world_patch_v3 import build_surface_world


POSE_PUBLISHER_NAME = "gz::sim::systems::PosePublisher"


def _set_required_text(parent: ET.Element, path: str, value: str) -> None:
    element = parent.find(path)
    if element is None:
        raise RuntimeError(f"Missing generated SDF element: {path}")
    element.text = value


def build_runtime_v13_world(source: Path, destination: Path) -> Path:
    """Generate the accepted surface world plus a 20 Hz model-only Pose_V topic."""
    build_surface_world(source, destination)
    tree = ET.parse(destination)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("Generated SDF has no world")

    # Lower the CPU cost while retaining over 20 physics samples per 9 Hz
    # vibration cycle. Horizontal KTY transport is deterministic in the Gazebo
    # system plugin, so it does not require a 250 Hz solver.
    physics = world.find("physics")
    if physics is None:
        raise RuntimeError("Generated world has no physics block")
    _set_required_text(physics, "max_step_size", "0.005")
    _set_required_text(physics, "real_time_update_rate", "200")

    sensor = world.find(".//sensor[@name='overhead_rgbd']")
    if sensor is None:
        raise RuntimeError("RGB-D sensor is missing")
    _set_required_text(sensor, "update_rate", "5")
    _set_required_text(sensor, "camera/image/width", "448")
    _set_required_text(sensor, "camera/image/height", "336")

    for plugin in list(world.findall("plugin")):
        if plugin.attrib.get("name") == POSE_PUBLISHER_NAME:
            world.remove(plugin)

    publisher = ET.SubElement(
        world,
        "plugin",
        {
            "filename": "gz-sim-pose-publisher-system",
            "name": POSE_PUBLISHER_NAME,
        },
    )
    ET.SubElement(publisher, "publish_link_pose").text = "false"
    ET.SubElement(publisher, "publish_visual_pose").text = "false"
    ET.SubElement(publisher, "publish_collision_pose").text = "false"
    ET.SubElement(publisher, "publish_sensor_pose").text = "false"
    ET.SubElement(publisher, "publish_model_pose").text = "true"
    ET.SubElement(publisher, "publish_nested_model_pose").text = "true"
    ET.SubElement(publisher, "use_pose_vector_msg").text = "true"
    ET.SubElement(publisher, "update_frequency").text = "20"
    ET.SubElement(publisher, "static_publisher").text = "false"

    tree.write(destination, encoding="utf-8", xml_declaration=True)
    ET.parse(destination)
    return destination
