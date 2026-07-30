"""Build the runtime-v14 KTY world using SceneBroadcaster pose feedback."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from .world_patch_v3 import build_surface_world


SCENE_BROADCASTER_NAME = "gz::sim::systems::SceneBroadcaster"


def _set_required_text(parent: ET.Element, path: str, value: str) -> None:
    element = parent.find(path)
    if element is None:
        raise RuntimeError(f"Missing generated SDF element: {path}")
    element.text = value


def build_runtime_v13_world(source: Path, destination: Path) -> Path:
    """Generate the accepted surface world with a lighter runtime profile.

    SceneBroadcaster is already part of the base world and publishes the
    canonical ``/world/<name>/dynamic_pose/info`` Pose_V topic.  Runtime v14
    bridges that existing stream instead of adding a second PosePublisher with
    a different topic name.
    """
    build_surface_world(source, destination)
    tree = ET.parse(destination)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("Generated SDF has no world")

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

    scene_broadcaster = world.find(
        f"plugin[@name='{SCENE_BROADCASTER_NAME}']"
    )
    if scene_broadcaster is None:
        raise RuntimeError("SceneBroadcaster system is required for dynamic poses")

    # Remove the v13 PosePublisher if an older generated tree is reused.  Its
    # default world-scoped topic did not match the bridge and caused the startup
    # deadlock fixed by runtime v14.
    for plugin in list(world.findall("plugin")):
        if plugin.attrib.get("name") == "gz::sim::systems::PosePublisher":
            world.remove(plugin)

    tree.write(destination, encoding="utf-8", xml_declaration=True)
    ET.parse(destination)
    return destination
