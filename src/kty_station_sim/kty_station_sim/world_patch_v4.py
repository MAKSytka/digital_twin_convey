"""Build the runtime-v14+ KTY world using a robust transport envelope."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from .world_patch_v3 import build_surface_world


SCENE_BROADCASTER_NAME = "gz::sim::systems::SceneBroadcaster"
CONTACT_PLUGIN_NAME = "kty_conveyor_surface::KtyConveyorSurfaceSystem"


def _set_required_text(parent: ET.Element, path: str, value: str) -> None:
    element = parent.find(path)
    if element is None:
        raise RuntimeError(f"Missing generated SDF element: {path}")
    element.text = value


def build_runtime_v13_world(source: Path, destination: Path) -> Path:
    """Generate the accepted surface world with a lighter runtime profile.

    SceneBroadcaster remains available for GUI state.  Runtime v15+ gets named
    model poses from the contact-surface JSON registry.  The transport envelope is
    deliberately wider than the nominal contact plane so an empty queued KTY that
    is lifted or tilted by a stray product still receives admission velocity.
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

    contact_plugin = world.find(f"plugin[@name='{CONTACT_PLUGIN_NAME}']")
    if contact_plugin is None:
        raise RuntimeError("KTY contact-surface system is missing")
    _set_required_text(contact_plugin, "contact_tolerance", "0.300")

    # Increase overlap around the infeed / active hand-off.  Both zones receive
    # the same command during POSITION_NEXT, so overlap cannot reverse the KTY;
    # it only prevents the model centre from falling into an undriven gap.
    for zone in contact_plugin.findall("zone"):
        name = zone.findtext("name", default="")
        if name == "infeed":
            _set_required_text(zone, "max_x", "-0.100")
        elif name == "active":
            _set_required_text(zone, "min_x", "-0.800")
        _set_required_text(zone, "min_y", "-0.500")
        _set_required_text(zone, "max_y", "0.500")

    # Remove the obsolete PosePublisher if an older generated tree is reused.
    for plugin in list(world.findall("plugin")):
        if plugin.attrib.get("name") == "gz::sim::systems::PosePublisher":
            world.remove(plugin)

    tree.write(destination, encoding="utf-8", xml_declaration=True)
    ET.parse(destination)
    return destination
