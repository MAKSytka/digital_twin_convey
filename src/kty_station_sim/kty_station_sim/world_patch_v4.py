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


def _append_chute_guide(
    chute_link: ET.Element,
    *,
    name: str,
    pose: str,
) -> None:
    """Add one tall tapered guide wall to the product chute."""
    collision = ET.SubElement(
        chute_link,
        "collision",
        {"name": f"{name}_collision"},
    )
    ET.SubElement(collision, "pose").text = pose
    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = "1.010 0.030 0.240"

    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    ET.SubElement(ode, "mu").text = "0.25"
    ET.SubElement(ode, "mu2").text = "0.25"
    bounce = ET.SubElement(surface, "bounce")
    ET.SubElement(bounce, "restitution_coefficient").text = "0.0"
    ET.SubElement(bounce, "threshold").text = "0.50"
    contact = ET.SubElement(surface, "contact")
    contact_ode = ET.SubElement(contact, "ode")
    ET.SubElement(contact_ode, "kp").text = "4000000"
    ET.SubElement(contact_ode, "kd").text = "180"
    ET.SubElement(contact_ode, "max_vel").text = "0.03"
    ET.SubElement(contact_ode, "min_depth").text = "0.0003"

    visual = ET.SubElement(
        chute_link,
        "visual",
        {"name": f"{name}_visual"},
    )
    ET.SubElement(visual, "pose").text = pose
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_box = ET.SubElement(visual_geometry, "box")
    ET.SubElement(visual_box, "size").text = "1.010 0.030 0.240"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0.72 0.34 0.10 1"
    ET.SubElement(material, "diffuse").text = "0.90 0.46 0.14 1"


def build_runtime_v13_world(source: Path, destination: Path) -> Path:
    """Generate the accepted surface world with a lighter runtime profile.

    SceneBroadcaster remains available for GUI state. Runtime v15+ gets named
    model poses from the contact-surface JSON registry. The transport envelope is
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

    # Keep the accepted camera orientation and move only its physical height.
    # KTY bottom Z is 0.50 m, therefore camera Z=1.60 m gives 1.10 m to bottom.
    camera_link = world.find(
        "model[@name='kty_mechatronics_vision_station']/link[@name='camera_link']"
    )
    if camera_link is None:
        raise RuntimeError("KTY RGB-D camera link is missing")
    _set_required_text(
        camera_link,
        "pose",
        "0 0 1.60 0 1.57079632679 1.57079632679",
    )

    # The original chute has parallel outer rails. Add a second, visible pair
    # of taller guides which converges from roughly 565 mm at the inlet to
    # roughly 405 mm at the outlet. The outlet therefore matches the 400 mm
    # internal KTY width and prevents products from leaving beside the container.
    chute_link = world.find(
        "model[@name='kty_product_chute']/link[@name='chute']"
    )
    if chute_link is None:
        raise RuntimeError("KTY product chute link is missing")
    _append_chute_guide(
        chute_link,
        name="funnel_guide_neg_y",
        pose="0 -0.255 0.120 0 0 0.079830",
    )
    _append_chute_guide(
        chute_link,
        name="funnel_guide_pos_y",
        pose="0 0.255 0.120 0 0 -0.079830",
    )

    scene_broadcaster = world.find(
        f"plugin[@name='{SCENE_BROADCASTER_NAME}']"
    )
    if scene_broadcaster is None:
        raise RuntimeError("SceneBroadcaster system is required for dynamic poses")

    contact_plugin = world.find(f"plugin[@name='{CONTACT_PLUGIN_NAME}']")
    if contact_plugin is None:
        raise RuntimeError("KTY contact-surface system is missing")
    _set_required_text(contact_plugin, "contact_tolerance", "0.300")

    # Increase overlap around the infeed / active hand-off. Both zones receive
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
