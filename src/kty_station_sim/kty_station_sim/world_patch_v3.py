"""Generate the roller-free contact-surface KTY world."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from .world_patch_v2 import build_balanced_world


ROLLER_GROUPS = ("infeed_roller_", "active_roller_", "outfeed_roller_")


def _surface_link(
    name: str,
    pose: str,
    size: str,
    color: str,
) -> ET.Element:
    link = ET.Element("link", {"name": name})
    ET.SubElement(link, "pose").text = pose
    collision = ET.SubElement(link, "collision", {"name": "contact_surface"})
    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = size
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    ET.SubElement(ode, "mu").text = "1.20"
    ET.SubElement(ode, "mu2").text = "1.00"
    contact = ET.SubElement(surface, "contact")
    contact_ode = ET.SubElement(contact, "ode")
    ET.SubElement(contact_ode, "kp").text = "4000000"
    ET.SubElement(contact_ode, "kd").text = "120"
    ET.SubElement(contact_ode, "max_vel").text = "0.08"
    ET.SubElement(contact_ode, "min_depth").text = "0.0003"
    visual = ET.SubElement(link, "visual", {"name": "surface_visual"})
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_box = ET.SubElement(visual_geometry, "box")
    ET.SubElement(visual_box, "size").text = size
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = color
    ET.SubElement(material, "diffuse").text = color
    return link


def _fixed_joint(name: str, child: str) -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "parent").text = "base"
    ET.SubElement(joint, "child").text = child
    return joint


def _add_zone(plugin: ET.Element, *, name: str, topic: str, min_x: float, max_x: float) -> None:
    zone = ET.SubElement(plugin, "zone")
    ET.SubElement(zone, "name").text = name
    ET.SubElement(zone, "topic").text = topic
    ET.SubElement(zone, "min_x").text = f"{min_x:.3f}"
    ET.SubElement(zone, "max_x").text = f"{max_x:.3f}"
    ET.SubElement(zone, "min_y").text = "-0.34"
    ET.SubElement(zone, "max_y").text = "0.34"


def build_surface_world(source: Path, destination: Path) -> Path:
    intermediate = destination.with_suffix(".balanced.sdf")
    build_balanced_world(source, intermediate)
    tree = ET.parse(intermediate)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("Generated SDF has no world")
    world.set("name", "kty_mechatronics_surface")

    machine = world.find("model[@name='kty_mechatronics_machine']")
    if machine is None:
        raise RuntimeError("Machine model is missing")

    # Remove all roller bodies, axle joints and their velocity controllers.
    for link in list(machine.findall("link")):
        name = link.attrib.get("name", "")
        if name.startswith(ROLLER_GROUPS):
            machine.remove(link)
    for joint in list(machine.findall("joint")):
        name = joint.attrib.get("name", "")
        if "_roller_" in name:
            machine.remove(joint)
    for plugin in list(machine.findall("plugin")):
        topic = plugin.findtext("topic", default="")
        if "_rollers/cmd_vel" in topic:
            machine.remove(plugin)

    # Flat inlet and outlet plates. Their upper plane is z=0.500 m.
    machine.append(
        _surface_link(
            "infeed_surface",
            "-1.200 0 0.460 0 0 0",
            "1.620 0.620 0.080",
            "0.16 0.22 0.30 1",
        )
    )
    machine.append(_fixed_joint("infeed_surface_joint", "infeed_surface"))
    machine.append(
        _surface_link(
            "outfeed_surface",
            "1.200 0 0.460 0 0 0",
            "1.620 0.620 0.080",
            "0.16 0.30 0.22 1",
        )
    )
    machine.append(_fixed_joint("outfeed_surface_joint", "outfeed_surface"))

    # The active contact plate is part of the vibrating link, therefore the KTY
    # receives both longitudinal conveyor force and physical Z vibration.
    deck = machine.find("link[@name='vibration_deck']")
    if deck is None:
        raise RuntimeError("Vibration deck is missing")
    active_collision = ET.SubElement(deck, "collision", {"name": "active_contact_surface"})
    ET.SubElement(active_collision, "pose").text = "0 0 0.075 0 0 0"
    geometry = ET.SubElement(active_collision, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = "0.780 0.620 0.040"
    surface = ET.SubElement(active_collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    ET.SubElement(ode, "mu").text = "1.25"
    ET.SubElement(ode, "mu2").text = "1.05"
    active_visual = ET.SubElement(deck, "visual", {"name": "active_surface_visual"})
    ET.SubElement(active_visual, "pose").text = "0 0 0.075 0 0 0"
    visual_geometry = ET.SubElement(active_visual, "geometry")
    visual_box = ET.SubElement(visual_geometry, "box")
    ET.SubElement(visual_box, "size").text = "0.780 0.620 0.040"
    material = ET.SubElement(active_visual, "material")
    ET.SubElement(material, "ambient").text = "0.12 0.38 0.52 1"
    ET.SubElement(material, "diffuse").text = "0.18 0.55 0.72 1"

    plugin = ET.SubElement(
        world,
        "plugin",
        {
            "filename": "KtyConveyorSurfaceSystem",
            "name": "kty_conveyor_surface::KtyConveyorSurfaceSystem",
        },
    )
    ET.SubElement(plugin, "model_prefix").text = "kty_mech_container_"
    ET.SubElement(plugin, "surface_z").text = "0.500"
    ET.SubElement(plugin, "contact_tolerance").text = "0.085"
    ET.SubElement(plugin, "velocity_gain").text = "260.0"
    ET.SubElement(plugin, "max_force").text = "380.0"
    ET.SubElement(plugin, "velocity_deadband").text = "0.004"
    _add_zone(
        plugin,
        name="infeed",
        topic="/kty/mech/infeed_surface/cmd_vel",
        min_x=-2.10,
        max_x=-0.34,
    )
    _add_zone(
        plugin,
        name="active",
        topic="/kty/mech/active_surface/cmd_vel",
        min_x=-0.39,
        max_x=0.39,
    )
    _add_zone(
        plugin,
        name="outfeed",
        topic="/kty/mech/outfeed_surface/cmd_vel",
        min_x=0.34,
        max_x=2.15,
    )

    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    ET.parse(destination)
    intermediate.unlink(missing_ok=True)
    return destination
