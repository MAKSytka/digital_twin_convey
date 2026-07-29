"""Generate the corrected stage-7 Gazebo world from the stage-4 SDF.

The first physical world rotated each complete roller link and left the joint
frame at the model origin.  The result was a set of rollers orbiting / tilting
around the wrong anchor.  This patch keeps every link frame unrotated, rotates
only the cylinder geometry and places every revolute joint at the roller
centre.  It also replaces multi-joint velocity controllers with one controller
per roller and lowers camera / physics load for a usable real-time factor.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROLLER_GROUPS = ("infeed", "active", "outfeed")


def _set_text(parent: ET.Element, path: str, value: str) -> None:
    node = parent.find(path)
    if node is None:
        raise RuntimeError(f"Missing SDF element: {path}")
    node.text = value


def _ensure_pose(element: ET.Element, value: str) -> None:
    pose = element.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        element.insert(0, pose)
    pose.attrib.clear()
    pose.text = value


def _roller_group(name: str) -> str | None:
    for group in ROLLER_GROUPS:
        if name.startswith(f"{group}_roller_"):
            return group
    return None


def build_balanced_world(source: Path, destination: Path) -> Path:
    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("Source SDF has no world")
    world.set("name", "kty_mechatronics_v2")

    physics = world.find("physics")
    if physics is None:
        raise RuntimeError("Source SDF has no physics block")
    _set_text(physics, "max_step_size", "0.002")
    _set_text(physics, "real_time_update_rate", "500")

    sensor = world.find(".//sensor[@name='overhead_rgbd']")
    if sensor is None:
        raise RuntimeError("RGB-D sensor is missing")
    _set_text(sensor, "update_rate", "8")
    _set_text(sensor, "camera/image/width", "640")
    _set_text(sensor, "camera/image/height", "480")
    visualize = sensor.find("visualize")
    if visualize is None:
        visualize = ET.SubElement(sensor, "visualize")
    visualize.text = "false"

    machine = world.find("model[@name='kty_mechatronics_machine']")
    if machine is None:
        raise RuntimeError("Machine model is missing")

    # The hinged gate is removed.  Runtime v7 uses an idempotent static gate
    # model created / removed through the already available world services.
    for link in list(machine.findall("link")):
        if link.attrib.get("name") == "gate":
            machine.remove(link)
    for joint in list(machine.findall("joint")):
        if joint.attrib.get("name") == "gate_joint":
            machine.remove(joint)

    roller_positions: dict[str, tuple[float, float, float]] = {}
    for link in machine.findall("link"):
        name = link.attrib.get("name", "")
        group = _roller_group(name)
        if group is None:
            continue
        pose = link.find("pose")
        if pose is None or not pose.text:
            raise RuntimeError(f"Roller {name} has no pose")
        values = [float(value) for value in pose.text.split()[:3]]
        x, y, z = values
        roller_positions[name] = (x, y, z)
        # Keep the link frame aligned with the machine.  Rotate only the
        # cylinder, whose native axis is +Z, onto the physical +Y axle.
        _ensure_pose(link, f"{x:.6f} {y:.6f} {z:.6f} 0 0 0")
        for geometry_owner in (*link.findall("collision"), *link.findall("visual")):
            _ensure_pose(geometry_owner, "0 0 0 1.57079632679 0 0")
        collision = link.find("collision")
        if collision is not None:
            friction = collision.find("surface/friction/ode")
            if friction is not None:
                for tag, value in (("mu", "1.35"), ("mu2", "1.10")):
                    child = friction.find(tag)
                    if child is None:
                        child = ET.SubElement(friction, tag)
                    child.text = value

    roller_joint_names: list[tuple[str, str]] = []
    for joint in machine.findall("joint"):
        name = joint.attrib.get("name", "")
        if not name.endswith("_roller_01_joint") and "_roller_" not in name:
            continue
        child = joint.findtext("child", default="")
        group = _roller_group(child)
        if group is None or child not in roller_positions:
            continue
        x, y, z = roller_positions[child]
        _ensure_pose(joint, f"{x:.6f} {y:.6f} {z:.6f} 0 0 0")
        _set_text(joint, "axis/xyz", "0 1 0")
        _set_text(joint, "axis/limit/effort", "220")
        _set_text(joint, "axis/limit/velocity", "55")
        roller_joint_names.append((name, group))

    # Remove the old grouped roller controllers and the obsolete gate
    # position controller.  Some Gazebo builds only drive the first joint in a
    # grouped JointController, so v7 creates one plugin per physical axle.
    for plugin in list(machine.findall("plugin")):
        filename = plugin.attrib.get("filename", "")
        topic = plugin.findtext("topic", default="")
        if filename == "gz-sim-joint-controller-system" and "_rollers/cmd_vel" in topic:
            machine.remove(plugin)
        elif filename == "gz-sim-joint-position-controller-system" and topic == "/kty/mech/gate/cmd_pos":
            machine.remove(plugin)
        elif filename == "gz-sim-joint-position-controller-system" and topic == "/kty/mech/clamps/cmd_pos":
            machine.remove(plugin)

    for joint_name, group in roller_joint_names:
        plugin = ET.SubElement(
            machine,
            "plugin",
            {
                "filename": "gz-sim-joint-controller-system",
                "name": "gz::sim::systems::JointController",
            },
        )
        ET.SubElement(plugin, "joint_name").text = joint_name
        ET.SubElement(plugin, "topic").text = f"/kty/mech/{group}_rollers/cmd_vel"
        ET.SubElement(plugin, "initial_velocity").text = "0"

    # One controller per clamp avoids the same grouped-controller ambiguity.
    for joint_name in ("clamp_neg_y_joint", "clamp_pos_y_joint"):
        plugin = ET.SubElement(
            machine,
            "plugin",
            {
                "filename": "gz-sim-joint-position-controller-system",
                "name": "gz::sim::systems::JointPositionController",
            },
        )
        ET.SubElement(plugin, "joint_name").text = joint_name
        ET.SubElement(plugin, "topic").text = "/kty/mech/clamps/cmd_pos"
        ET.SubElement(plugin, "p_gain").text = "5200"
        ET.SubElement(plugin, "d_gain").text = "200"
        ET.SubElement(plugin, "cmd_max").text = "1800"
        ET.SubElement(plugin, "cmd_min").text = "-1800"

    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    # Parse the generated file once more before handing it to Gazebo.
    ET.parse(destination)
    return destination
