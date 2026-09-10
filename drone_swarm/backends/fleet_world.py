"""Generate a self-contained Gazebo world for the kinematic fleet backend.

The fleet uses static models moved by a single Gazebo system. It does not
load the Physics system or simulate motors, lift, aircraft dynamics, or
physical collision avoidance. Gazebo still advances its simulation clock.
"""

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


MAX_DRONES = 400
WORLD_LIMIT = 500.0
INITIAL_ALTITUDE = 0.5
PHYSICS_HZ = 120


def _asset_root() -> Path:
    """Prefer installed ROS assets; allow generation before ROS is installed."""
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        pass
    else:
        try:
            return Path(get_package_share_directory("drone_swarm_simulator"))
        except PackageNotFoundError:
            pass
    root = Path(__file__).resolve().parents[2]
    if (root / "worlds/drone_swarm.sdf").is_file():
        return root
    raise FileNotFoundError("Drone swarm assets are not installed or present in the source tree")


def initial_grid(count: int, spacing: float = 1.5) -> tuple[tuple[float, float, float], ...]:
    """Return deterministic row-major positions for drone_001 through drone_N."""
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= MAX_DRONES:
        raise ValueError(f"count must be an integer from 1 to {MAX_DRONES}")
    if isinstance(spacing, bool) or not isinstance(spacing, (int, float)):
        raise ValueError("spacing must be a finite number of at least 1 metre")
    if not math.isfinite(spacing) or spacing < 1.0:
        raise ValueError("spacing must be a finite number of at least 1 metre")
    columns = math.ceil(math.sqrt(count))
    rows = math.ceil(count / columns)
    positions = tuple(
        ((index % columns - (columns - 1) / 2) * spacing,
         (index // columns - (rows - 1) / 2) * spacing,
         INITIAL_ALTITUDE)
        for index in range(count)
    )
    if any(abs(axis) > WORLD_LIMIT for position in positions for axis in position[:2]):
        raise ValueError("initial grid exceeds the backend's +/-500 metre world bounds")
    return positions


def render_world(count: int, spacing: float = 1.5) -> str:
    """Render a world containing one plugin and ``count`` static quadrotors.

    References sent to /swarm/reference specify the complete active fleet.
    Models omitted from a reference batch are parked below the ground and
    excluded from /swarm/poses; they can subsequently be reactivated.
    """
    positions = initial_grid(count, spacing)
    assets = _asset_root()
    document = ET.parse(assets / "worlds/drone_swarm.sdf").getroot()
    world = document.find("world")
    if world is None:
        raise ValueError("worlds/drone_swarm.sdf must contain a world")
    world.set("name", "fleet_world")
    # FleetSystem owns static model poses directly. Preserve scene and command
    # systems, while avoiding a physics-engine traversal for every fleet step.
    for system in list(world.findall("plugin")):
        if system.get("name") == "gz::sim::systems::Physics":
            world.remove(system)
    physics = world.find("physics")
    if physics is not None:
        physics.set("name", "fleet_kinematic_physics")
        # SimulationRunner uses these timing settings even without Physics.
        # Four world steps per 30 Hz reference preserve responsive updates.
        for key, value in (("max_step_size", 1 / PHYSICS_HZ),
                           ("real_time_update_rate", PHYSICS_HZ)):
            element = physics.find(key)
            if element is None:
                element = ET.SubElement(physics, key)
            element.text = str(value)
    title = world.find("gui/plugin/gz-gui/title")
    if title is not None:
        title.text = "Drone Swarm · Kinematic fleet"
    camera = world.find("gui/plugin/camera_pose")
    if camera is not None:
        extent = max(10.0, math.ceil(math.sqrt(count)) * spacing)
        camera.text = f"{extent:.6g} {-extent * 1.25:.6g} {extent * 0.8:.6g} 0 0.34 2.2455"

    plugin = ET.SubElement(world, "plugin", {
        "filename": "swarm_fleet_system", "name": "drone_swarm::FleetSystem",
    })
    for key, value in (
        ("model_count", count), ("publish_frequency", 30),
        ("reference_topic", "/swarm/reference"), ("pose_topic", "/swarm/poses"),
    ):
        ET.SubElement(plugin, key).text = str(value)

    template = ET.parse(assets / "models/drone/model.sdf").getroot().find("model")
    if template is None:
        raise ValueError("models/drone/model.sdf must contain a model")
    template.find("static").text = "true"
    for node in list(template.findall("plugin")):
        template.remove(node)
    # Kinematic bodies do not take part in contact dynamics. Preserve their
    # primitive visual geometry, including all four visible rotor disks.
    for link in template.findall("link"):
        for collision in list(link.findall("collision")):
            link.remove(collision)

    for index, (x, y, z) in enumerate(positions, start=1):
        model = deepcopy(template)
        model.set("name", f"drone_{index:03d}")
        pose = model.find("pose")
        if pose is None:
            pose = ET.SubElement(model, "pose")
        pose.text = f"{x:.10g} {y:.10g} {z:.10g} 0 0 0"
        world.append(model)
    ET.indent(document, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(document, encoding="unicode") + "\n"


def generate_world(count: int, output_path: str | Path, spacing: float = 1.5) -> Path:
    """Write a generated world and return its absolute path."""
    contents = render_world(count, spacing)
    path = Path(output_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf-8")
    return path
