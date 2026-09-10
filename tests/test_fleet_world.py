"""Generated fleet resource checks runnable without ROS or Gazebo."""

import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from drone_swarm.backends.fleet_world import generate_world, initial_grid, render_world


ROOT = Path(__file__).resolve().parents[1]


class FleetWorldTests(unittest.TestCase):
    def test_five_models_use_one_fleet_plugin_and_preserve_quadrotors(self):
        world = ET.fromstring(render_world(5)).find("world")
        self.assertEqual(world.get("name"), "fleet_world")
        plugins = world.findall("plugin[@name='drone_swarm::FleetSystem']")
        self.assertEqual(len(plugins), 1)
        self.assertEqual(plugins[0].findtext("model_count"), "5")
        self.assertEqual(plugins[0].findtext("reference_topic"), "/swarm/reference")
        self.assertEqual(plugins[0].findtext("pose_topic"), "/swarm/poses")
        drones = [model for model in world.findall("model")
                  if model.get("name", "").startswith("drone_")]
        self.assertEqual([model.get("name") for model in drones],
                         [f"drone_{index:03d}" for index in range(1, 6)])
        for model in drones:
            self.assertEqual(model.findtext("static"), "true")
            self.assertEqual(model.findall(".//plugin"), [])
            self.assertEqual(model.findall(".//collision"), [])
            rotors = [visual for visual in model.findall(".//visual")
                      if visual.get("name", "").startswith("rotor_")]
            self.assertEqual(len(rotors), 4)
        self.assertEqual(world.findall(".//uri"), [])

    def test_grid_matches_spawn_identity_order_and_spacing(self):
        positions = initial_grid(5)
        self.assertEqual(positions, ((-1.5, -0.75, 0.5), (0.0, -0.75, 0.5),
                                     (1.5, -0.75, 0.5), (-1.5, 0.75, 0.5),
                                     (0.0, 0.75, 0.5)))
        world = ET.fromstring(render_world(5)).find("world")
        for index, expected in enumerate(positions, start=1):
            pose = world.findtext(f"model[@name='drone_{index:03d}']/pose")
            self.assertEqual(tuple(map(float, pose.split()[:3])), expected)
        minimum = min(math.dist(a, b) for index, a in enumerate(positions)
                      for b in positions[index + 1:])
        self.assertGreaterEqual(minimum, 1.5)
        self.assertEqual(initial_grid(1), ((0.0, 0.0, 0.5),))

    def test_kinematic_fleet_omits_physics_and_retains_clock_and_scene_systems(self):
        source = ET.parse(ROOT / "worlds/drone_swarm.sdf").getroot().find("world")
        source_plugins = {plugin.get("name") for plugin in source.findall("plugin")}
        self.assertIn("gz::sim::systems::Physics", source_plugins)
        source_step = source.findtext("physics/max_step_size")
        for count in (1, 5, 400):
            with self.subTest(count=count):
                world = ET.fromstring(render_world(count)).find("world")
                plugins = {plugin.get("name") for plugin in world.findall("plugin")}
                self.assertEqual(plugins, {
                    "gz::sim::systems::UserCommands",
                    "gz::sim::systems::SceneBroadcaster",
                    "drone_swarm::FleetSystem",
                })
                self.assertAlmostEqual(float(world.findtext("physics/max_step_size")), 1 / 120)
                self.assertEqual(float(world.findtext("physics/real_time_update_rate")), 120)
                self.assertEqual(float(world.findtext("physics/real_time_factor")), 1)
        # Generation cannot change the independent one-drone physics demo.
        preserved = ET.parse(ROOT / "worlds/drone_swarm.sdf").getroot().find("world")
        self.assertEqual({plugin.get("name") for plugin in preserved.findall("plugin")},
                         source_plugins)
        self.assertEqual(preserved.findtext("physics/max_step_size"), source_step)

    def test_capacity_400_has_unique_names_without_per_drone_systems(self):
        world = ET.fromstring(render_world(400)).find("world")
        names = [model.get("name") for model in world.findall("model")
                 if model.get("name", "").startswith("drone_")]
        self.assertEqual(len(names), 400)
        self.assertEqual(len(set(names)), 400)
        self.assertEqual(names[-1], "drone_400")
        self.assertEqual(len(world.findall(".//plugin[@name='drone_swarm::FleetSystem']")), 1)
        self.assertLess(max(abs(x) for position in initial_grid(400) for x in position), 20)

    def test_invalid_configuration_rejected_before_writing(self):
        for count in (0, -1, 401, True, 5.0, "5", None):
            with self.subTest(count=count), self.assertRaises(ValueError):
                render_world(count)
        for spacing in (0, -1, 0.9, True, float("nan"), float("inf"), "1.5", None):
            with self.subTest(spacing=spacing), self.assertRaises(ValueError):
                render_world(5, spacing)
        with self.assertRaises(ValueError):
            render_world(400, 100.0)

    def test_generation_preserves_source_assets_and_writes_exact_result(self):
        paths = (ROOT / "worlds/drone_swarm.sdf", ROOT / "models/drone/model.sdf")
        before = [path.read_bytes() for path in paths]
        with tempfile.TemporaryDirectory() as directory:
            output = generate_world(5, Path(directory) / "nested/fleet.sdf")
            self.assertTrue(output.is_absolute())
            self.assertEqual(output.read_text(), render_world(5))
        self.assertEqual([path.read_bytes() for path in paths], before)

    def test_asset_directory_can_be_installed_share(self):
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory)
            for asset in ("worlds/drone_swarm.sdf", "models/drone/model.sdf"):
                output = share / asset
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_bytes((ROOT / asset).read_bytes())
            with patch("drone_swarm.backends.fleet_world._asset_root", return_value=share):
                self.assertEqual(ET.fromstring(render_world(3)).findtext(
                    "world/plugin[@name='drone_swarm::FleetSystem']/model_count"), "3")


if __name__ == "__main__":
    unittest.main()
