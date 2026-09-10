"""Structural checks catch missing installed resources before ROS is installed."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class AssetTests(unittest.TestCase):
    def test_ros_manifest_and_resource_marker_match(self):
        package = ET.parse(ROOT / "package.xml").getroot()
        self.assertEqual(package.findtext("name"), "drone_swarm_simulator")
        self.assertTrue((ROOT / "resource" / package.findtext("name")).is_file())

    def test_model_has_feedback_velocity_and_four_rotors(self):
        model = ET.parse(ROOT / "models/drone/model.sdf").getroot().find("model")
        self.assertEqual(model.attrib["name"], "drone_001")
        link = model.find("link")
        self.assertEqual(link.findtext("gravity"), "false")
        rotors = [v for v in link.findall("visual") if v.attrib["name"].startswith("rotor_")]
        self.assertEqual(len(rotors), 4)
        plugins = {p.attrib["name"]: p for p in model.findall("plugin")}
        self.assertIn("gz::sim::systems::VelocityControl", plugins)
        odom = plugins["gz::sim::systems::OdometryPublisher"]
        self.assertEqual(odom.findtext("dimensions"), "3")
        self.assertEqual(odom.findtext("odom_frame"), "world")

    def test_world_supplies_spawning_physics_and_scene(self):
        world = ET.parse(ROOT / "worlds/drone_swarm.sdf").getroot().find("world")
        self.assertEqual(world.attrib["name"], "swarm_world")
        plugins = {p.attrib["name"] for p in world.findall("plugin")}
        self.assertTrue({"gz::sim::systems::Physics", "gz::sim::systems::UserCommands",
                         "gz::sim::systems::SceneBroadcaster"}.issubset(plugins))

    def test_assets_do_not_download_external_models(self):
        for name in ("worlds/drone_swarm.sdf", "models/drone/model.sdf"):
            root = ET.parse(ROOT / name).getroot()
            self.assertEqual(root.findall(".//uri"), [])


if __name__ == "__main__":
    unittest.main()
