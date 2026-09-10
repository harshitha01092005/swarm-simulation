from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.distance import cdist

from drone_swarm.swarm.fleet import FleetController
from drone_swarm.utils.fleet_config import FleetConfig


class ImageFleetTests(unittest.TestCase):
    def test_hundred_drones_reach_every_image_target_and_keep_pixel_colors(self):
        fleet = FleetController(FleetConfig(count=100, transition_time=2.0, color_mode="image"))
        result = fleet.load_image((Path(__file__).resolve().parents[1] / "assets/sample.png").read_bytes())
        fleet.start()
        for _ in range(3000):
            fleet.step(1 / 30)
            if fleet.state == "FORMATION_COMPLETE":
                break
        fleet.apply_formation("image")
        for _ in range(3000):
            fleet.step(1 / 30)
            if fleet.state == "FORMATION_COMPLETE":
                break
        self.assertEqual(fleet.state, "FORMATION_COMPLETE")
        mapping = cdist(fleet.positions[:, [0, 2]], result.points[:, [0, 2]]).argmin(axis=1)
        self.assertEqual(len(set(mapping)), 100)
        np.testing.assert_allclose(fleet.positions[:, [0, 2]], result.points[mapping][:, [0, 2]])
        np.testing.assert_allclose(fleet.colors, result.colors[mapping])

    def test_image_without_upload_rejected_before_takeoff(self):
        fleet = FleetController()
        fleet.apply_formation("image")
        with self.assertRaises(ValueError):
            fleet.start()
        self.assertEqual(fleet.state, "IDLE")
