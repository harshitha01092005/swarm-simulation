import unittest
import numpy as np
from drone_swarm.swarm.fleet import FleetController
from drone_swarm.utils.fleet_config import FleetConfig


class FleetTests(unittest.TestCase):
    def test_five_drones_take_off_and_arrive_together(self):
        fleet = FleetController(FleetConfig(transition_time=2.0, altitude=12.0))
        start = fleet.positions.copy()
        fleet.start()
        states = {fleet.state}
        peak = 0
        for _ in range(3000):
            fleet.step(1/60)
            states.add(fleet.state)
            peak = max(peak, np.linalg.norm(fleet.velocities, axis=1).max())
            if fleet.state == "FORMATION_COMPLETE":
                break
        self.assertEqual(fleet.state, "FORMATION_COMPLETE")
        self.assertIn("TAKEOFF", states)
        self.assertIn("TRANSITIONING", states)
        self.assertTrue(np.allclose(fleet.positions, fleet.targets))
        self.assertFalse(np.allclose(fleet.positions, start))
        self.assertLessEqual(peak, fleet.config.speed + 1e-10)

    def test_pause_and_stop_hold_positions(self):
        fleet = FleetController()
        fleet.start()
        fleet.step(0.1)
        fleet.pause()
        before = fleet.positions.copy()
        time_before = fleet.simulation_time
        fleet.step(0.1)
        np.testing.assert_array_equal(before, fleet.positions)
        self.assertEqual(time_before, fleet.simulation_time)
        fleet.resume()
        fleet.step(0.1)
        self.assertGreater(fleet.simulation_time, time_before)
        fleet.stop()
        before = fleet.positions.copy()
        fleet.step(0.1)
        np.testing.assert_array_equal(before, fleet.positions)

    def test_ground_violation_and_invalid_count_are_rejected(self):
        with self.assertRaises(ValueError):
            FleetConfig(count=401)
        with self.assertRaises(ValueError):
            FleetController(FleetConfig(altitude=3, size=30)).start()
        with self.assertRaises(ValueError):
            FleetConfig(count=True)

    def test_count_requires_stop_and_reset_restores_grid(self):
        fleet = FleetController()
        original = fleet.positions.copy()
        fleet.start()
        with self.assertRaises(ValueError):
            fleet.configure(count=20)
        fleet.step(0.1)
        fleet.reset()
        np.testing.assert_array_equal(fleet.positions, original)
        fleet.configure(count=20)
        self.assertEqual(fleet.positions.shape, (20, 3))
