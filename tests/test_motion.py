import math
import unittest

from drone_swarm.motion.geometry import Vec3, world_to_body
from drone_swarm.motion.trajectory import Trajectory
from drone_swarm.utils.config import MissionConfig


class MotionTests(unittest.TestCase):
    def test_zero_velocity_at_endpoints_and_correct_position(self):
        trajectory = Trajectory(Vec3(1, 2, 3), Vec3(4, 6, 8), 5)
        self.assertEqual(trajectory.sample(0), (trajectory.start, Vec3()))
        self.assertEqual(trajectory.sample(5), (trajectory.target, Vec3()))
        self.assertEqual(trajectory.sample(20), (trajectory.target, Vec3()))
        self.assertEqual(trajectory.sample(-1), (trajectory.start, Vec3()))

    def test_speed_bound_and_monotonic_motion(self):
        trajectory = Trajectory.at_speed(Vec3(), Vec3(100, 0, 0), 1, 2)
        previous = -1
        for step in range(101):
            p, v = trajectory.sample(trajectory.duration * step / 100)
            self.assertGreaterEqual(p.x, previous)
            self.assertLessEqual(v.norm, 2 + 1e-10)
            previous = p.x

    def test_zero_distance_remains_finite(self):
        trajectory = Trajectory.at_speed(Vec3(3, 2, 1), Vec3(3, 2, 1), 1, 2)
        self.assertEqual(trajectory.sample(0.5), (Vec3(3, 2, 1), Vec3()))

    def test_world_to_body_rotates_yaw_and_normalizes(self):
        half = math.sqrt(0.5)
        result = world_to_body(Vec3(1, 0, 0), (0, 0, half * 2, half * 2))
        self.assertAlmostEqual(result.x, 0)
        self.assertAlmostEqual(result.y, -1)
        self.assertAlmostEqual(result.norm, 1)

    def test_world_to_body_rotates_pitch(self):
        half = math.sqrt(0.5)
        result = world_to_body(Vec3(0, 0, 1), (0, half, 0, half))
        self.assertAlmostEqual(result.x, -1)
        self.assertAlmostEqual(result.z, 0)

    def test_invalid_values_are_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ValueError):
                Vec3(value, 0, 0)
            with self.assertRaises(ValueError):
                Trajectory(Vec3(), Vec3(), value)
        for duration in (0, -1):
            with self.assertRaises(ValueError):
                Trajectory(Vec3(), Vec3(), duration)
        with self.assertRaises(ValueError):
            world_to_body(Vec3(), (0, 0, 0, 0))
        with self.assertRaises(ValueError):
            Trajectory.at_speed(Vec3(), Vec3(), 1, 0)

    def test_settings_reject_invalid_ranges(self):
        for kwargs in ({"target_altitude": -1}, {"max_speed": 100},
                       {"transition_time": 0}, {"update_rate": float("nan")},
                       {"feedback_timeout": True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                MissionConfig(**kwargs)


if __name__ == "__main__":
    unittest.main()
