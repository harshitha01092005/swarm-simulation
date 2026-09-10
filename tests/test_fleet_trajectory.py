import math
import time
import unittest

import numpy as np
from scipy.spatial.distance import pdist

from drone_swarm.motion.fleet_trajectory import plan_transition


class FleetTrajectoryTests(unittest.TestCase):
    def setUp(self):
        # Perpendicular pair separations attain the 1/sqrt(2) direct bound.
        self.start = np.array([[-1.0, 0.0, 4.0], [1.0, 0.0, 4.0]])
        self.target = np.array([[0.0, -1.0, 4.0], [0.0, 1.0, 4.0]])

    def plan(self, **changes):
        settings = dict(start=self.start, target=self.target, min_distance=2.0,
                        speed=3.0, duration=1.0, avoidance=True)
        settings.update(changes)
        return plan_transition(**settings)

    def test_expansion_prevents_perpendicular_pair_collision_continuously(self):
        plan = self.plan()
        self.assertAlmostEqual(plan.expansion, math.sqrt(2), places=5)
        self.assertEqual(len(plan.segment_durations), 3)
        # Endpoints are exactly 2m apart, so the certificate must not claim the
        # additional 1e-6m transit margin for the whole route.
        self.assertAlmostEqual(plan.minimum_separation, 2.0)
        expand, morph, _ = plan.segment_durations
        midpoint = plan.sample(expand + morph / 2)[0]
        self.assertGreaterEqual(pdist(midpoint).min(), 2.0)
        for elapsed in np.linspace(0, plan.duration, 161):
            positions, _ = plan.sample(elapsed)
            self.assertGreaterEqual(pdist(positions).min(), 2.0 - 1e-10)
            self.assertGreaterEqual(positions[:, 2].min(), 0.5)

    def test_avoidance_off_reports_the_smaller_actual_certificate(self):
        plan = self.plan(avoidance=False)
        self.assertEqual(plan.expansion, 1.0)
        self.assertEqual(len(plan.segment_durations), 1)
        self.assertAlmostEqual(plan.minimum_separation, math.sqrt(2))
        positions, _ = plan.sample(plan.duration / 2)
        self.assertAlmostEqual(pdist(positions).min(), plan.minimum_separation)

    def test_assignment_uses_squared_cost_not_distance(self):
        start = [[0, 3, 4], [0, -5, 4], [-4, -4, 4]]
        target = [[1, 5, 4], [5, 5, 4], [2, -3, 4]]
        plan = self.plan(start=start, target=target, min_distance=1)
        # Ordinary distance assignment [0,2,1] yields a negative pair dot
        # product and does not support the planner's expansion bound.
        np.testing.assert_array_equal(plan.assignment, [1, 2, 0])
        np.testing.assert_array_equal(plan.sample(plan.duration)[0], np.array(target)[[1, 2, 0]])

    def test_exact_endpoint_positions_zero_velocities_and_clamping(self):
        plan = self.plan()
        for elapsed in (-10, 0):
            positions, velocities = plan.sample(elapsed)
            np.testing.assert_array_equal(positions, self.start)
            np.testing.assert_array_equal(velocities, np.zeros_like(self.start))
        for elapsed in (plan.duration, plan.duration + 100):
            positions, velocities = plan.sample(elapsed)
            np.testing.assert_array_equal(positions, self.target[plan.assignment])
            np.testing.assert_array_equal(velocities, np.zeros_like(self.target))

    def test_peak_speed_bound_and_synchronized_segment_boundaries(self):
        plan = self.plan(speed=0.8, duration=0.01)
        elapsed = 0.0
        for segment_duration in plan.segment_durations:
            peak_velocity = plan.sample(elapsed + segment_duration / 2)[1]
            self.assertAlmostEqual(np.linalg.norm(peak_velocity, axis=1).max(), 0.8, places=8)
            elapsed += segment_duration
            positions, velocities = plan.sample(elapsed)
            self.assertLess(np.linalg.norm(velocities, axis=1).max(), 1e-8)
            epsilon = min(1e-6, segment_duration * 1e-6)
            before = plan.sample(elapsed - epsilon)[0]
            after = plan.sample(elapsed + epsilon)[0]
            self.assertLess(np.linalg.norm(after - before, axis=1).max(), 1e-8)
        self.assertAlmostEqual(sum(plan.segment_durations), plan.duration)

    def test_requested_duration_is_a_minimum_and_allocates_all_segments(self):
        plan = self.plan(duration=90)
        self.assertAlmostEqual(plan.duration, 90)
        self.assertAlmostEqual(sum(plan.segment_durations), 90)
        for elapsed in np.linspace(0, plan.duration, 101):
            self.assertLessEqual(np.linalg.norm(plan.sample(elapsed)[1], axis=1).max(), 3.0)

    def test_floor_pivot_keeps_low_altitude_expansion_above_ground(self):
        start = self.start.copy()
        target = self.target.copy()
        start[:, 2] = target[:, 2] = 0.5
        plan = self.plan(start=start, target=target)
        for elapsed in np.linspace(0, plan.duration, 51):
            np.testing.assert_array_equal(plan.sample(elapsed)[0][:, 2], [0.5, 0.5])

    def test_single_drone_and_stationary_fleet(self):
        single = self.plan(start=[[1, 2, 3]], target=[[4, 6, 3]], min_distance=2, speed=1)
        self.assertIsNone(single.minimum_separation)
        self.assertEqual(single.expansion, 1)
        self.assertAlmostEqual(single.duration, 7.5)
        # A stationary fleet at exactly the required spacing must not move
        # merely to add the optional transit margin.
        stationary = self.plan(target=self.start[::-1], min_distance=2)
        self.assertEqual(stationary.duration, 1)
        self.assertEqual(stationary.expansion, 1)
        for elapsed in (0, 0.5, 1):
            np.testing.assert_array_equal(stationary.sample(elapsed)[0], self.start)
            np.testing.assert_array_equal(stationary.sample(elapsed)[1], np.zeros_like(self.start))

    def test_safe_direct_movement_needs_no_expansion(self):
        plan = self.plan(target=self.start + [4, 5, 3], min_distance=2)
        self.assertEqual(plan.expansion, 1.0)
        self.assertAlmostEqual(plan.minimum_separation, 2.0)

    def test_input_and_sample_mutations_do_not_change_plan(self):
        start, target = self.start.copy(), self.target.copy()
        plan = self.plan(start=start, target=target)
        start[:] = target[:] = 0
        positions, _ = plan.sample(0)
        positions[:] = 0
        np.testing.assert_array_equal(plan.sample(0)[0], self.start)
        with self.assertRaises(ValueError):
            plan.assignment[0] = 99

    def test_invalid_inputs_and_endpoint_spacing_are_rejected(self):
        invalid = [
            {"start": []}, {"start": [[0, 1]]},
            {"start": [[0, 0, 0.5]], "target": self.target},
            {"start": np.zeros((501, 3))},
            {"start": [[0, 0, float("nan")], [1, 2, 3]]},
            {"start": [[0, 0, float("inf")], [1, 2, 3]]},
            {"start": [[0, 0, 0.49], [1, 2, 3]]},
            {"start": [[1_000_001, 0, 4], [0, 0, 4]]},
            {"start": [["1", "2", "3"], ["4", "5", "6"]]},
            {"target": [[0, 0, 4], [0, 0, 4]]},
            {"min_distance": 3}, {"min_distance": 0}, {"min_distance": True},
            {"speed": 0}, {"speed": -1}, {"speed": float("inf")},
            {"duration": 0}, {"duration": float("nan")},
            {"duration": "3"}, {"avoidance": 1},
        ]
        for changes in invalid:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.plan(**changes)
        plan = self.plan()
        for elapsed in (float("nan"), float("inf"), True, "1"):
            with self.subTest(elapsed=elapsed), self.assertRaises(ValueError):
                plan.sample(elapsed)

    def test_replanning_from_a_valid_intermediate_cloud_is_deterministic(self):
        first = self.plan()
        current, _ = first.sample(first.duration * 0.42)
        # Caller is responsible for stopping/braking coherently before replacing
        # a live trajectory; this checks the new position-space certificate.
        target = self.start + [10, 3, 6]
        left = self.plan(start=current, target=target)
        right = self.plan(start=current, target=target)
        np.testing.assert_array_equal(left.assignment, right.assignment)
        self.assertEqual(left.duration, right.duration)
        self.assertEqual(left.expansion, right.expansion)
        self.assertGreaterEqual(left.minimum_separation, 2 - 1e-10)
        np.testing.assert_array_equal(left.sample(0)[0], current)

    def test_400_drone_certificate_and_planning_budget(self):
        x, y = np.meshgrid(np.arange(20) * 1.6, np.arange(20) * 1.6)
        start = np.column_stack((x.ravel(), y.ravel(), np.full(400, 0.5)))
        target = np.column_stack((x.ravel() + 10, np.full(400, 4), y.ravel() + 10))
        beginning = time.perf_counter()
        plan = self.plan(start=start, target=target, min_distance=1.5, speed=5, duration=4)
        planning_seconds = time.perf_counter() - beginning
        self.assertLess(planning_seconds, 5.0)
        self.assertEqual(len(set(plan.assignment)), 400)
        self.assertGreaterEqual(plan.minimum_separation, 1.5 - 1e-10)
        self.assertLessEqual(plan.expansion, math.sqrt(2) * (1 + 1e-6 / 1.5) + 1e-10)
        for elapsed in np.linspace(0, plan.duration, 31):
            positions, velocities = plan.sample(elapsed)
            self.assertGreaterEqual(pdist(positions).min(), 1.5 - 1e-9)
            self.assertLessEqual(np.linalg.norm(velocities, axis=1).max(), 5 + 1e-10)

    def test_maximum_supported_count_is_500(self):
        start = np.column_stack((np.arange(500) * 2, np.zeros(500), np.full(500, 0.5)))
        target = start + [0, 0, 4]
        plan = self.plan(start=start, target=target, min_distance=1.5)
        self.assertEqual(len(plan.assignment), 500)
        np.testing.assert_array_equal(plan.sample(plan.duration)[0], target)


if __name__ == "__main__":
    unittest.main()
