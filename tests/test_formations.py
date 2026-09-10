"""Geometric contracts for sparse and full-sized vertical formations."""

import unittest
import numpy as np

from drone_swarm.formations.basic import (
    FORMATION_NAMES, _resample_path, circle, generate_formation, ground_grid,
)


class FormationTests(unittest.TestCase):
    def test_twenty_drone_patterns_are_finite_unique_and_vertical(self):
        for name in FORMATION_NAMES:
            with self.subTest(name=name):
                result = generate_formation(name, 20, 20.0, 30.0)
                self.assertEqual(result.shape, (20, 3))
                self.assertTrue(np.issubdtype(result.dtype, np.floating))
                self.assertTrue(np.isfinite(result).all())
                self.assertEqual(len(np.unique(np.round(result, 10), axis=0)), 20)
                np.testing.assert_array_equal(result[:, 1], np.zeros(20))
                self.assertLessEqual(np.abs(result[:, 0]).max(), 10.0 + 1e-10)
                self.assertLessEqual(np.abs(result[:, 2] - 30.0).max(), 10.0 + 1e-10)
                self.assertGreaterEqual(result[:, 2].min(), 20.0 - 1e-10)

    def test_sparse_and_400_drone_patterns_have_exact_unique_counts(self):
        for name in FORMATION_NAMES:
            for count in (1, 2, 3, 4, 5, 6, 400):
                with self.subTest(name=name, count=count):
                    points = generate_formation(name, count, 40.0, 30.0)
                    self.assertEqual(points.shape, (count, 3))
                    self.assertTrue(np.isfinite(points).all())
                    self.assertEqual(len(np.unique(np.round(points, 10), axis=0)), count)

    def test_circle_preserves_radius_orientation_and_equal_chords(self):
        points = circle(20, 20, 30)
        np.testing.assert_allclose(points[0], [10, 0, 30])
        distances = np.linalg.norm(points - [0, 0, 30], axis=1)
        np.testing.assert_allclose(distances, 10)
        chords = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        np.testing.assert_allclose(chords, chords[0])
        np.testing.assert_array_equal(points, generate_formation(" CIRCLE ", 20, 20, 30))

    def test_grid_preserves_existing_identity_order(self):
        np.testing.assert_allclose(ground_grid(5), [
            [-1.5, -0.75, 0.5], [0, -0.75, 0.5], [1.5, -0.75, 0.5],
            [-1.5, 0.75, 0.5], [0, 0.75, 0.5],
        ])
        np.testing.assert_array_equal(ground_grid(1), [[0, 0, 0.5]])

    def test_arc_length_sampling_handles_unequal_and_zero_length_segments(self):
        points = _resample_path([[0, 0], [4, 0], [4, 0], [4, 1]], 6)
        np.testing.assert_allclose(points, [[0, 0], [1, 0], [2, 0],
                                            [3, 0], [4, 0], [4, 1]])
        closed = _resample_path([[0, 0], [2, 0], [2, 2], [0, 2]], 8, closed=True)
        distances = np.linalg.norm(np.roll(closed, -1, axis=0) - closed, axis=1)
        np.testing.assert_allclose(distances, np.ones(8))
        np.testing.assert_allclose(_resample_path([[0, 0], [10, 0]], 1), [[5, 0]])

    def test_spiral_does_not_cluster_at_its_center(self):
        points = generate_formation("spiral", 100, 30, 25)
        distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
        # Arc-length samples have nearly equal adjacent chords even though
        # an Archimedean spiral changes radius throughout its path.
        self.assertGreater(distances.min() / distances.max(), 0.75)
        np.testing.assert_allclose(points[0], [0, 0, 25])

    def test_shapes_scale_and_translate_without_changing_order(self):
        for name in FORMATION_NAMES:
            with self.subTest(name=name):
                base = generate_formation(name, 20, 10, 0)
                moved = generate_formation(name, 20, 30, 40)
                np.testing.assert_allclose(moved, base * 3 + [0, 0, 40], atol=1e-12)
                np.testing.assert_array_equal(moved, generate_formation(name, 20, 30, 40))

    def test_ground_clearance_is_delegated_to_the_controller(self):
        points = generate_formation("circle", 20, 20, 1)
        self.assertLess(points[:, 2].min(), 0.5)

    def test_invalid_parameters_raise_clear_errors(self):
        for name in ("", "text", "unknown", None, 4):
            with self.subTest(name=name), self.assertRaises(ValueError):
                generate_formation(name, 20, 20, 30)
        for count in (0, -1, 401, True, 20.0, "20", None):
            with self.subTest(count=count), self.assertRaises(ValueError):
                generate_formation("star", count, 20, 30)
        for size in (0, -1, float("nan"), float("inf"), True, "20", None):
            with self.subTest(size=size), self.assertRaises(ValueError):
                generate_formation("wave", 20, size, 30)
        for altitude in (float("nan"), float("inf"), True, "30", None):
            with self.subTest(altitude=altitude), self.assertRaises(ValueError):
                generate_formation("infinity", 20, 20, altitude)
        for spacing in (0, -1, float("nan"), float("inf"), True, "1.5", None):
            with self.subTest(spacing=spacing), self.assertRaises(ValueError):
                ground_grid(5, spacing)
        with self.assertRaises(ValueError):
            ground_grid(400, 1e308)
        with self.assertRaises(ValueError):
            generate_formation("circle", 20, 1e308, 1.7e308)


if __name__ == "__main__":
    unittest.main()
