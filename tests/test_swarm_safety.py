from pathlib import Path
from dataclasses import replace
import unittest
import numpy as np
import cv2
from scipy.spatial.distance import pdist
from drone_swarm.swarm.fleet import FleetController
from drone_swarm.swarm.separation import depth_layer
from drone_swarm.utils.fleet_config import FleetConfig


class SwarmSafetyTests(unittest.TestCase):
    @staticmethod
    def dense_image():
        image = np.full((512, 512, 3), 255, dtype=np.uint8)
        image[246:266, 246:266] = 0
        success, encoded = cv2.imencode('.png', image)
        if not success:
            raise RuntimeError('Could not encode test image')
        return encoded.tobytes()

    def finish_fast(self, fleet):
        for _ in range(20000):
            if fleet.state in {'FORMATION_COMPLETE', 'ANIMATING'}:
                return
            fleet.step(0.2)
        self.fail('Fleet failed to complete its planned route')

    def finish(self, fleet):
        minimum, peak_ratio = float('inf'), 0
        for _ in range(10000):
            fleet.step(1 / 30)
            minimum = min(minimum, pdist(fleet.positions).min())
            peak_ratio = max(peak_ratio, np.linalg.norm(fleet.velocities, axis=1).max() / fleet.config.speed)
            if fleet.state in {'FORMATION_COMPLETE', 'ANIMATING'}:
                break
        self.assertIn(fleet.state, {'FORMATION_COMPLETE', 'ANIMATING'})
        self.assertGreaterEqual(minimum, fleet.config.min_distance - 1e-8)
        self.assertLessEqual(peak_ratio, 1 + 1e-8)

    def test_400_drones_preserve_clearance_across_all_patterns_and_image(self):
        fleet = FleetController(FleetConfig(count=400))
        fleet.load_image((Path(__file__).resolve().parents[1] / 'assets/sample.png').read_bytes())
        fleet.start()
        self.finish(fleet)
        for name in ['star', 'spiral', 'wave', 'infinity', 'image']:
            fleet.apply_formation(name)
            self.finish(fleet)

    def test_layering_preserves_front_view_and_handles_duplicate_projection(self):
        points = np.array([[0., 0., 5.], [0., 0., 5.], [1., 0., 5.]])
        layered, layers = depth_layer(points, 1.5)
        np.testing.assert_array_equal(layered[:, [0, 2]], points[:, [0, 2]])
        self.assertEqual(layers, 3)
        self.assertGreaterEqual(pdist(layered).min(), 1.5)

    def test_motion_edits_queue_and_pause_freezes_effects(self):
        fleet = FleetController(FleetConfig(count=20, color_mode='rainbow'))
        fleet.start()
        fleet.step(.1)
        before = fleet.positions.copy()
        fleet.apply_formation('star')
        fleet.configure(speed=3.0, altitude=25.0)
        np.testing.assert_array_equal(before, fleet.positions)
        self.assertEqual(fleet.snapshot()['pending_formation'], 'star')
        fleet.pause()
        frozen = fleet.colors.copy()
        clock = fleet.simulation_time
        fleet.step(.1)
        np.testing.assert_array_equal(frozen, fleet.colors)
        self.assertEqual(clock, fleet.simulation_time)
        fleet.resume()
        self.finish(fleet)
        self.assertEqual(fleet.formation, 'star')
        self.assertEqual(fleet.config.speed, 3.0)

    def test_scene_changes_require_stop_and_reset_retains_uploaded_image(self):
        fleet = FleetController()
        fleet.load_image((Path(__file__).resolve().parents[1] / 'assets/sample.png').read_bytes())
        fleet.start()
        with self.assertRaises(ValueError):
            fleet.configure(min_distance=2)
        fleet.stop()
        fleet.configure(count=100, min_distance=2)
        fleet.reset()
        fleet.apply_formation('image')
        fleet.start()
        self.finish(fleet)

    def test_invalid_live_geometry_leaves_config_unchanged(self):
        fleet = FleetController()
        fleet.start()
        original = fleet.config
        with self.assertRaises(ValueError):
            fleet.configure(size=120, altitude=3)
        self.assertEqual(original, fleet.config)

    def test_restoring_current_speed_cancels_queued_speed(self):
        fleet = FleetController()
        fleet.start()
        fleet.configure(speed=3)
        self.assertEqual(fleet.snapshot()['pending_config']['speed'], 3)
        fleet.configure(speed=5)
        self.assertIsNone(fleet.snapshot()['pending_config'])
        self.finish_fast(fleet)
        self.assertEqual(fleet.config.speed, 5)

    def test_world_bounds_cover_expansion_not_just_final_formation(self):
        fleet = FleetController(FleetConfig(count=400, size=75, altitude=80, min_distance=3))
        fleet.load_image(self.dense_image(), mode='filled')
        fleet.apply_formation('image')
        targets, _ = fleet._formation_targets()
        self.assertLess(np.abs(targets).max(), 500)
        before = fleet.snapshot()
        with self.assertRaisesRegex(ValueError, 'world bounds'):
            fleet.start()
        self.assertEqual(fleet.snapshot(), before)
        self.assertEqual(fleet.state, 'IDLE')

    def test_rejected_configure_restores_all_previous_control_state(self):
        fleet = FleetController(FleetConfig(count=400, size=120, altitude=80, min_distance=3))
        fleet.load_image(self.dense_image(), mode='filled')
        fleet.apply_formation('image')
        fleet.start()
        self.finish_fast(fleet)
        before = fleet.snapshot()
        with self.assertRaisesRegex(ValueError, 'world bounds'):
            fleet.configure(size=5)
        self.assertEqual(fleet.snapshot(), before)
        self.assertEqual(fleet.config.size, 120)

    def test_rejected_formation_keeps_previous_selection_and_plan(self):
        fleet = FleetController(FleetConfig(count=400, min_distance=3))
        fleet.start()
        self.finish_fast(fleet)
        fleet.load_image(self.dense_image(), mode='filled')
        before = fleet.snapshot()
        with self.assertRaisesRegex(ValueError, 'world bounds'):
            fleet.apply_formation('image')
        self.assertEqual(fleet.snapshot(), before)
        self.assertEqual(fleet.formation, 'circle')

    def test_rejected_queued_geometry_keeps_original_pending_selection(self):
        fleet = FleetController(FleetConfig(count=400, size=120, altitude=80, min_distance=3))
        fleet.load_image(self.dense_image(), mode='filled')
        fleet.start()
        fleet.apply_formation('image')
        before = fleet.snapshot()
        with self.assertRaisesRegex(ValueError, 'world bounds'):
            fleet.configure(size=75)
        self.assertEqual(fleet.snapshot(), before)
        self.assertEqual(fleet.snapshot()['pending_formation'], 'image')

    def test_queued_commit_failure_does_not_commit_config_or_formation(self):
        fleet = FleetController(FleetConfig(count=400, size=75, altitude=80, min_distance=3))
        fleet.load_image((Path(__file__).resolve().parents[1] / 'assets/sample.png').read_bytes())
        fleet.start()
        fleet.apply_formation('image')
        fleet.configure(speed=3)
        # Reprocessing an uploaded image can change geometry after it was
        # queued. A later failed plan must still preserve the accepted state.
        fleet.load_image(self.dense_image(), mode='filled')
        config = fleet.config
        with self.assertRaisesRegex(ValueError, 'world bounds'):
            for _ in range(20000):
                fleet.step(0.2)
        self.assertEqual(fleet.config, config)
        self.assertEqual(fleet.formation, 'circle')
        self.assertEqual(fleet.snapshot()['pending_config']['speed'], 3)
        self.assertEqual(fleet.snapshot()['pending_formation'], 'image')

    def test_recount_refreshes_image_cache_and_metadata_when_applied(self):
        fleet = FleetController(FleetConfig(count=20))
        fleet.load_image((Path(__file__).resolve().parents[1] / 'assets/sample.png').read_bytes())
        fleet.configure(count=100)
        fleet.apply_formation('image')
        fleet.start()
        self.finish_fast(fleet)
        self.assertEqual(fleet.snapshot()['image']['count'], 100)
        self.assertEqual(fleet._image_key, (100, fleet.config.size, fleet.config.altitude))
        self.assertEqual(fleet._image_result.colors.shape, (100, 3))

    def test_disabling_animation_reports_formation_complete(self):
        fleet = FleetController(FleetConfig(color_mode='pulse'))
        fleet.start()
        self.finish_fast(fleet)
        fleet.step(0.1)
        self.assertEqual(fleet.state, 'ANIMATING')
        fleet.configure(color_mode='single')
        self.assertEqual(fleet.state, 'FORMATION_COMPLETE')

    def test_worker_image_commit_rejects_stale_settings_and_invalid_rgb_atomically(self):
        fleet = FleetController(FleetConfig(count=20))
        data = (Path(__file__).resolve().parents[1] / 'assets/sample.png').read_bytes()
        result = fleet.load_image(data)
        key = (fleet.config.count, fleet.config.size, fleet.config.altitude)
        before = fleet.snapshot()
        for colors in (np.zeros((19, 3)), np.full((20, 3), np.nan), np.full((20, 3), 1.1)):
            invalid = replace(result, colors=colors)
            with self.assertRaises(ValueError):
                fleet.set_processed_image(data, {}, invalid, key)
            self.assertEqual(fleet.snapshot(), before)
        fleet.configure(count=30)
        before = fleet.snapshot()
        with self.assertRaisesRegex(ValueError, 'settings changed'):
            fleet.set_processed_image(data, {}, result, key)
        self.assertEqual(fleet.snapshot(), before)
