"""Centralized fleet state machine, independent of ROS, HTTP, and rendering."""
from dataclasses import asdict, replace
from functools import wraps
import math
import numpy as np
from scipy.spatial.distance import pdist
from drone_swarm.formations.basic import FORMATION_NAMES, generate_formation, ground_grid
from drone_swarm.motion.fleet_trajectory import plan_transition
from drone_swarm.swarm.lighting import light_colors
from drone_swarm.swarm.separation import depth_layer
from drone_swarm.utils.fleet_config import FleetConfig

MOVING = {'TAKEOFF', 'TRANSITIONING'}
ANIMATED = {'rainbow', 'pulse', 'wave', 'blink', 'fade'}
MOTION_SETTINGS = {'size', 'altitude', 'speed', 'transition_time', 'collision_avoidance'}


def _atomic_control_edit(method):
    """Restore rejected control edits; these methods replace arrays, never edit them in place."""
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        previous = self.__dict__.copy()
        try:
            return method(self, *args, **kwargs)
        except Exception:
            self.__dict__.clear()
            self.__dict__.update(previous)
            raise
    return wrapped


def _check_world_bounds(lower, upper):
    if (np.any(lower[:2] < -500) or np.any(upper[:2] > 500)
            or lower[2] < 0.5 or upper[2] > 500):
        raise ValueError('Formation or transition exceeds the simulator world bounds')


class FleetController:
    def __init__(self, config=None):
        self.config = config or FleetConfig()
        self.formation, self.state, self.message = 'circle', 'IDLE', 'Ready for takeoff'
        self.simulation_time = self.progress = self._elapsed = self._duration = 0.0
        self.positions = ground_grid(self.config.count, self.config.min_distance + 1e-4)
        self.targets = self.positions.copy()
        self.velocities = np.zeros_like(self.positions)
        self._image_colors = None
        self.colors = light_colors(self.config, None, self.positions, 0)
        self._plan = self._paused_state = self._queued_formation = self._queued_config = None
        self._pending_formation = False
        self._image_data = self._image_result = self._image_key = None
        self._image_options = {}
        self._start_colors, self._target_colors = self.colors.copy(), self.colors.copy()
        self.depth_layers = 1

    def _raw_targets(self, name=None, config=None):
        name, config = name or self.formation, config or self.config
        if name == 'image':
            if self._image_data is None:
                raise ValueError('Upload an image before selecting Image formation')
            key = (config.count, config.size, config.altitude)
            if key != self._image_key:
                from drone_swarm.image_processing import process_image
                result = process_image(self._image_data, *key, **self._image_options)
            else:
                result = self._image_result
            targets, colors = result.points.copy(), result.colors.copy()
            # Candidate validation remains pure. Accepted/current geometry can
            # refresh the cache; atomic edits roll this back if planning fails.
            if config is self.config:
                self._image_result, self._image_key = result, key
        else:
            targets = generate_formation(name, config.count, config.size, config.altitude)
            colors = np.tile(config.color, (config.count, 1))
        if targets[:, 2].min() < 0.5:
            raise ValueError('The formation reaches below ground; raise altitude or reduce size')
        return targets, colors

    def _formation_data(self, name=None, config=None):
        config = config or self.config
        targets, colors = self._raw_targets(name, config)
        layers = 1
        if config.collision_avoidance:
            targets, layers = depth_layer(targets, config.min_distance)
        _check_world_bounds(targets.min(axis=0), targets.max(axis=0))
        return targets, colors, layers

    def _formation_targets(self):
        targets, colors, self.depth_layers = self._formation_data()
        return targets, colors

    def load_image(self, data, **options):
        from drone_swarm.image_processing import process_image
        key = (self.config.count, self.config.size, self.config.altitude)
        result = process_image(data, *key, **options)
        return self.set_processed_image(data, options, result, key)

    def set_processed_image(self, data, options, result, key):
        """Install a worker result only if its geometry still matches the fleet.

        Decoding and point selection may run off the ROS executor. This method
        performs bounded validation and commits the finished result on that
        executor, without calling OpenCV or changing a flight trajectory.
        """
        expected = (self.config.count, self.config.size, self.config.altitude)
        if not isinstance(key, tuple) or key != expected:
            raise ValueError('Flight settings changed while processing the image; process it again')
        try:
            points = np.asarray(result.points, dtype=float)
            colors = np.asarray(result.colors, dtype=float)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise ValueError('Processed image must contain finite XYZ points and RGB colors') from error
        shape = (self.config.count, 3)
        if (points.shape != shape or colors.shape != shape or
                not np.isfinite(points).all() or not np.isfinite(colors).all() or
                np.any(colors < 0) or np.any(colors > 1)):
            raise ValueError('Processed image must contain one finite XYZ point and RGB color per drone')
        if len(np.unique(points, axis=0)) != self.config.count:
            raise ValueError('Processed image points must be distinct')
        if not isinstance(data, (bytes, bytearray, memoryview)) or not data or not isinstance(options, dict):
            raise ValueError('Processed image requires source bytes and processing options')
        source, processing_options = bytes(data), dict(options)
        self._image_data, self._image_options = source, processing_options
        self._image_result, self._image_key = result, key
        return result

    def _make_plan(self, start, target, config=None):
        config = config or self.config
        plan = plan_transition(start, target, config.min_distance,
                               config.speed, config.transition_time,
                               config.collision_avoidance)
        _check_world_bounds(*plan.bounds)
        return plan

    def _begin(self, target, state, colors=None):
        plan = self._make_plan(self.positions, target)
        endpoint, _ = plan.sample(plan.duration)
        self._plan, self.targets, self._duration = plan, endpoint, plan.duration
        self._elapsed, self.progress = 0.0, 0.0
        self._start_colors = (self._image_colors if self._image_colors is not None else
                              np.tile(self.config.color, (self.config.count, 1))).copy()
        self._target_colors = self._start_colors.copy() if colors is None else colors[plan.assignment].copy()
        self.state = state
        self.message = 'Taking off' if state == 'TAKEOFF' else f'Transitioning to {self.formation}'

    @_atomic_control_edit
    def _begin_formation(self):
        targets, colors = self._formation_targets()
        self._begin(targets, 'TRANSITIONING', colors)

    @_atomic_control_edit
    def start(self):
        if self.state == 'PAUSED':
            return self.resume()
        if self.state not in {'IDLE', 'STOPPED', 'FORMATION_COMPLETE', 'ANIMATING'}:
            raise ValueError('The swarm is already moving')
        target, colors = self._formation_targets()
        if self.state == 'IDLE':
            takeoff = self.positions.copy()
            takeoff[:, 2] = self.config.altitude
            # Certify the next leg before takeoff so an impossible image or
            # expanded route is rejected while the fleet is still on its grid.
            self._make_plan(takeoff, target)
            self._begin(takeoff, 'TAKEOFF')
            self._pending_formation = True
        else:
            self._begin(target, 'TRANSITIONING', colors)

    def pause(self):
        if self.state not in MOVING | {'ANIMATING', 'FORMATION_COMPLETE'}:
            raise ValueError('There is no active simulation to pause')
        self._paused_state, self.state = self.state, 'PAUSED'
        self.velocities[:] = 0
        self.message = 'Paused'

    def resume(self):
        if self.state != 'PAUSED':
            raise ValueError('The simulation is not paused')
        self.state, self.message = self._paused_state, 'Resumed'
        if self.state not in MOVING:
            self._apply_queued()

    def stop(self):
        self.state, self.message = 'STOPPED', 'Stopped at the current position'
        self._pending_formation = False
        self._queued_config = self._queued_formation = None
        self.velocities[:] = 0

    def _reset_with(self, config):
        saved = (self._image_data, self._image_options, self._image_result, self._image_key)
        self.__init__(config)
        self._image_data, self._image_options, self._image_result, self._image_key = saved

    def reset(self):
        self._reset_with(self.config)

    @_atomic_control_edit
    def configure(self, **settings):
        candidate = (self._queued_config or self.config).updated(**settings)
        scene_change = candidate.count != self.config.count or candidate.min_distance != self.config.min_distance
        if scene_change:
            if self.state not in {'IDLE', 'STOPPED'}:
                raise ValueError('Stop the simulation before changing drone count or minimum separation')
            self._reset_with(candidate)
            return
        geometry_change = any(getattr(candidate, key) != getattr(self.config, key) for key in MOTION_SETTINGS)
        active_state = self._paused_state if self.state == 'PAUSED' else self.state
        if geometry_change:
            targets, _, _ = self._formation_data(self._queued_formation or self.formation, candidate)
            if candidate.collision_avoidance and self.config.count > 1:
                if pdist(self.positions).min() < candidate.min_distance - 1e-8:
                    raise ValueError('Current spacing is too small to enable avoidance; stop and reset first')
            if active_state in MOVING or self.state in {'PAUSED', 'FORMATION_COMPLETE', 'ANIMATING'}:
                boundary = self._plan.sample(self._plan.duration)[0] if active_state in MOVING else self.positions
                self._make_plan(boundary, targets, candidate)
        if geometry_change and (active_state in MOVING or self.state == 'PAUSED'):
            self._queued_config = candidate
            self.config = replace(self.config, **{k: getattr(candidate, k) for k in ('color', 'color_mode', 'brightness')})
            self.message = 'Motion settings queued for the next formation boundary'
        else:
            self._queued_config = None
            self.config = candidate
            if geometry_change and self.state in {'FORMATION_COMPLETE', 'ANIMATING'}:
                self._begin_formation()
        self.colors = light_colors(self.config, self._image_colors, self.positions, self.simulation_time)
        if self.state == 'ANIMATING' and self.config.color_mode not in ANIMATED:
            self.state, self.message = 'FORMATION_COMPLETE', f'{self.formation.title()} formation complete'

    @_atomic_control_edit
    def apply_formation(self, name):
        if name not in (*FORMATION_NAMES, 'image'):
            raise ValueError('Unknown formation: ' + str(name))
        active_state = self._paused_state if self.state == 'PAUSED' else self.state
        if name != 'image' or self._image_data is not None or self.state not in {'IDLE', 'STOPPED'}:
            candidate = self._queued_config or self.config
            targets, _, _ = self._formation_data(name, candidate)
            if active_state in MOVING or self.state not in {'IDLE', 'STOPPED'}:
                boundary = self._plan.sample(self._plan.duration)[0] if active_state in MOVING else self.positions
                self._make_plan(boundary, targets, candidate)
        if active_state in MOVING or self.state == 'PAUSED':
            self._queued_formation = name
            self.message = f'{name.title()} queued; current motion will finish smoothly'
            return
        self.formation = name
        if self.state not in {'IDLE', 'STOPPED'}:
            self._begin_formation()

    @_atomic_control_edit
    def _apply_queued(self):
        if self._queued_config is None and self._queued_formation is None:
            return False
        if self._queued_config is not None:
            self.config, self._queued_config = self._queued_config, None
        if self._queued_formation is not None:
            self.formation, self._queued_formation = self._queued_formation, None
        self._pending_formation = False
        self._begin_formation()
        return True

    def step(self, dt):
        if not math.isfinite(dt) or not 0 <= dt <= 0.2:
            raise ValueError('Simulation step must be finite and between 0 and 0.2 seconds')
        if self.state in {'IDLE', 'STOPPED', 'PAUSED', 'ERROR'} or dt == 0:
            return
        self.simulation_time += dt
        if self.state in MOVING:
            self._elapsed = min(self._duration, self._elapsed + dt)
            self.progress = self._elapsed / self._duration
            self.positions, self.velocities = self._plan.sample(self._elapsed)
            easing = self.progress**2 * (3 - 2*self.progress)
            self._image_colors = self._start_colors + (self._target_colors - self._start_colors) * easing
            if self.progress >= 1:
                if self._apply_queued():
                    pass
                elif self._pending_formation:
                    self._pending_formation = False
                    self._begin_formation()
                else:
                    self.state, self.message = 'FORMATION_COMPLETE', f'{self.formation.title()} formation complete'
        elif self.state == 'FORMATION_COMPLETE' and self.config.color_mode in ANIMATED:
            self.state, self.message = 'ANIMATING', f'{self.config.color_mode.title()} lighting effect'
        self.colors = light_colors(self.config, self._image_colors, self.positions, self.simulation_time)

    def snapshot(self):
        separation = float(pdist(self.positions).min()) if self.config.count > 1 else None
        return {
            'state': self.state, 'message': self.message,
            'simulation_time': self.simulation_time, 'progress': self.progress,
            'count': self.config.count, 'formation': self.formation,
            'config': asdict(self.config), 'positions': self.positions.tolist(),
            'colors': self.colors.tolist(), 'targets': self.targets.tolist(),
            'speed_m_s': float(np.linalg.norm(self.velocities, axis=1).mean()),
            'min_separation_m': separation, 'depth_layers': self.depth_layers,
            'transition_duration_s': self._duration,
            'certified_minimum_separation_m': None if self._plan is None else self._plan.minimum_separation,
            'trajectory_expansion': 1.0 if self._plan is None else self._plan.expansion,
            'pending_formation': self._queued_formation,
            'pending_config': None if self._queued_config is None else asdict(self._queued_config),
            'image': None if self._image_result is None else self._image_result.metadata,
        }
