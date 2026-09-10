"""Deterministic, continuously certified transitions for point-like drone fleets.

Squared-distance optimal assignment makes every pair's start and target
separation vectors have a nonnegative dot product (otherwise swapping those
two assignments lowers the cost). A common-progress straight morph therefore
retains at least the smaller endpoint separation divided by sqrt(2).

When necessary, expand both clouds, morph, then contract. Expansion about a
pivot at the ground floor preserves altitude and cannot reduce separation.
Every segment uses the same cubic progress for all drones. Certificates apply
to the continuous references, not only to samples at the controller's rate.
Physical tracking error and asynchronous execution require an additional
separation allowance at the caller; this planner does not establish that bound.
"""

from dataclasses import dataclass
import math
from numbers import Real

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist


MAX_DRONES = 500
FLOOR_HEIGHT = 0.5
MAX_COORDINATE = 1_000_000.0
_RELATIVE_TOLERANCE = 1e-10
_TRANSIT_MARGIN = 1e-6


def _positive_number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a positive finite number")
    result = float(value)
    if not math.isfinite(result) or not 0 < result <= MAX_COORDINATE:
        raise ValueError(f"{name} must be positive and at most {MAX_COORDINATE:g}")
    return result


def _points(value, name):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf":
            raise ValueError("Coordinates must be real numbers")
        points = np.array(raw, dtype=np.float64, copy=True)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite numeric Nx3 array") from error
    if points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= MAX_DRONES:
        raise ValueError(f"{name} must contain between 1 and {MAX_DRONES} XYZ positions")
    if not np.isfinite(points).all() or np.abs(points).max() > MAX_COORDINATE:
        raise ValueError(f"{name} coordinates must be finite and within +/-{MAX_COORDINATE:g} m")
    if points[:, 2].min() < FLOOR_HEIGHT:
        raise ValueError(f"{name} coordinates must stay at or above z={FLOOR_HEIGHT} m")
    return points


def _pair_separations(start, target):
    """Return start, target and continuous straight-morph minimum distances.

    For each pair, relative position is a + u*b. Its squared norm is a
    quadratic minimized at clip(-dot(a,b)/dot(b,b), 0, 1). Cubic time easing
    visits the same u interval, so this is also its exact geometric minimum.
    No-pair (one drone) distances are represented by None for JSON consumers.
    """
    if len(start) == 1:
        return None, None, None
    first, second = np.triu_indices(len(start), 1)
    a = start[first] - start[second]
    end = target[first] - target[second]
    b = end - a
    denominator = np.einsum("ij,ij->i", b, b)
    numerator = -np.einsum("ij,ij->i", a, b)
    closest = np.zeros_like(denominator)
    np.divide(numerator, denominator, out=closest, where=denominator > 0)
    np.clip(closest, 0.0, 1.0, out=closest)
    relative = a + closest[:, None] * b
    return tuple(float(np.sqrt(np.einsum("ij,ij->i", points, points).min()))
                 for points in (a, end, relative))


def _expand(points, scale):
    pivot = np.array([points[:, 0].mean(), points[:, 1].mean(), FLOOR_HEIGHT])
    return pivot + scale * (points - pivot)


@dataclass(frozen=True)
class _Segment:
    start: np.ndarray
    target: np.ndarray
    duration: float


@dataclass(frozen=True)
class FleetTrajectory:
    """Read-only planned geometry; sample() returns independent writable arrays.

    assignment[i] is the input target column for input drone i.
    duration is at least the requested duration and may grow to respect speed.
    minimum_separation includes the endpoints and is None for one drone.
    expansion is 1 when direct travel suffices (or avoidance is disabled).
    """

    assignment: np.ndarray
    duration: float
    minimum_separation: float | None
    expansion: float
    _segments: tuple

    @property
    def segment_durations(self):
        """Durations of the nonempty expand/morph/contract segments."""
        return tuple(segment.duration for segment in self._segments)

    @property
    def bounds(self):
        """Return minimum/maximum XYZ coordinates across the complete route.

        Each cubic segment stays inside the axis-aligned bounds of its two
        endpoints, including expansion and contraction waypoints.
        """
        endpoints = [points for segment in self._segments
                     for points in (segment.start, segment.target)]
        return (np.minimum.reduce([points.min(axis=0) for points in endpoints]),
                np.maximum.reduce([points.max(axis=0) for points in endpoints]))

    def sample(self, elapsed):
        """Return positions and velocities, clamped to a stationary endpoint.

        All drones share segment boundaries and progress. The maximum cubic
        derivative is 1.5, which the planner accounts for when allocating time.
        Elapsed time uses the caller's simulation clock; sampling is stateless.
        """
        if isinstance(elapsed, (bool, np.bool_)) or not isinstance(elapsed, Real):
            raise ValueError("Elapsed time must be finite")
        elapsed = float(elapsed)
        if not math.isfinite(elapsed):
            raise ValueError("Elapsed time must be finite")
        if elapsed <= 0:
            positions = self._segments[0].start.copy()
            return positions, np.zeros_like(positions)
        if elapsed >= self.duration:
            positions = self._segments[-1].target.copy()
            return positions, np.zeros_like(positions)
        remaining = elapsed
        for segment in self._segments:
            if remaining <= segment.duration:
                u = min(1.0, max(0.0, remaining / segment.duration))
                delta = segment.target - segment.start
                positions = segment.start + delta * (u * u * (3.0 - 2.0 * u))
                velocities = delta * (6.0 * u * (1.0 - u) / segment.duration)
                return positions, velocities
            remaining -= segment.duration
        # Protect the final endpoint against accumulated floating-point sums.
        positions = self._segments[-1].target.copy()
        return positions, np.zeros_like(positions)


def plan_transition(start, target, min_distance, speed, duration, avoidance=True):
    """Assign targets and plan a synchronized transition for 1..500 drones.

    Coordinates are metres, z must be >=0.5, and coordinate magnitudes are
    bounded by 1e6. Distance/speed/requested duration must be positive, finite
    and <=1e6. With avoidance enabled, both endpoint clouds must already meet
    min_distance (up to 1e-10 relative floating-point tolerance).

    Transit gets an extra 1e-6 m of clearance when expansion is needed. This
    does not claim extra clearance at an endpoint that is exactly min_distance
    apart: minimum_separation reports the smallest certified value over all
    segments, including their endpoints. Avoidance=False permits smaller
    distances and reports that actual certificate without expanding the clouds.

    Cost: O(N^3) assignment in SciPy, O(N^2) certificate work/storage, and O(N)
    per sample. Each moving segment's duration is proportional to its largest
    drone displacement, with T >= 1.5*distance/speed for the cubic easing.
    """
    start = _points(start, "Start")
    target = _points(target, "Target")
    if target.shape != start.shape:
        raise ValueError("Start and target must have the same drone count")
    min_distance = _positive_number(min_distance, "Minimum distance")
    speed = _positive_number(speed, "Speed")
    duration = _positive_number(duration, "Requested duration")
    if not isinstance(avoidance, bool):
        raise ValueError("Avoidance must be true or false")

    # Squared costs are essential to the pairwise monotonicity argument.
    rows, columns = linear_sum_assignment(cdist(start, target, metric="sqeuclidean"))
    assignment = np.empty(len(start), dtype=np.intp)
    assignment[rows] = columns
    target = target[assignment].copy()
    start_min, target_min, direct_min = _pair_separations(start, target)

    expansion = 1.0
    minimum = direct_min
    if avoidance and direct_min is not None:
        endpoint_min = min(start_min, target_min)
        if endpoint_min < min_distance * (1.0 - _RELATIVE_TOLERANCE):
            raise ValueError(
                f"Endpoint separation {endpoint_min:.9g} m is below minimum "
                f"{min_distance:.9g} m; enlarge or depth-layer the formation"
            )
        if direct_min <= 0:
            raise ValueError("Cannot certify this assignment at the supplied coordinate precision")
        if direct_min < min_distance:
            expansion = (min_distance + _TRANSIT_MARGIN) / direct_min

    if expansion > 1.0:
        expanded_start = _expand(start, expansion)
        expanded_target = _expand(target, expansion)
        # Certify the actual floating-point expanded coordinates too.
        expanded_min = _pair_separations(expanded_start, expanded_target)[2]
        if expanded_min < min_distance * (1.0 - _RELATIVE_TOLERANCE):
            raise ValueError("Expanded transit could not be certified at coordinate precision")
        minimum = min(start_min, target_min, expanded_min)
        waypoints = (start, expanded_start, expanded_target, target)
    else:
        waypoints = (start, target)

    displacements = [float(np.linalg.norm(right - left, axis=1).max())
                     for left, right in zip(waypoints[:-1], waypoints[1:])]
    path_length = math.fsum(displacements)
    total_duration = max(duration, 1.5 * path_length / speed)
    if not math.isfinite(total_duration):
        raise ValueError("Speed and path length produce a non-finite duration")

    segments = []
    if path_length == 0:
        segments.append(_Segment(start, target, total_duration))
    else:
        for left, right, distance in zip(waypoints[:-1], waypoints[1:], displacements):
            if distance > 0:
                segments.append(_Segment(left, right, total_duration * (distance / path_length)))
    for segment in segments:
        segment.start.setflags(write=False)
        segment.target.setflags(write=False)
    assignment.setflags(write=False)
    return FleetTrajectory(assignment, total_duration, minimum, expansion, tuple(segments))
