"""Deterministic formation geometry, independent of ROS and rendering.

Public generators return exactly ``count`` unique finite XYZ points as a
NumPy array of shape ``(count, 3)``. Show formations lie in the vertical XZ
plane, with Y=0 and their design origin at (0, 0, altitude). ``size`` is the
maximum design width/height in metres, not a minimum drone separation.

The caller must validate ground clearance, world bounds, minimum separation,
and transition safety. Sampling a shape alone does not provide those checks.
"""

import math
from numbers import Real
import numpy as np


FORMATION_NAMES = ("circle", "star", "spiral", "wave", "infinity")


def _validate_count(count):
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 400:
        raise ValueError("Count must be an integer between 1 and 400")


def _finite_number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{name} must be {'positive and ' if positive else ''}finite")
    return result


def ground_grid(count, spacing=1.5):
    """Return the row-major launch grid at Z=0.5, with the given XY spacing."""
    _validate_count(count)
    spacing = _finite_number(spacing, "Spacing", positive=True)
    width = math.ceil(math.sqrt(count))
    index = np.arange(count)
    with np.errstate(over="ignore", invalid="ignore"):
        x = (index % width - (width - 1) / 2) * spacing
        y = (index // width - (math.ceil(count / width) - 1) / 2) * spacing
    return _check_points(np.column_stack((x, y, np.full(count, 0.5))))


def circle(count, size, altitude):
    """Return an evenly spaced vertical circle with diameter ``size``.

    The first drone is on the positive X axis, preserving the original API.
    A single drone occupies that position; two drones occupy opposite ends.
    """
    _validate_count(count)
    size = _finite_number(size, "Size", positive=True)
    altitude = _finite_number(altitude, "Altitude")
    angle = np.arange(count) * 2 * np.pi / count
    return _world_points(np.column_stack((np.cos(angle) / 2, np.sin(angle) / 2)),
                         size, altitude)


def _resample_path(points, count, *, closed=False, phase=0.0):
    """Interpolate points at equal distances along a normalized 2D polyline.

    Closed paths omit the duplicated final endpoint. Open paths include both
    endpoints when count>1; count=1 samples the middle of the path. ``phase``
    is a fractional sample offset for closed paths, useful at self-crossings.
    """
    path = np.asarray(points, dtype=float)
    if closed:
        path = np.vstack((path, path[0]))
    # Remove zero-length segments before interpolation.
    keep = np.concatenate(([True], np.any(np.diff(path, axis=0) != 0, axis=1)))
    path = path[keep]
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    distances = np.concatenate(([0.0], np.cumsum(lengths)))
    total = distances[-1]
    if total <= 0:
        raise ValueError("Formation path must have a positive length")
    if closed:
        samples = (np.arange(count) + phase) * total / count
    elif count == 1:
        samples = np.array([total / 2])
    else:
        samples = np.linspace(0.0, total, count)
    return np.column_stack([np.interp(samples, distances, path[:, axis])
                            for axis in range(2)])


def _world_points(points, size, altitude):
    with np.errstate(over="ignore", invalid="ignore"):
        coordinates = points * size
        result = np.column_stack((coordinates[:, 0], np.zeros(len(points)),
                                  altitude + coordinates[:, 1]))
    return _check_points(result)


def _check_points(result):
    if not np.all(np.isfinite(result)) or len(np.unique(result, axis=0)) != len(result):
        raise ValueError("Requested geometry exceeds numerical precision for this formation")
    return result


def generate_formation(name, count, size, altitude):
    """Generate ``circle``, ``star``, ``spiral``, ``wave``, or ``infinity``.

    Names are case-insensitive. Circle and star are closed outlines; spiral is
    an open 2.5-turn Archimedean spiral. Wave has two cycles and a 2.5:1 design
    aspect ratio; infinity has a 2:1 ratio. Curves are densely approximated,
    then resampled by arc length so drones do not cluster at parameter bends.
    Star's straight edges are resampled exactly by length.

    Infinity's sample phase avoids placing two drones at the exact crossing.
    Nearby samples can still be close: minimum separation belongs to the
    caller. Counts 1 and 2 are supported as sparse samples of the same paths.
    """
    if not isinstance(name, str) or name.strip().lower() not in FORMATION_NAMES:
        raise ValueError("Unknown formation; choose " + ", ".join(FORMATION_NAMES))
    name = name.strip().lower()
    _validate_count(count)
    size = _finite_number(size, "Size", positive=True)
    altitude = _finite_number(altitude, "Altitude")
    if name == "circle":
        return circle(count, size, altitude)
    samples = max(4097, count * 16 + 1)
    if name == "star":
        angles = np.pi / 2 + np.arange(10) * np.pi / 5
        radii = np.where(np.arange(10) % 2 == 0, 0.5, 0.22)
        path = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
        points = _resample_path(path, count, closed=True)
    elif name == "spiral":
        parameter = np.linspace(0.0, 1.0, samples)
        angles = parameter * 5 * np.pi
        path = np.column_stack((parameter / 2 * np.cos(angles),
                                parameter / 2 * np.sin(angles)))
        points = _resample_path(path, count)
    elif name == "wave":
        parameter = np.linspace(0.0, 1.0, samples)
        path = np.column_stack((parameter - 0.5, 0.2 * np.sin(4 * np.pi * parameter)))
        points = _resample_path(path, count)
    else:
        angles = np.linspace(0.0, 2 * np.pi, samples, endpoint=False)
        path = np.column_stack((0.5 * np.cos(angles), 0.25 * np.sin(2 * angles)))
        # An irrational fractional offset avoids the two exact center crossings
        # without perturbing the shape or the uniform arc-length interval.
        points = _resample_path(path, count, closed=True, phase=(3 - math.sqrt(5)) / 2)
    return _world_points(points, size, altitude)
