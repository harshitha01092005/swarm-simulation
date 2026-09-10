"""Give dense planar patterns safe depth while preserving their front view."""

import numpy as np
from scipy.spatial.distance import cdist


def depth_layer(points, minimum):
    """Color the proximity graph; conflicting targets occupy separate Y layers.

    The X/Z image projection is unchanged. Deterministic degree-first greedy
    coloring is O(N²) for at most 400 targets. This establishes endpoint
    clearance; the trajectory planner separately certifies the whole route.
    """
    result = np.asarray(points, dtype=float).copy()
    if result.ndim != 2 or result.shape[1] != 3 or not np.isfinite(result).all():
        raise ValueError("Expected finite XYZ targets")
    if not np.isfinite(minimum) or minimum <= 0:
        raise ValueError("Minimum separation must be positive")
    if len(result) < 2:
        return result, 1
    spacing = minimum + 1e-4
    conflict = cdist(result[:, [0, 2]], result[:, [0, 2]], "sqeuclidean") < spacing**2
    np.fill_diagonal(conflict, False)
    colors = np.full(len(result), -1, dtype=int)
    order = np.argsort(-conflict.sum(axis=1), kind="stable")
    for index in order:
        unavailable = set(colors[conflict[index]])
        color = 0
        while color in unavailable:
            color += 1
        colors[index] = color
    layers = int(colors.max()) + 1
    result[:, 1] = (colors - (layers - 1) / 2) * spacing
    return result, layers
