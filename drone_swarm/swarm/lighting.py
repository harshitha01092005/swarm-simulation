"""Vectorized RGB effects driven exclusively by the controller simulation clock."""

import numpy as np


def light_colors(config, image_colors, positions, simulation_time):
    count = len(positions)
    colors = np.tile(config.color, (count, 1)).astype(float)
    phase = simulation_time
    if config.color_mode == "image" and image_colors is not None:
        colors = image_colors.copy()
    elif config.color_mode in {"rainbow", "wave"}:
        spatial = positions[:, 0] / max(config.size, 1) + positions[:, 2] / max(config.size, 1)
        hue = spatial + phase * 0.15
        offsets = np.array([0.0, 2 / 3, 1 / 3])
        colors = np.clip(np.abs((hue[:, None] + offsets) % 1 * 6 - 3) - 1, 0, 1)
        if config.color_mode == "wave":
            colors *= (0.2 + 0.8 * (0.5 + 0.5 * np.sin(spatial * 8 - phase * 3)))[:, None]
    elif config.color_mode == "pulse":
        colors *= 0.15 + 0.85 * (0.5 + 0.5 * np.sin(phase * 3))
    elif config.color_mode == "blink":
        colors *= 1.0 if phase % 1 < 0.5 else 0.03
    elif config.color_mode == "fade":
        colors *= 0.5 - 0.5 * np.cos(phase * 0.8)
    return np.clip(colors * config.brightness, 0, 1)
