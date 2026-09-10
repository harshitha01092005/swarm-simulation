"""Validated mission settings, shared by adapters and core tests."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MissionConfig:
    target_altitude: float = 4.0
    max_speed: float = 1.5
    transition_time: float = 6.0
    update_rate: float = 30.0
    feedback_timeout: float = 2.0

    def __post_init__(self):
        ranges = {
            "target_altitude": (0.75, 25.0), "max_speed": (0.05, 5.0),
            "transition_time": (0.5, 120.0), "update_rate": (5.0, 100.0),
            "feedback_timeout": (0.1, 10.0),
        }
        for key, (low, high) in ranges.items():
            value = getattr(self, key)
            if isinstance(value, bool) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} must be between {low} and {high}")
