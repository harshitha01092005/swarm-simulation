"""Validated settings for the centralized swarm controller."""

from dataclasses import asdict, dataclass, replace
import math


@dataclass(frozen=True)
class FleetConfig:
    count: int = 5
    size: float = 20.0
    altitude: float = 20.0
    transition_time: float = 8.0
    speed: float = 5.0
    min_distance: float = 1.5
    collision_avoidance: bool = True
    color_mode: str = "single"
    color: tuple = (0.2, 0.8, 1.0)
    brightness: float = 1.0

    def __post_init__(self):
        if isinstance(self.count, bool) or not isinstance(self.count, int) or not 1 <= self.count <= 400:
            raise ValueError("Drone count must be an integer between 1 and 400")
        for key, limits in {
            "size": (5, 120), "altitude": (3, 100), "transition_time": (1, 120),
            "speed": (0.1, 15), "min_distance": (0.3, 3), "brightness": (0, 1),
        }.items():
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not limits[0] <= value <= limits[1]:
                raise ValueError(f"{key} must be a finite number between {limits[0]} and {limits[1]}")
        if not isinstance(self.collision_avoidance, bool):
            raise ValueError("collision_avoidance must be true or false")
        if self.color_mode not in ("single", "image", "rainbow", "pulse", "wave", "blink", "fade"):
            raise ValueError("Unknown color mode")
        if len(self.color) != 3 or not all(isinstance(v, (float, int)) and math.isfinite(v) and 0 <= v <= 1 for v in self.color):
            raise ValueError("Color must contain three RGB values from 0 to 1")

    def updated(self, **changes):
        unknown = set(changes) - set(asdict(self))
        if unknown:
            raise ValueError("Unknown settings: " + ", ".join(sorted(unknown)))
        if "color" in changes:
            changes["color"] = tuple(changes["color"])
        return replace(self, **changes)
