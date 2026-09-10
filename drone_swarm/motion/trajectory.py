"""Analytical cubic trajectories with zero endpoint velocity."""

from dataclasses import dataclass
import math
from .geometry import Vec3


@dataclass(frozen=True)
class Trajectory:
    start: Vec3
    target: Vec3
    duration: float

    def __post_init__(self):
        if not math.isfinite(self.duration) or self.duration <= 0:
            raise ValueError("Trajectory duration must be positive and finite")

    @classmethod
    def at_speed(cls, start, target, duration, maximum_speed):
        if not math.isfinite(maximum_speed) or maximum_speed <= 0:
            raise ValueError("Maximum speed must be positive and finite")
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("Requested duration must be positive and finite")
        # The cubic easing derivative peaks at 1.5; extend to respect speed.
        return cls(start, target, max(duration, 1.5 * (target - start).norm / maximum_speed))

    def sample(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError("Elapsed time must be finite")
        u = min(1.0, max(0.0, elapsed / self.duration))
        delta = self.target - self.start
        position = self.start + delta * (u*u*(3 - 2*u))
        velocity = delta * (6*u*(1 - u) / self.duration)
        return position, velocity
