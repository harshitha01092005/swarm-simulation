"""Small immutable geometry primitives, independent of ROS messages."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.z)):
            raise ValueError("Vector coordinates must be finite")

    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, value):
        return Vec3(self.x * value, self.y * value, self.z * value)

    @property
    def norm(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def limited(self, maximum):
        if not math.isfinite(maximum) or maximum <= 0:
            raise ValueError("Speed limit must be positive and finite")
        return self * (maximum / self.norm) if self.norm > maximum else self


def world_to_body(velocity: Vec3, quaternion) -> Vec3:
    """Inverse-rotate a world vector by a normalized xyzw orientation."""
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError("Orientation must contain four finite xyzw values")
    length = math.sqrt(sum(v * v for v in quaternion))
    if length < 1e-9:
        raise ValueError("Orientation quaternion cannot be zero")
    x, y, z, w = (v / length for v in quaternion)
    # Transpose of the body-to-world quaternion rotation matrix.
    return Vec3(
        (1 - 2 * (y*y + z*z)) * velocity.x + 2 * (x*y + z*w) * velocity.y + 2 * (x*z - y*w) * velocity.z,
        2 * (x*y - z*w) * velocity.x + (1 - 2 * (x*x + z*z)) * velocity.y + 2 * (y*z + x*w) * velocity.z,
        2 * (x*z + y*w) * velocity.x + 2 * (y*z - x*w) * velocity.y + (1 - 2 * (x*x + y*y)) * velocity.z,
    )
