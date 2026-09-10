"""The mission controller is independent of the transport used by a backend."""

from typing import Protocol
from drone_swarm.motion.geometry import Vec3


class VelocityBackend(Protocol):
    def send_velocity(self, world_velocity: Vec3) -> None:
        """Command velocity in metres/second, expressed in the world frame."""

    def stop(self) -> None:
        """Send an explicit zero-velocity command."""
