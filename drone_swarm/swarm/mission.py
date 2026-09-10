"""A deterministic one-drone state machine, driven by measured simulation time."""

from enum import Enum
import math
from drone_swarm.motion.geometry import Vec3
from drone_swarm.motion.trajectory import Trajectory
from drone_swarm.utils.config import MissionConfig


class State(str, Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    MOVING = "MOVING"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


class Mission:
    def __init__(self, config=None):
        self.config = config or MissionConfig()
        self.state = State.IDLE
        self.reason = "Waiting for Gazebo odometry"
        self.trajectory = None
        self.elapsed = 0.0
        self._last_time = None
        self._targets = []
        self._segment = 0
        self._resume_state = State.IDLE

    def start(self, position, sim_time):
        if self.state not in (State.IDLE, State.STOPPED, State.COMPLETE):
            raise ValueError(f"Cannot start in state {self.state.value}")
        if not math.isfinite(sim_time):
            raise ValueError("Simulation time must be finite")
        altitude = self.config.target_altitude
        self._targets = [Vec3(position.x, position.y, altitude),
                         Vec3(position.x + 3, position.y + 2, altitude),
                         Vec3(position.x, position.y, altitude)]
        self._segment = 0
        self._begin_segment(position, sim_time)

    def _begin_segment(self, position, sim_time):
        self.trajectory = Trajectory.at_speed(position, self._targets[self._segment],
                                             self.config.transition_time, self.config.max_speed)
        self.elapsed = 0.0
        self._last_time = sim_time
        self.state = State.TAKEOFF if self._segment == 0 else State.MOVING
        self.reason = f"Flying segment {self._segment + 1} of {len(self._targets)}"

    def command(self, command, position, sim_time):
        if command == "start":
            self.start(position, sim_time)
        elif command == "pause" and self.state in (State.TAKEOFF, State.MOVING):
            self._resume_state = self.state
            self.state = State.PAUSED
            self.reason = "Paused; zero velocity commanded"
        elif command == "resume" and self.state == State.PAUSED:
            self.state = self._resume_state
            self._last_time = sim_time
            self.reason = "Mission resumed"
        elif command == "stop":
            self.state = State.STOPPED
            self.reason = "Stopped; zero velocity commanded"
        else:
            raise ValueError(f"Command {command!r} is invalid in {self.state.value}")

    def fail(self, reason):
        self.state = State.ERROR
        self.reason = reason

    @property
    def progress(self):
        if self.state == State.COMPLETE:
            return 1.0
        if self.trajectory is None:
            return 0.0
        return (self._segment + min(1.0, self.elapsed / self.trajectory.duration)) / len(self._targets)

    def update(self, position, sim_time):
        if self.state not in (State.TAKEOFF, State.MOVING):
            return Vec3()
        if not math.isfinite(sim_time) or sim_time < self._last_time:
            self.fail("Simulation clock moved backwards or is invalid; restart the controller")
            return Vec3()
        self.elapsed += sim_time - self._last_time
        self._last_time = sim_time
        reference, feedforward = self.trajectory.sample(self.elapsed)
        error = self.trajectory.target - position
        if self.elapsed >= self.trajectory.duration and error.norm < 0.06:
            self._segment += 1
            if self._segment == len(self._targets):
                self._segment -= 1
                self.state = State.COMPLETE
                self.reason = "Demo complete; hovering at the starting X/Y position"
            else:
                self._begin_segment(position, sim_time)
            return Vec3()
        if self.elapsed > self.trajectory.duration + 15:
            self.fail("Drone did not reach its target within the settling timeout")
            return Vec3()
        return (feedforward + (reference - position) * 1.4).limited(self.config.max_speed)
