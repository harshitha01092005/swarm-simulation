"""One-drone demonstration using measured Gazebo pose and simulation time."""

from dataclasses import fields
import json
import math
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from drone_swarm.backends.ros_gazebo import GazeboVelocityBackend
from drone_swarm.motion.geometry import Vec3, world_to_body
from drone_swarm.swarm.mission import Mission, State
from drone_swarm.utils.config import MissionConfig


class DroneDemo(Node):
    def __init__(self):
        super().__init__("drone_demo")
        defaults = MissionConfig()
        values = {}
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
            values[field.name] = float(self.get_parameter(field.name).value)
        self.config = MissionConfig(**values)
        self.mission = Mission(self.config)
        self.backend = GazeboVelocityBackend(self)
        self.telemetry = self.create_publisher(String, "/swarm/telemetry", 10)
        self.create_subscription(Odometry, "/drone_001/odometry", self.on_odometry,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/swarm/command", self.on_command, 10)
        self.position = None
        self.measured_velocity = Vec3()
        self.last_feedback_wall = None
        self.last_feedback_stamp = -1.0
        self.created_wall = time.monotonic()
        self.last_clock_wall = self.created_wall
        self.last_sim_time = 0.0
        self.last_telemetry_wall = self.created_wall
        self.last_rate_wall = self.created_wall
        self.control_ticks = 0
        self.measured_update_rate = 0.0
        self.last_state = None
        self.auto_started = False
        # Steady-clock timer can issue zero velocity even if /clock freezes.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.config.update_rate, self.tick, clock=self.steady_clock)
        self.log_event("controller_ready", backend="gazebo_velocity", drones=1)

    def log_event(self, event, **values):
        self.get_logger().info(json.dumps({"event": event, **values}, allow_nan=False))

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def fail(self, reason):
        if self.mission.state != State.ERROR:
            self.get_logger().error(json.dumps({"event": "controller_error", "reason": reason}))
            self.mission.fail(reason)
        self.backend.stop()

    def on_odometry(self, message):
        try:
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            v = message.twist.twist.linear
            position = Vec3(p.x, p.y, p.z)
            velocity = Vec3(v.x, v.y, v.z)
            orientation = (q.x, q.y, q.z, q.w)
            world_to_body(Vec3(), orientation)  # Validate before storing feedback.
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if not math.isfinite(stamp) or stamp < self.last_feedback_stamp:
                raise ValueError("Gazebo odometry timestamp moved backwards")
            if message.header.frame_id != "world":
                raise ValueError(f"Expected world-frame odometry, got {message.header.frame_id!r}")
            if stamp > self.last_feedback_stamp:
                self.last_feedback_wall = time.monotonic()
                self.last_feedback_stamp = stamp
            self.position = position
            self.measured_velocity = velocity
            self.backend.orientation = orientation
        except (ValueError, OverflowError) as error:
            self.fail(f"Invalid Gazebo feedback: {error}")

    def on_command(self, message):
        command = message.data.strip().lower()
        if command == "stop":
            self.mission.command("stop", self.position, self.sim_time())
            self.auto_started = True
            self.backend.stop()
            return
        if self.position is None or self.last_feedback_wall is None:
            self.get_logger().warning("Command rejected: waiting for Gazebo odometry")
            return
        if time.monotonic() - self.last_feedback_wall > self.config.feedback_timeout:
            self.get_logger().warning("Command rejected: Gazebo odometry is stale")
            return
        try:
            self.mission.command(command, self.position, self.sim_time())
            self.auto_started = True
            if command == "pause":
                self.backend.stop()
            self.log_event("command_accepted", command=command)
        except ValueError as error:
            self.get_logger().warning(str(error))

    def tick(self):
        wall = time.monotonic()
        sim = self.sim_time()
        progressed = sim > self.last_sim_time
        if sim < self.last_sim_time:
            self.fail("Simulation clock moved backwards; restart the controller")
        if progressed:
            self.last_clock_wall = wall
        self.last_sim_time = sim
        fresh = self.last_feedback_wall is not None and wall - self.last_feedback_wall < self.config.feedback_timeout
        if not fresh:
            self.backend.stop()
            if self.last_feedback_wall is not None:
                self.fail("Gazebo odometry stopped updating")
            elif wall - self.created_wall > 30:
                self.fail("No Gazebo odometry after 30 seconds; check spawning and the ROS bridge")
        elif wall - self.last_clock_wall > self.config.feedback_timeout:
            self.fail("Simulation clock stopped; unpause Gazebo and restart the controller")
        elif sim > 0:
            if not self.auto_started and self.mission.state == State.IDLE and self.backend.publisher.get_subscription_count() > 0:
                self.mission.start(self.position, sim)
                self.auto_started = True
            if progressed:
                self.backend.send_velocity(self.mission.update(self.position, sim))
                self.control_ticks += 1
            elif self.mission.state in (State.ERROR, State.PAUSED, State.STOPPED, State.COMPLETE):
                self.backend.stop()
        if self.mission.state != self.last_state:
            self.log_event("state_changed", state=self.mission.state.value, reason=self.mission.reason)
            self.last_state = self.mission.state
        if wall - self.last_rate_wall >= 1:
            self.measured_update_rate = self.control_ticks / (wall - self.last_rate_wall)
            self.control_ticks = 0
            self.last_rate_wall = wall
        if wall - self.last_telemetry_wall >= 0.2:
            message = String()
            message.data = json.dumps({
                "backend": "gazebo_velocity", "drones": 1, "active": int(fresh),
                "state": self.mission.state.value, "reason": self.mission.reason,
                "simulation_time_s": round(sim, 3),
                "position_m": None if self.position is None else vars(self.position),
                "speed_m_s": round(self.measured_velocity.norm, 3),
                "transition": round(self.mission.progress, 4),
                "control_update_hz": round(self.measured_update_rate, 1),
                "feedback_age_s": None if self.last_feedback_wall is None else round(wall - self.last_feedback_wall, 3),
                "physics": "velocity-controlled; gravity disabled",
            }, allow_nan=False)
            self.telemetry.publish(message)
            self.last_telemetry_wall = wall


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = DroneDemo()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f"Drone controller failed: {error}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            if rclpy.ok():
                node.backend.stop()
                time.sleep(0.1)  # Give the ROS transport time to deliver the stop.
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
