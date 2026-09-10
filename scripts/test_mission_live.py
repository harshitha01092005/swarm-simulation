#!/usr/bin/env python3
"""Accept the automatic one-drone demo against real ROS/Gazebo feedback.

Start a fresh simulation with auto_demo:=true, then run this script promptly.
It commands only /swarm/command; the controller owns drone velocity commands.
"""

from collections import deque
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class MissionAcceptance(Node):
    def __init__(self):
        super().__init__("mission_live_acceptance")
        self.deadline = time.monotonic() + 90.0
        self.command = self.create_publisher(String, "/swarm/command", 10)
        self.create_subscription(String, "/swarm/telemetry", self.on_telemetry, 10)
        self.create_subscription(
            Odometry, "/drone_001/odometry", self.on_odometry,
            qos_profile_sensor_data,
        )
        self.telemetry = None
        self.samples = deque(maxlen=4096)
        self.first_stamp = None
        self.last_odom_wall = None
        self.last_telemetry_wall = None
        self.states = set()
        self.first_active_state = None
        self.max_speed = 0.0

    def on_telemetry(self, message):
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise AssertionError("Telemetry must be a JSON object")

        def check_finite(value):
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise AssertionError("Telemetry contains a non-finite number")
            if isinstance(value, dict):
                for item in value.values():
                    check_finite(item)
            if isinstance(value, list):
                for item in value:
                    check_finite(item)

        check_finite(data)
        state = data.get("state")
        if state not in {"IDLE", "TAKEOFF", "MOVING", "PAUSED", "COMPLETE", "STOPPED", "ERROR"}:
            raise AssertionError(f"Invalid telemetry state: {state!r}")
        if data.get("backend") != "gazebo_velocity" or data.get("drones") != 1:
            raise AssertionError("Telemetry does not describe the one-drone Gazebo backend")
        for field in ("simulation_time_s", "speed_m_s", "transition", "control_update_hz"):
            value = data.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
                raise AssertionError(f"Invalid numeric telemetry field: {field}")
        if data["transition"] > 1:
            raise AssertionError("Telemetry transition exceeds 100%")
        if state == "ERROR":
            raise AssertionError(f"Controller entered ERROR: {data.get('reason')}")
        if state in {"TAKEOFF", "MOVING"} and self.first_active_state is None:
            self.first_active_state = state
        self.states.add(state)
        self.telemetry = data
        self.last_telemetry_wall = time.monotonic()

    def on_odometry(self, message):
        p = message.pose.pose.position
        v = message.twist.twist.linear
        q = message.pose.pose.orientation
        position = (p.x, p.y, p.z)
        values = (*position, v.x, v.y, v.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            raise AssertionError("Gazebo odometry contains a non-finite value")
        if message.header.frame_id != "world":
            raise AssertionError("Gazebo odometry must use the world frame")
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.samples and stamp < self.samples[-1][0]:
            raise AssertionError("Odometry simulation time moved backwards")
        if self.samples and stamp == self.samples[-1][0]:
            return
        speed = math.hypot(v.x, v.y, v.z)
        if self.first_stamp is None:
            self.first_stamp = stamp
        # Gazebo's odometry differentiator needs an initial pose sample. Exclude
        # its brief startup transient from the velocity bound, not from finiteness.
        if stamp - self.first_stamp > 0.3:
            self.max_speed = max(self.max_speed, speed)
            if speed > 1.8:
                raise AssertionError(f"Measured speed {speed:.3f} m/s exceeds the 1.5 m/s limit plus tolerance")
            if self.samples:
                delta = stamp - self.samples[-1][0]
                if math.dist(position, self.samples[-1][1]) > 1.8 * delta + 0.025:
                    raise AssertionError("Measured pose jumped beyond the velocity bound")
        self.samples.append((stamp, position, speed))
        self.last_odom_wall = time.monotonic()

    def spin(self):
        if time.monotonic() >= self.deadline:
            raise TimeoutError("Automatic mission acceptance exceeded 90 wall seconds")
        if not rclpy.ok():
            raise RuntimeError("ROS stopped before mission acceptance completed")
        rclpy.spin_once(self, timeout_sec=0.05)
        now = time.monotonic()
        for name, last in (("odometry", self.last_odom_wall), ("telemetry", self.last_telemetry_wall)):
            if last is not None and now - last > 3.0:
                raise AssertionError(f"{name} stopped updating")

    def wait(self, condition, description, timeout=30.0):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while time.monotonic() < deadline:
            self.spin()
            if condition():
                return
        state = None if self.telemetry is None else self.telemetry["state"]
        raise TimeoutError(f"Timed out waiting for {description}; last state={state}")

    def send(self, command):
        message = String()
        message.data = command
        self.command.publish(message)

    def hold(self, expected_state):
        start_stamp = self.samples[-1][0]
        self.wait(lambda: self.samples[-1][0] >= start_stamp + 0.35, "stop command to settle", 8)
        baseline_stamp, baseline, _ = self.samples[-1]
        self.wait(lambda: self.samples[-1][0] >= baseline_stamp + 0.8, "measured position hold", 8)
        held = [sample for sample in self.samples if sample[0] > baseline_stamp]
        drift = max(math.dist(sample[1], baseline) for sample in held)
        if drift > 0.05 or held[-1][2] > 0.05:
            raise AssertionError(f"{expected_state} did not hold: drift={drift:.3f} m, speed={held[-1][2]:.3f} m/s")
        if self.telemetry["state"] != expected_state:
            raise AssertionError(f"Controller left {expected_state} without a command")
        return drift

    def run(self):
        self.wait(
            lambda: self.telemetry is not None and len(self.samples) >= 2
            and self.command.get_subscription_count() > 0,
            "controller telemetry, command discovery, and measured odometry",
        )
        baseline = self.samples[-1][1]

        def moving():
            if self.telemetry["state"] == "COMPLETE":
                raise AssertionError("Mission completed before pause testing; rerun against a fresh world")
            return (
                self.telemetry["state"] in {"TAKEOFF", "MOVING"}
                and math.dist(self.samples[-1][1], baseline) > 0.12
                and self.samples[-1][2] > 0.05
            )

        self.wait(moving, "measured automatic movement", 20)
        self.send("pause")
        self.wait(lambda: self.telemetry["state"] == "PAUSED", "PAUSED acknowledgement", 8)
        paused_drift = self.hold("PAUSED")
        print(f"Pause verified: measured drift={paused_drift:.4f} m", flush=True)

        self.send("resume")
        self.wait(lambda: self.telemetry["state"] in {"TAKEOFF", "MOVING"}, "resume acknowledgement", 8)
        self.wait(lambda: self.telemetry["state"] == "COMPLETE", "automatic mission completion", 65)
        self.hold("COMPLETE")
        final = self.samples[-1][1]
        error = math.dist(final, (0.0, 0.0, 4.0))
        if error > 0.1:
            raise AssertionError(f"Final measured pose {final} is {error:.3f} m from (0,0,4)")
        if not {"MOVING", "PAUSED", "COMPLETE"}.issubset(self.states):
            raise AssertionError(f"Required state transitions missing: {sorted(self.states)}")
        if self.telemetry["transition"] != 1 or self.telemetry.get("active") != 1:
            raise AssertionError("Completed telemetry does not report 100% transition and an active drone")
        observation = "" if "TAKEOFF" in self.states else "; subscribed after TAKEOFF"
        print(
            f"PASS: automatic movement, pause/hold/resume, and COMPLETE; "
            f"final_error={error:.4f} m, max_speed={self.max_speed:.3f} m/s, "
            f"states={','.join(sorted(self.states))}{observation}", flush=True,
        )


def main():
    rclpy.init()
    node = None
    result = 1
    try:
        node = MissionAcceptance()
        node.run()
        result = 0
    except KeyboardInterrupt:
        result = 130
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
    finally:
        if node is not None:
            try:
                for _ in range(3):
                    if not rclpy.ok():
                        break
                    node.send("stop")
                    time.sleep(0.08)
            except Exception as error:
                print(f"Final stop failed: {error}", file=sys.stderr, flush=True)
                result = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
