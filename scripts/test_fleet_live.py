#!/usr/bin/env python3
"""Verify the fresh auto-started fleet using actual Gazebo positions in ROS state."""

import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FleetAcceptance(Node):
    def __init__(self):
        super().__init__("fleet_live_acceptance")
        self.declare_parameter("count", 5)
        self.expected_count = self.get_parameter("count").value
        self.message = None
        self.states = set()
        self.minimum_spacing = math.inf
        self.last_wall = None
        self.first_positions = None
        self.movement = 0.0
        self.command = self.create_publisher(String, "/swarm/command", 10)
        self.create_subscription(String, "/swarm/state", self.on_state, 10)

    def on_state(self, message):
        data = json.loads(message.data)
        if data.get("state") == "ERROR":
            raise AssertionError(f"Fleet entered ERROR: {data.get('error', data)}")
        self.states.add(data.get("state"))
        if data.get("count") != self.expected_count:
            raise AssertionError(f"Expected {self.expected_count} drones, got {data.get('count')}")
        actual = data.get("actual_positions")
        if actual is None:
            return
        positions = np.asarray(actual, dtype=float)
        if positions.shape != (self.expected_count, 3) or not np.isfinite(positions).all():
            raise AssertionError("Actual Gazebo pose batch is missing drones or contains invalid coordinates")
        if data.get("positions") != actual:
            raise AssertionError("State positions must contain measured Gazebo positions")
        desired = np.asarray(data.get("desired_positions"), dtype=float)
        if desired.shape != positions.shape or not np.isfinite(desired).all():
            raise AssertionError("Desired reference poses must be separate, finite coordinates")
        for field in ("simulation_time", "progress", "actual_speed_m_s", "control_update_hz"):
            if not math.isfinite(float(data[field])):
                raise AssertionError(f"Non-finite fleet telemetry: {field}")
        if self.first_positions is None:
            self.first_positions = positions.copy()
        self.movement = max(self.movement, float(np.linalg.norm(positions - self.first_positions, axis=1).min()))
        if self.expected_count > 1:
            distances = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
            np.fill_diagonal(distances, np.inf)
            separation = float(distances.min())
            self.minimum_spacing = min(self.minimum_spacing, separation)
            if separation < 1.35:
                raise AssertionError(f"Measured fleet separation fell to {separation:.3f} m (configured minimum 1.5 m)")
        self.message = data
        self.last_wall = time.monotonic()

    def run(self):
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_wall is not None and time.monotonic() - self.last_wall > 3:
                raise AssertionError("Measured Gazebo fleet feedback stopped updating")
            if self.message is None:
                continue
            data = self.message
            if data["state"] in {"FORMATION_COMPLETE", "ANIMATING"}:
                if not data.get("connected") or data.get("actual_count") != self.expected_count:
                    raise AssertionError("Completed fleet is not fully connected")
                if self.movement < 0.5:
                    raise AssertionError("No continuous measured fleet movement was observed; use a fresh world")
                error = float(data["tracking_error_m"])
                if error > 0.3:
                    continue  # Let Gazebo settle behind the final reference.
                if data["progress"] < 0.99:
                    raise AssertionError("Completed fleet reports incomplete progress")
                actual = np.asarray(data["actual_positions"])
                target = np.asarray(data["targets"])
                final_error = float(np.linalg.norm(actual - target, axis=1).max())
                if final_error > 0.3:
                    raise AssertionError(f"Measured formation is {final_error:.3f} m from assigned targets")
                print(
                    f"PASS: count={self.expected_count}, measured_motion>={self.movement:.3f}m, "
                    f"minimum_spacing={self.minimum_spacing:.3f}m, final_error={final_error:.3f}m, "
                    f"states={','.join(sorted(self.states))}", flush=True,
                )
                return
        raise TimeoutError(f"Fleet did not complete within 100 wall seconds; states={self.states}")

    def stop(self):
        command = String()
        command.data = "stop"
        for _ in range(3):
            if rclpy.ok():
                self.command.publish(command)
                time.sleep(0.08)


def main():
    rclpy.init()
    node = None
    result = 1
    try:
        node = FleetAcceptance()
        node.run()
        result = 0
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
