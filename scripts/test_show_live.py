#!/usr/bin/env python3
"""Measure a sequence of live Gazebo formations through the public ROS contract."""

import base64
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class ShowAcceptance(Node):
    def __init__(self):
        super().__init__("show_live_acceptance")
        self.declare_parameter("count", 20)
        self.declare_parameter("formations", "circle,star,spiral,wave,infinity")
        self.declare_parameter("timeout", 180.0)
        self.declare_parameter("report_path", "")
        self.declare_parameter("image_path", "")
        self.count = self.get_parameter("count").value
        self.formations = [name.strip() for name in self.get_parameter("formations").value.split(",") if name.strip()]
        self.timeout = float(self.get_parameter("timeout").value)
        self.report_path = self.get_parameter("report_path").value
        self.image_path = self.get_parameter("image_path").value
        self.image_uploaded = False
        if not 1 <= self.count <= 400 or not self.formations:
            raise ValueError("count must be 1–400 and formations must contain at least one name")
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        if "image" in self.formations and not self.image_path:
            raise ValueError("The image formation requires an image_path parameter")
        self.created = time.monotonic()
        self.cpu_start = self.cpu_seconds()
        self.rtfs = []
        self.control_rates = []
        self.deadline = self.created + self.timeout
        self.latest = None
        self.last_state_wall = None
        self.previous_positions = None
        self.previous_sim_time = None
        self.minimum_spacing = math.inf
        self.maximum_speed = 0.0
        self.maximum_tracking_error = 0.0
        self.maximum_control_hz = 0.0
        self.pose_batches = 0
        self.first_pose_wall = None
        self.last_pose_wall = None
        self.transition_serial = 0
        self.states = set()
        self.results = []
        self.command = self.create_publisher(String, "/swarm/command", 10)
        self.create_subscription(String, "/swarm/state", self.on_state, 10)
        self.create_subscription(TFMessage, "/swarm/poses", self.on_poses, qos_profile_sensor_data)

    @staticmethod
    def cpu_seconds():
        try:
            values = dict(line.split() for line in Path('/sys/fs/cgroup/cpu.stat').read_text().splitlines())
            return int(values['usage_usec']) / 1e6
        except (OSError, KeyError, ValueError):
            return None

    def on_poses(self, message):
        if not message.transforms:
            return
        for transform in message.transforms:
            p = transform.transform.translation
            if not all(math.isfinite(value) for value in (p.x, p.y, p.z)):
                raise AssertionError("Gazebo pose transport contains a non-finite coordinate")
        now = time.monotonic()
        if self.first_pose_wall is None:
            self.first_pose_wall = now
        self.last_pose_wall = now
        self.pose_batches += 1

    def on_state(self, message):
        data = json.loads(message.data)
        state = data.get("state")
        if not isinstance(state, str):
            raise AssertionError("State must be a string")
        if state == "ERROR":
            raise AssertionError(f"Fleet entered ERROR: {data.get('error', data.get('message'))}")
        if data.get("count") != self.count:
            raise AssertionError(f"Expected {self.count} active drones, got {data.get('count')}")
        self.states.add(state)
        if state == "TRANSITIONING":
            self.transition_serial += 1
        if data.get("actual_positions") is None:
            return
        actual = np.asarray(data["actual_positions"], dtype=float)
        desired = np.asarray(data.get("desired_positions"), dtype=float)
        targets = np.asarray(data.get("targets"), dtype=float)
        for name, positions in (("actual", actual), ("desired", desired), ("targets", targets)):
            if positions.shape != (self.count, 3) or not np.isfinite(positions).all():
                raise AssertionError(f"{name} must contain {self.count} finite XYZ coordinates")
        if data.get("positions") != data["actual_positions"]:
            raise AssertionError("Public positions field must contain measured Gazebo coordinates")
        for key in ("simulation_time", "progress", "control_update_hz", "actual_speed_m_s", "tracking_error_m"):
            value = data.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise AssertionError(f"Invalid telemetry value: {key}={value!r}")
        if not 0 <= data["progress"] <= 1:
            raise AssertionError("Progress is outside [0,1]")
        tracking = float(np.linalg.norm(actual - desired, axis=1).max())
        self.maximum_tracking_error = max(self.maximum_tracking_error, tracking)
        self.maximum_control_hz = max(self.maximum_control_hz, data["control_update_hz"])
        if self.count > 1:
            distances = np.linalg.norm(actual[:, None, :] - actual[None, :, :], axis=2)
            np.fill_diagonal(distances, np.inf)
            spacing = float(distances.min())
            self.minimum_spacing = min(self.minimum_spacing, spacing)
            if spacing <= 1e-6:
                raise AssertionError("Two measured Gazebo drones occupy the same point")
        stamp = data["actual_simulation_time"]
        if data.get('real_time_factor', 0) > 0:
            self.rtfs.append(data['real_time_factor'])
            self.control_rates.append(data['control_update_hz'])
        if self.previous_sim_time is not None and stamp > self.previous_sim_time:
            delta = stamp - self.previous_sim_time
            speed = float(np.linalg.norm(actual - self.previous_positions, axis=1).max() / delta)
            self.maximum_speed = max(self.maximum_speed, speed)
            limit = float(data["config"]["speed"])
            if speed > 1.3 * limit:
                raise AssertionError(f"Measured interval speed {speed:.3f} m/s exceeds 1.3× configured limit {limit:.3f}")
        self.previous_positions = actual.copy()
        self.previous_sim_time = stamp
        self.latest = data
        self.last_state_wall = time.monotonic()

    def wait(self, predicate, description, timeout=50.0):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise RuntimeError("ROS shut down during show acceptance")
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            for name, last in (("state", self.last_state_wall), ("Gazebo poses", self.last_pose_wall)):
                if last is not None and now - last > 3:
                    raise AssertionError(f"{name} stopped updating")
            if predicate():
                return
        state = None if self.latest is None else self.latest.get("state")
        raise TimeoutError(f"Timed out waiting for {description}; last state={state}")

    def send(self, action, **parameters):
        message = String()
        message.data = json.dumps({"action": action, **parameters}, allow_nan=False)
        self.command.publish(message)

    def completed(self, formation=None):
        if self.latest is None:
            return False
        data = self.latest
        if formation is not None and data.get("formation") != formation:
            return False
        return (
            data["state"] in {"FORMATION_COMPLETE", "ANIMATING"}
            and data.get("connected")
            and data.get("actual_count") == self.count
            and data["progress"] >= 0.99
            and data["tracking_error_m"] <= 0.35
        )

    def upload_image(self):
        if self.image_uploaded:
            return
        path = Path(self.image_path).expanduser()
        limit = 8 * 1024 * 1024
        with path.open("rb") as image_file:
            contents = image_file.read(limit + 1)
        if not contents:
            raise ValueError(f"Image file is empty: {path}")
        if len(contents) > limit:
            raise ValueError(f"Image file exceeds the 8 MiB upload limit: {path}")
        self.send(
            "image", data=base64.b64encode(contents).decode("ascii"),
            mode="edges", threshold=127, invert=False,
        )
        self.wait(
            lambda: self.latest is not None and self.latest.get("image") is not None,
            "image upload acknowledgement (snapshot.image is missing; inspect fleet_command_rejected logs)",
            15,
        )
        self.image_uploaded = True
        print(f"Image upload acknowledged: {path.name}, {len(contents)} bytes", flush=True)

    def run(self):
        self.wait(lambda: self.completed() and self.command.get_subscription_count() > 0, "initial formation completion", 90)
        self.send("configure", settings={"transition_time": 3.0})
        self.wait(lambda: self.latest["config"]["transition_time"] == 3.0, "configuration acknowledgement", 8)
        for formation in self.formations:
            if formation == "image":
                self.upload_image()
            baseline = np.asarray(self.latest["actual_positions"], dtype=float)
            serial = self.transition_serial
            started = time.monotonic()
            self.send("apply", formation=formation)
            self.wait(
                lambda: self.transition_serial > serial and self.latest.get("formation") == formation,
                f"{formation} TRANSITIONING acknowledgement", 8,
            )
            self.wait(lambda: self.completed(formation), f"{formation} measured completion")
            actual = np.asarray(self.latest["actual_positions"], dtype=float)
            targets = np.asarray(self.latest["targets"], dtype=float)
            error = float(np.linalg.norm(actual - targets, axis=1).max())
            if error > 0.35:
                raise AssertionError(f"{formation}: actual positions remain {error:.3f} m from targets")
            record = {
                "formation": formation, "wall_seconds": time.monotonic() - started,
                "target_error_m": error,
                "measured_displacement_m": float(np.linalg.norm(actual - baseline, axis=1).max()),
            }
            self.results.append(record)
            print(f"PASS: {formation}, measured target error={error:.3f} m, elapsed={record['wall_seconds']:.2f} s", flush=True)

    def report(self, passed, error=None):
        rate = None
        if self.pose_batches > 1 and self.last_pose_wall > self.first_pose_wall:
            rate = (self.pose_batches - 1) / (self.last_pose_wall - self.first_pose_wall)
        result = {
            "passed": passed, "error": error, "count": self.count,
            "wall_seconds": time.monotonic() - self.created,
            "formations": self.results, "observed_states": sorted(self.states),
            "observed_minimum_separation_m": self.minimum_spacing if math.isfinite(self.minimum_spacing) else None,
            "observed_maximum_interval_speed_m_s": self.maximum_speed,
            "observed_maximum_tracking_error_m": self.maximum_tracking_error,
            "received_gazebo_pose_batches_hz": rate,
            "maximum_control_update_hz": self.maximum_control_hz,
            "mean_control_update_hz": float(np.mean(self.control_rates)) if self.control_rates else None,
            "mean_real_time_factor": float(np.mean(self.rtfs)) if self.rtfs else None,
            "visible_cpus": os.cpu_count(),
            "separation_requirement": "No exact overlap; strict configured separation is a separate collision-avoidance gate",
        }
        cpu_end = self.cpu_seconds()
        result['average_container_cpu_cores'] = None if cpu_end is None or self.cpu_start is None else (cpu_end-self.cpu_start)/(time.monotonic()-self.created)
        try:
            result['container_peak_memory_mib'] = int(Path('/sys/fs/cgroup/memory.peak').read_text()) / 1024**2
        except (OSError, ValueError):
            result['container_peak_memory_mib'] = None
        if self.report_path:
            path = Path(self.report_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(json.dumps(result, allow_nan=False), flush=True)


def main():
    rclpy.init()
    node = None
    result = 1
    failure = None
    try:
        node = ShowAcceptance()
        node.run()
        result = 0
    except KeyboardInterrupt:
        result = 130
        failure = "Interrupted"
    except Exception as error:
        failure = f"{type(error).__name__}: {error}"
        print(f"FAIL: {failure}", file=sys.stderr, flush=True)
    finally:
        if node is not None:
            try:
                for _ in range(3):
                    if rclpy.ok():
                        node.send("stop")
                        time.sleep(0.08)
                node.report(result == 0, failure)
            except Exception as error:
                print(f"Cleanup/report failed: {error}", file=sys.stderr, flush=True)
                result = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
