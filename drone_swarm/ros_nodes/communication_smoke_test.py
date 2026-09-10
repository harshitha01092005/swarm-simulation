"""Verify real ROS command delivery using Gazebo's clock and odometry feedback.

Run against a fresh, unpaused Phase 1 world with ``auto_demo:=false``. This
executable intentionally fails when ROS, the bridge, or the simulator is absent;
it is an integration test, not a mock of the communication path.
"""

from collections import deque
from dataclasses import dataclass
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    position: tuple[float, float, float]
    speed: float


class CommunicationSmokeTest(Node):
    """Exercise vertical movement, horizontal movement, and holding position."""

    def __init__(self) -> None:
        super().__init__("communication_smoke_test")
        self.declare_parameter("readiness_timeout", 60.0)
        self.declare_parameter("phase_timeout", 30.0)
        self.readiness_timeout = self._positive_parameter("readiness_timeout")
        self.phase_timeout = self._positive_parameter("phase_timeout")
        self.command = self.create_publisher(Twist, "/drone_001/cmd_vel", 10)
        self.create_subscription(
            Odometry, "/drone_001/odometry", self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.samples: deque[PoseSample] = deque(maxlen=4096)
        self.clock_time: float | None = None
        self.first_clock: float | None = None
        self.first_odom_stamp: float | None = None
        self.last_clock_wall = 0.0
        self.last_odom_wall = 0.0
        self.feedback_error: str | None = None

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be a finite positive number")
        return value

    def _on_clock(self, message: Clock) -> None:
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if self.clock_time is not None and stamp < self.clock_time:
            self.feedback_error = "Gazebo clock moved backwards during the test"
        if self.first_clock is None:
            self.first_clock = stamp
        if self.clock_time is None or stamp > self.clock_time:
            self.last_clock_wall = time.monotonic()
        self.clock_time = stamp

    def _on_odometry(self, message: Odometry) -> None:
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        v = message.twist.twist.linear
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z)
        if not all(math.isfinite(value) for value in values):
            self.feedback_error = "Gazebo odometry contains non-finite values"
            return
        if message.header.frame_id != "world":
            self.feedback_error = (
                f"Expected world-frame odometry, got {message.header.frame_id!r}"
            )
            return
        if abs(q.x) > 0.02 or abs(q.y) > 0.02 or abs(q.z) > 0.02 or abs(abs(q.w) - 1) > 0.02:
            self.feedback_error = "Drone orientation changed without an angular command"
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.samples and stamp <= self.samples[-1].stamp:
            return
        if self.first_odom_stamp is None:
            self.first_odom_stamp = stamp
        self.samples.append(PoseSample(stamp, (p.x, p.y, p.z), math.hypot(v.x, v.y, v.z)))
        self.last_odom_wall = time.monotonic()

    def _spin(self) -> None:
        if not rclpy.ok():
            raise RuntimeError("ROS shut down before the integration test completed")
        rclpy.spin_once(self, timeout_sec=0.02)
        if self.feedback_error:
            raise AssertionError(self.feedback_error)

    def _fresh(self) -> bool:
        now = time.monotonic()
        return (
            bool(self.samples)
            and self.clock_time is not None
            and now - self.last_clock_wall < 2.0
            and now - self.last_odom_wall < 2.0
            and abs(self.clock_time - self.samples[-1].stamp) < 0.5
        )

    def _ready(self) -> None:
        deadline = time.monotonic() + self.readiness_timeout
        self.get_logger().info("Waiting for command subscriber, advancing /clock, and fresh odometry")
        while time.monotonic() < deadline:
            self._spin()
            self.command.publish(Twist())
            if (
                self.command.get_subscription_count() > 0
                and self.count_publishers("/drone_001/odometry") > 0
                and self._fresh()
                and self.first_clock is not None
                and self.clock_time > self.first_clock + 0.1
                and self.first_odom_stamp is not None
                and self.samples[-1].stamp > self.first_odom_stamp + 0.1
            ):
                self.get_logger().info("ROS bridge and Gazebo feedback are ready")
                return
        raise TimeoutError(
            "Simulation was not ready: "
            f"command_subscribers={self.command.get_subscription_count()}, "
            f"odometry_samples={len(self.samples)}, clock={self.clock_time}. "
            "Check launch.log, spawn status, bridge topics, and that Gazebo is unpaused."
        )

    def _phase(self, velocity: tuple[float, float, float], duration: float) -> list[PoseSample]:
        """Publish at 20 Hz wall time; measure movement using odometry's sim time."""
        if not self.samples:
            raise RuntimeError("Cannot command movement before receiving odometry")
        start_stamp = self.samples[-1].stamp
        deadline = time.monotonic() + self.phase_timeout
        next_publish = 0.0
        command = Twist()
        command.linear.x, command.linear.y, command.linear.z = velocity
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                self.command.publish(command)
                next_publish = time.monotonic() + 0.05
            self._spin()
            if not self._fresh():
                raise AssertionError("Gazebo clock or odometry became stale during movement")
            if self.samples[-1].stamp - start_stamp >= duration:
                return [sample for sample in self.samples if sample.stamp > start_stamp]
        raise TimeoutError(
            f"Gazebo failed to advance {duration:.2f} simulation seconds within "
            f"{self.phase_timeout:.1f} wall seconds"
        )

    @staticmethod
    def _check_movement(
        baseline: PoseSample, samples: list[PoseSample], axis: int, expected: float,
    ) -> None:
        if len(samples) < 5:
            raise AssertionError("Too few intermediate odometry samples to verify continuous movement")
        displacement = samples[-1].position[axis] - baseline.position[axis]
        if not expected * 0.70 <= displacement <= expected * 1.30:
            raise AssertionError(
                f"Axis {axis} displacement {displacement:.3f} m is outside "
                f"[{expected * 0.70:.3f}, {expected * 1.30:.3f}] m"
            )
        middle = [
            sample for sample in samples
            if expected * 0.20 < sample.position[axis] - baseline.position[axis] < expected * 0.80
        ]
        if len(middle) < 3:
            raise AssertionError("Movement lacks intermediate poses; continuous travel was not observed")
        previous = baseline
        for sample in samples:
            delta_time = sample.stamp - previous.stamp
            delta = sample.position[axis] - previous.position[axis]
            if delta < -0.02 or delta > 0.7 * delta_time + 0.03:
                raise AssertionError("Observed a backwards step or implausible position jump")
            if any(
                abs(sample.position[other] - baseline.position[other]) > 0.06
                for other in range(3) if other != axis
            ):
                raise AssertionError("Drone moved along an axis that was not commanded")
            previous = sample

    def _check_hold(self) -> None:
        self._phase((0.0, 0.0, 0.0), 0.4)
        baseline = self.samples[-1]
        held = self._phase((0.0, 0.0, 0.0), 0.8)
        drift = max(math.dist(sample.position, baseline.position) for sample in held)
        if drift > 0.04 or held[-1].speed > 0.03:
            raise AssertionError(
                f"Zero velocity did not hold position: drift={drift:.3f} m, "
                f"speed={held[-1].speed:.3f} m/s"
            )

    def run(self) -> None:
        self._ready()
        start = self.samples[-1]
        if math.dist(start.position, (0.0, 0.0, 0.5)) > 0.12:
            raise AssertionError(
                f"Expected a fresh world with drone at (0, 0, 0.5), got {start.position}. "
                "Disable auto_demo and do not run other command publishers during this test."
            )
        self.get_logger().info("Commanding vertical motion: +0.4 m/s for 2 simulation seconds")
        vertical = self._phase((0.0, 0.0, 0.4), 2.0)
        self._check_movement(start, vertical, axis=2, expected=0.8)
        self._check_hold()
        horizontal_start = self.samples[-1]
        self.get_logger().info("Commanding horizontal motion: +0.3 m/s for 2 simulation seconds")
        horizontal = self._phase((0.3, 0.0, 0.0), 2.0)
        self._check_movement(horizontal_start, horizontal, axis=0, expected=0.6)
        self._check_hold()
        self.get_logger().info(
            "PASS: ROS Twist commands produced continuous Gazebo movement and stopped. "
            f"Vertical={vertical[-1].position[2] - start.position[2]:.3f} m, "
            f"horizontal={horizontal[-1].position[0] - horizontal_start.position[0]:.3f} m"
        )

    def send_stop(self) -> None:
        """Best-effort repeated stop even when feedback has failed."""
        for _ in range(5):
            if not rclpy.ok():
                break
            self.command.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = None
    result = 1
    try:
        node = CommunicationSmokeTest()
        node.run()
        result = 0
    except KeyboardInterrupt:
        result = 130
    except Exception as error:
        if node is not None:
            node.get_logger().error(f"FAIL: {type(error).__name__}: {error}")
        else:
            print(f"FAIL: {type(error).__name__}: {error}", flush=True)
    finally:
        if node is not None:
            try:
                node.send_stop()
            except Exception as error:
                node.get_logger().error(f"Unable to send final stop command: {error}")
                result = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
