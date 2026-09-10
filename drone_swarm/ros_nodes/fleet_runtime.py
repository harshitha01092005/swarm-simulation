"""One ROS node connecting the NumPy fleet controller to measured Gazebo poses."""

from dataclasses import dataclass, fields
import base64
import binascii
from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from scipy.spatial.distance import pdist
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from drone_swarm.swarm.fleet import FleetController
from drone_swarm.utils.fleet_config import FleetConfig


@dataclass(frozen=True)
class PreparedImage:
    """Worker output committed only by the ROS executor."""

    command: dict
    key: tuple
    data: bytes = b""
    options: dict | None = None
    result: object = None
    preview: str = ""
    error: str | None = None


class FleetRuntime(Node):
    """Own command dispatch and bridge transport; formation logic stays in core."""

    def __init__(self):
        super().__init__("fleet_runtime")
        share = Path(get_package_share_directory("drone_swarm_simulator"))
        self.declare_parameter("config_file", str(share / "config" / "swarm.json"))
        config_file = self.get_parameter("config_file").value
        defaults = FleetConfig()
        if config_file:
            with Path(config_file).expanduser().open(encoding="utf-8") as source:
                configured = json.load(source)
            if not isinstance(configured, dict):
                raise ValueError("config_file must contain a JSON object of fleet settings")
            defaults = defaults.updated(**configured)
        values = {}
        for field in fields(defaults):
            default = getattr(defaults, field.name)
            if field.name == "color":
                default = [float(value) for value in default]
            self.declare_parameter(field.name, default)
            values[field.name] = self.get_parameter(field.name).value
        self.declare_parameter("auto_start", True)
        self.declare_parameter("capacity", values["count"])
        self.declare_parameter("web_enabled", False)
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8765)
        self.auto_start = self.get_parameter("auto_start").value
        self.controller = FleetController(FleetConfig(**values))
        # OpenCV's initial extension import can hold the GIL. Complete that
        # import before starting ROS subscriptions or the control timer.
        from drone_swarm.image_processing import process_image
        self._process_image = process_image
        self.capacity = self.get_parameter("capacity").value
        if isinstance(self.capacity, bool) or not isinstance(self.capacity, int) or not self.controller.config.count <= self.capacity <= 400:
            raise ValueError("capacity must be an integer from active count through 400")
        self.pool_identifiers = {f"drone_{index + 1:03d}" for index in range(self.capacity)}
        self.sim_clock = None
        self.last_step_clock = None
        self.last_clock_wall = time.monotonic()
        self.created_wall = time.monotonic()
        self.last_state_wall = 0.0
        self.last_rate_wall = time.monotonic()
        self.tick_count = 0
        self.update_hz = 0.0
        self.pose_rate_count = 0
        self.pose_update_hz = 0.0
        self.pose_sequence = 0
        self.rate_sim_clock = None
        self.real_time_factor = 0.0
        self.started = False
        self.runtime_error = None
        self.last_command = None
        self.reset_feedback()
        self.last_colors_wall = 0.0
        self._queue = Queue(maxsize=32)
        self._image_ready = Queue(maxsize=2)
        self._image_pending = set()
        self._image_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="swarm-image")
        self._prefer_image = True
        self._queue_lock = threading.Lock()
        self._snapshot_lock = threading.Lock()
        self._cached_json = "{}"
        self._closing = False
        self.http_server = None
        self.reference = self.create_publisher(TFMessage, "/swarm/reference", 1)
        self.color_reference = self.create_publisher(TFMessage, "/swarm/colors", 1)
        self.state_publisher = self.create_publisher(String, "/swarm/state", 10)
        self.create_subscription(TFMessage, "/swarm/poses", self.on_poses, qos_profile_sensor_data)
        self.create_subscription(ClockMessage, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(String, "/swarm/command", self.on_command, 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / 30, self.tick, clock=self.steady_clock)
        self.publish_state()
        if self.get_parameter("web_enabled").value:
            from drone_swarm.gui.server import SwarmHTTPServer
            host = self.get_parameter("web_host").value
            port = self.get_parameter("web_port").value
            if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
                raise ValueError("web_port must be an integer from 1 to 65535")
            self.http_server = SwarmHTTPServer(
                (host, port), self.cached_state, self.submit,
                sample_path=share / "assets" / "sample.png",
            )
            self.http_server.start()
            self.get_logger().info(f"Swarm interface listening on {host}:{port}")
        self.get_logger().info(f"Fleet runtime ready: capacity={self.capacity}, auto_start={self.auto_start}")

    def reset_feedback(self):
        """Invalidate the old layout before resetting or changing active count."""
        count = self.controller.config.count
        self.identifiers = [f"drone_{index + 1:03d}" for index in range(count)]
        self.index = {name: index for index, name in enumerate(self.identifiers)}
        self.actual = np.full((count, 3), np.nan)
        self.actual_stamp = None
        self.actual_stamp_ns = None
        self.last_feedback_wall = None
        self.actual_speed = 0.0
        self.actual_max_speed = 0.0
        self.actual_peak_speed = 0.0
        self.speed_valid = False
        self.actual_min_separation = None
        self.tracking_error_since = None
        self.layout_pending = True
        self.feedback_grace_until = time.monotonic() + 2.0
        self.last_step_clock = self.sim_clock

    def cached_state(self):
        """Return an independent snapshot without touching controller state."""
        with self._snapshot_lock:
            encoded = self._cached_json
        snapshot = json.loads(encoded)
        age = max(0.0, time.time() - snapshot.get("snapshot_time", time.time()))
        snapshot["snapshot_age_s"] = age
        if age >= 2.0:
            snapshot["connected"] = False
        return snapshot

    def submit(self, command):
        """HTTP threads enqueue immutable command copies; ROS owns mutations."""
        future = Future()
        try:
            immutable = json.loads(json.dumps(command, allow_nan=False))
            if isinstance(immutable, dict) and immutable.get("action") == "image":
                config = self.cached_state()["config"]
                key = (config["count"], config["size"], config["altitude"])
                with self._queue_lock:
                    if self._closing:
                        raise RuntimeError("Fleet runtime is shutting down")
                    if len(self._image_pending) >= 2:
                        raise RuntimeError("Two images are already pending; wait for processing to finish")
                    self._image_pending.add(future)
                    try:
                        self._image_executor.submit(self.prepare_image, immutable, key, future)
                    except RuntimeError:
                        self._image_pending.discard(future)
                        raise
                return future
            with self._queue_lock:
                if self._closing:
                    raise RuntimeError("Fleet runtime is shutting down")
                self._queue.put_nowait((immutable, future))
        except Full:
            future.set_exception(RuntimeError("Command queue is full; wait for pending commands"))
        except (ValueError, TypeError, RuntimeError) as error:
            future.set_exception(error)
        return future

    def prepare_image(self, command, key, future):
        """Decode and process bytes off-thread without accessing the controller."""
        if future.cancelled():
            self.release_image(future)
            return
        try:
            encoded = command.get("data", "")
            if not isinstance(encoded, str) or len(encoded) > 12 * 1024 * 1024:
                raise ValueError("Image payload is too large")
            data = base64.b64decode(encoded, validate=True)
            if not data or len(data) > 8 * 1024 * 1024:
                raise ValueError("Image must contain between 1 byte and 8 MiB")
            options = {
                "mode": command.get("mode", "edges"),
                "threshold": command.get("threshold", 127),
                "invert": command.get("invert", False),
            }
            result = self._process_image(data, *key, **options)
            result.points.setflags(write=False)
            result.colors.setflags(write=False)
            prepared = PreparedImage(
                command, key, data, options, result,
                base64.b64encode(result.preview_png).decode("ascii"),
            )
        except Exception as error:
            prepared = PreparedImage(command, key, error=f"{type(error).__name__}: {error}")
        with self._queue_lock:
            if self._closing or future.cancelled():
                self._image_pending.discard(future)
                return
            # There can be at most two admitted images, including prepared
            # results awaiting commit. Ordinary commands cannot fill this queue.
            self._image_ready.put_nowait((prepared, future))

    def release_image(self, future):
        with self._queue_lock:
            self._image_pending.discard(future)

    def next_command(self):
        queues = (self._image_ready, self._queue) if self._prefer_image else (self._queue, self._image_ready)
        for queue in queues:
            try:
                item = queue.get_nowait()
                self._prefer_image = queue is self._queue
                return item
            except Empty:
                pass
        raise Empty

    def process_commands(self):
        for _ in range(2):
            try:
                item, future = self.next_command()
            except Empty:
                return
            prepared = item if isinstance(item, PreparedImage) else None
            command = prepared.command if prepared is not None else item
            if not future.set_running_or_notify_cancel():
                if prepared is not None:
                    self.release_image(future)
                continue
            try:
                if prepared is not None and prepared.error:
                    raise ValueError(prepared.error)
                result = self.apply_command(command, prepared_image=prepared)
                self.publish_state()
                future.set_result(result)
            except Exception as error:
                # Complete the HTTP response even if diagnostic logging fails.
                future.set_exception(RuntimeError(str(error)))
                self.record_command(command, False, str(error))
            finally:
                if prepared is not None:
                    self.release_image(future)

    def on_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if self.sim_clock is None:
            self.rate_sim_clock = stamp
            self.last_rate_wall = time.monotonic()
        if self.sim_clock is not None and stamp < self.sim_clock:
            self.fail("Gazebo simulation clock moved backwards; restart the world")
        if self.sim_clock is None or stamp > self.sim_clock:
            self.last_clock_wall = time.monotonic()
        self.sim_clock = stamp

    def on_poses(self, message):
        positions = np.full_like(self.actual, np.nan)
        seen = set()
        stamp_ns = None
        for transform in message.transforms:
            identifier = transform.child_frame_id
            if identifier not in self.pool_identifiers or identifier in seen:
                self.fail(f"Unknown or duplicate Gazebo pose identity: {identifier!r}")
                return
            seen.add(identifier)
            if transform.header.frame_id != "world":
                self.fail(f"Expected world-frame poses, got {transform.header.frame_id!r}")
                return
            p = transform.transform.translation
            q = transform.transform.rotation
            if not all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                self.fail("Gazebo pose batch contains non-finite values")
                return
            seconds = transform.header.stamp.sec
            nanoseconds = transform.header.stamp.nanosec
            if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
                self.fail("Gazebo pose has an invalid simulation timestamp")
                return
            current_ns = seconds * 1_000_000_000 + nanoseconds
            if stamp_ns is not None and current_ns != stamp_ns:
                self.fail("Gazebo pose batch contains mixed simulation timestamps")
                return
            stamp_ns = current_ns
            index = self.index.get(identifier)
            if index is not None:
                positions[index] = (p.x, p.y, p.z)
        if stamp_ns is None or not np.isfinite(positions).all():
            # Ignore old, smaller batches during count changes; never mix them
            # with earlier positions to manufacture a complete observation.
            return
        if self.actual_stamp_ns is not None and stamp_ns < self.actual_stamp_ns:
            self.fail("Gazebo pose timestamps moved backwards")
            return
        if self.actual_stamp_ns is not None and stamp_ns == self.actual_stamp_ns:
            return
        stamp = stamp_ns * 1e-9
        if self.actual_stamp_ns is not None and np.isfinite(self.actual).all() and not self.layout_pending:
            speeds = np.linalg.norm(positions - self.actual, axis=1) / ((stamp_ns - self.actual_stamp_ns) * 1e-9)
            self.actual_speed = float(speeds.mean())
            self.actual_max_speed = float(speeds.max())
            self.actual_peak_speed = max(self.actual_peak_speed, self.actual_max_speed)
            self.speed_valid = True
        self.actual = positions
        self.actual_stamp = stamp
        self.actual_stamp_ns = stamp_ns
        self.actual_min_separation = float(pdist(positions).min()) if len(positions) > 1 else None
        self.last_feedback_wall = time.monotonic()
        self.pose_sequence += 1
        self.pose_rate_count += 1
        if self.layout_pending and np.linalg.norm(positions - self.controller.positions, axis=1).max() <= 0.05:
            self.layout_pending = False
            self.last_step_clock = self.sim_clock

    def connected(self):
        return (
            self.last_feedback_wall is not None
            and not self.layout_pending
            and time.monotonic() - self.last_feedback_wall < 2.0
            and np.isfinite(self.actual).all()
            and self.sim_clock is not None
            and time.monotonic() - self.last_clock_wall < 2.0
            and abs(self.sim_clock - self.actual_stamp) < 0.5
            and self.reference.get_subscription_count() > 0
        )

    def fail(self, reason):
        if self.runtime_error != reason:
            self.get_logger().error(json.dumps({"event": "fleet_runtime_error", "reason": reason}))
        self.runtime_error = reason
        self.controller.stop()

    def record_command(self, command, accepted, error=None):
        command = command if isinstance(command, dict) else {}
        request_id = command.get("request_id")
        if not isinstance(request_id, (str, int)) or isinstance(request_id, bool):
            request_id = None
        if isinstance(request_id, str):
            request_id = request_id[:128]
        self.last_command = {
            "request_id": request_id, "action": str(command.get("action", ""))[:64],
            "accepted": accepted, "error": None if error is None else error[:1024],
        }
        event = {"event": "fleet_command_accepted" if accepted else "fleet_command_rejected", **self.last_command}
        encoded = json.dumps(event)
        try:
            # rclpy caches severity by call site: keep distinct source lines.
            if accepted:
                self.get_logger().info(encoded)
            else:
                self.get_logger().warning(encoded)
        except Exception as logging_error:
            # Diagnostics must not turn an applied command into a rejection or
            # strand a future. Keep an independent fallback when ROS logging fails.
            print(f"Fleet command logging failed ({logging_error}): {encoded}", file=sys.stderr)

    def apply_command(self, command, prepared_image=None):
        """Apply a command on the ROS executor and return its acknowledgement."""
        if isinstance(command, str):
            command = {"action": command.strip().lower()}
        if not isinstance(command, dict) or not isinstance(command.get("action"), str):
            raise ValueError("Command must be an action string or JSON object with an action")
        action = command["action"]
        if not action or len(action) > 32:
            raise ValueError("Action must contain between 1 and 32 characters")
        request_id = command.get("request_id")
        if request_id is not None and (
            isinstance(request_id, bool) or not isinstance(request_id, (str, int))
            or isinstance(request_id, str) and len(request_id) > 128
        ):
            raise ValueError("request_id must be an integer or a string of at most 128 characters")
        if action in {"start", "resume", "apply", "configure", "reset"} and not self.connected():
            raise ValueError("Waiting for fresh Gazebo poses and the ROS bridge")
        if self.runtime_error and action not in {"stop", "reset"}:
            raise ValueError(f"Runtime is in ERROR: {self.runtime_error}; reset or relaunch")
        result = {"ok": True, "accepted": True, "action": action, "request_id": command.get("request_id")}
        if action in {"start", "pause", "resume", "stop", "reset"}:
            getattr(self.controller, action)()
            if action == "reset":
                self.runtime_error = None
                self.reset_feedback()
            self.started = True
        elif action == "apply":
            self.controller.apply_formation(command["formation"])
        elif action == "configure":
            settings = command.get("settings", {})
            if not isinstance(settings, dict):
                raise ValueError("configure.settings must be an object")
            candidate = self.controller.config.updated(**settings)
            if candidate.count > self.capacity:
                raise ValueError(f"This world supports up to {self.capacity} drones; relaunch with greater capacity")
            count_changed = candidate.count != self.controller.config.count
            layout_changed = count_changed or candidate.min_distance != self.controller.config.min_distance
            if layout_changed and self.controller.state not in {"IDLE", "STOPPED"}:
                raise ValueError("Stop the simulation before changing drone count or minimum separation")
            self.controller.configure(**settings)
            if layout_changed:
                self.reset_feedback()
                self.started = True
            current = self.controller.snapshot()
            result["config"] = current["config"]
            result["pending_config"] = current.get("pending_config")
        elif action == "image":
            if prepared_image is None:
                raise ValueError("Submit image commands through the asynchronous image queue")
            image_result = self.controller.set_processed_image(
                prepared_image.data, prepared_image.options,
                prepared_image.result, prepared_image.key,
            )
            result["image"] = image_result.metadata
            result["preview"] = prepared_image.preview
        else:
            raise ValueError(f"Unknown action: {action!r}")
        self.record_command(command, True)
        return result

    def on_command(self, message):
        command = {"action": message.data.strip().lower()}
        try:
            try:
                command = json.loads(message.data)
            except json.JSONDecodeError:
                pass
            if isinstance(command, str):
                command = {"action": command.strip().lower()}
            if isinstance(command, dict) and command.get("action") == "image":
                future = self.submit(command)
                if future.done() and future.exception() is not None:
                    raise future.exception()
            else:
                self.apply_command(command)
        except (ValueError, KeyError, TypeError, RuntimeError, binascii.Error) as error:
            self.record_command(command, False, str(error))

    def publish_reference(self):
        message = TFMessage()
        stamp = self.get_clock().now().to_msg()
        for identifier, position in zip(self.identifiers, self.controller.positions):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "world"
            transform.child_frame_id = identifier
            transform.transform.translation.x = float(position[0])
            transform.transform.translation.y = float(position[1])
            transform.transform.translation.z = float(position[2])
            transform.transform.rotation.w = 1.0
            message.transforms.append(transform)
        self.reference.publish(message)

    def publish_colors(self):
        colors = np.asarray(self.controller.colors)
        if colors.shape != (len(self.identifiers), 3) or not np.isfinite(colors).all() or (colors < 0).any() or (colors > 1).any():
            raise ValueError("Controller colors must contain one finite RGB triple per active drone")
        message = TFMessage()
        stamp = self.get_clock().now().to_msg()
        for identifier, color in zip(self.identifiers, colors):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "world"
            transform.child_frame_id = identifier
            transform.transform.translation.x = float(color[0])
            transform.transform.translation.y = float(color[1])
            transform.transform.translation.z = float(color[2])
            transform.transform.rotation.w = 1.0
            message.transforms.append(transform)
        self.color_reference.publish(message)

    def publish_state(self):
        snapshot = self.controller.snapshot()
        snapshot["desired_positions"] = snapshot.pop("positions")
        snapshot["desired_speed_m_s"] = snapshot.pop("speed_m_s", None)
        snapshot["desired_min_separation_m"] = snapshot.pop("min_separation_m", None)
        actual_valid = np.isfinite(self.actual).all()
        observed = self.actual.tolist() if actual_valid else None
        snapshot["positions"] = observed
        snapshot["actual_positions"] = observed
        snapshot["actual_count"] = int(np.isfinite(self.actual).all(axis=1).sum())
        snapshot["connected"] = bool(self.connected())
        snapshot["backend"] = "gazebo_fleet"
        snapshot["capacity"] = self.capacity
        snapshot["snapshot_time"] = time.time()
        snapshot["layout_pending"] = self.layout_pending
        snapshot["last_command"] = self.last_command
        with self._queue_lock:
            snapshot["image_processing"] = bool(self._image_pending)
        snapshot["colors_source"] = "controller"
        snapshot["feedback_age_s"] = None if self.last_feedback_wall is None else time.monotonic() - self.last_feedback_wall
        snapshot["actual_speed_m_s"] = self.actual_speed
        snapshot["actual_max_speed_m_s"] = self.actual_max_speed
        snapshot["actual_peak_speed_m_s"] = self.actual_peak_speed
        snapshot["actual_speed_valid"] = self.speed_valid
        snapshot["speed_m_s"] = self.actual_speed if self.speed_valid else None
        snapshot["pose_sequence"] = self.pose_sequence
        snapshot["actual_simulation_time"] = self.actual_stamp
        snapshot["gazebo_simulation_time"] = self.sim_clock
        snapshot["control_update_hz"] = self.update_hz
        snapshot["pose_update_hz"] = self.pose_update_hz
        snapshot["real_time_factor"] = self.real_time_factor
        snapshot["tracking_error_m"] = None
        snapshot["actual_min_separation_m"] = None
        if actual_valid:
            snapshot["tracking_error_m"] = float(np.linalg.norm(self.actual - self.controller.positions, axis=1).max())
            snapshot["actual_min_separation_m"] = self.actual_min_separation
        snapshot["min_separation_m"] = snapshot["actual_min_separation_m"]
        if self.runtime_error:
            snapshot["controller_state"] = snapshot["state"]
            snapshot["state"] = "ERROR"
            snapshot["error"] = self.runtime_error
        message = String()
        message.data = json.dumps(snapshot, allow_nan=False)
        with self._snapshot_lock:
            self._cached_json = message.data
        self.state_publisher.publish(message)

    def tick(self):
        wall = time.monotonic()
        try:
            self.process_commands()
            connected = self.connected()
            if self.started and not connected and wall >= self.feedback_grace_until and self.runtime_error is None:
                self.fail("Gazebo fleet feedback or simulation clock stopped updating")
            if not self.started and wall - self.created_wall > 45 and not connected:
                self.fail("Fleet did not connect within 45 seconds; inspect Gazebo and bridge logs")
            if self.auto_start and not self.started and connected and self.runtime_error is None:
                self.controller.start()
                self.started = True
            if self.sim_clock is not None:
                if self.last_step_clock is not None and connected and self.runtime_error is None:
                    dt = self.sim_clock - self.last_step_clock
                    if dt > 0.5:
                        self.fail("Controller fell more than 0.5 simulation seconds behind")
                    elif dt > 0:
                        steps = max(1, math.ceil(dt / (1 / 30)))
                        for _ in range(steps):
                            self.controller.step(dt / steps)
                        self.tick_count += 1
                self.last_step_clock = self.sim_clock
            self.publish_reference()
            self.check_tracking(wall)
            if wall - self.last_colors_wall >= 0.1:
                self.publish_colors()
                self.last_colors_wall = wall
            if wall - self.last_rate_wall >= 1:
                elapsed = wall - self.last_rate_wall
                self.update_hz = self.tick_count / elapsed
                self.pose_update_hz = self.pose_rate_count / elapsed
                if self.sim_clock is not None and self.rate_sim_clock is not None:
                    self.real_time_factor = max(0.0, (self.sim_clock - self.rate_sim_clock) / elapsed)
                self.rate_sim_clock = self.sim_clock
                self.tick_count = 0
                self.pose_rate_count = 0
                self.last_rate_wall = wall
            if wall - self.last_state_wall >= 0.1:
                self.publish_state()
                self.last_state_wall = wall
        except (ValueError, TypeError, RuntimeError, FloatingPointError) as error:
            self.fail(f"Controller tick failed: {error}")

    def check_tracking(self, wall):
        """Detect a live transport whose Gazebo models stopped following commands."""
        if not self.connected() or self.layout_pending or wall < self.feedback_grace_until:
            self.tracking_error_since = None
            return
        if (
            self.controller.config.collision_avoidance
            and self.actual_min_separation is not None
            and self.actual_min_separation < self.controller.config.min_distance - 1e-5
        ):
            self.fail(
                f"Measured drone separation {self.actual_min_separation:.5f} m is below "
                f"the configured {self.controller.config.min_distance:.5f} m minimum"
            )
            return
        active = self.controller.state in {
            "TAKEOFF", "TRANSITIONING", "FORMATION_READY", "FORMATION_COMPLETE", "ANIMATING",
        }
        error = float(np.linalg.norm(self.actual - self.controller.positions, axis=1).max())
        tolerance = max(0.5, self.controller.config.speed * 0.2)
        if active and error > tolerance:
            if self.tracking_error_since is None:
                self.tracking_error_since = wall
            elif wall - self.tracking_error_since >= 1.0:
                self.fail(f"Fresh Gazebo poses failed to track references: error={error:.3f} m for over 1 second")
        else:
            self.tracking_error_since = None

    def close(self):
        with self._queue_lock:
            self._closing = True
            pending = set(self._image_pending)
            self._image_pending.clear()
            for queue in (self._queue, self._image_ready):
                while True:
                    try:
                        _, future = queue.get_nowait()
                    except Empty:
                        break
                    pending.add(future)
        for future in pending:
            if not future.done() and future.set_running_or_notify_cancel():
                future.set_exception(RuntimeError("Fleet runtime shut down before applying the command"))
        # At most one bounded 8 MiB / 16 MP operation may finish in the
        # background; it cannot commit after _closing is set. Cancel queued work.
        self._image_executor.shutdown(wait=False, cancel_futures=True)
        if self.http_server is not None:
            self.http_server.shutdown()
            self.http_server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = FleetRuntime()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f"Fleet runtime failed: {error}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            node.close()
            node.controller.stop()
            if rclpy.ok():
                node.publish_reference()
                time.sleep(0.08)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
