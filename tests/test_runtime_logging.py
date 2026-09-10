"""ROS-runtime regressions; run inside the project Docker image.

Host-only test suites explicitly skip this module when rclpy is unavailable.
No Gazebo process, fleet launch, or HTTP server is needed.
"""

from collections import deque
from concurrent.futures import Future
from contextlib import redirect_stderr
import io
from queue import Empty
from types import MethodType, SimpleNamespace
import unittest

try:
    import rclpy
except ModuleNotFoundError as error:
    if error.name != "rclpy":
        raise
    rclpy = None
else:
    from rclpy.context import Context
    from rclpy.node import Node
    from drone_swarm.ros_nodes.fleet_runtime import FleetRuntime


@unittest.skipIf(rclpy is None, "rclpy is unavailable; run this regression in the ROS Docker image")
class RuntimeLoggingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def test_actual_rclpy_accept_reject_accept_uses_separate_call_sites(self):
        node = Node("fleet_logging_regression", context=self.context)
        fallback = io.StringIO()
        try:
            with redirect_stderr(fallback):
                for accepted in (True, False, True):
                    FleetRuntime.record_command(
                        node, {"action": "configure", "request_id": "logging-regression"},
                        accepted, None if accepted else "Stop before changing drone count",
                    )
                    self.assertEqual(node.last_command["accepted"], accepted)
            # Catching and hiding the rclpy severity exception is insufficient:
            # all three calls must use the actual ROS logger successfully.
            self.assertNotIn("logging failed", fallback.getvalue())
        finally:
            node.destroy_node()

    @staticmethod
    def harness(commands):
        queued = deque((command, Future()) for command in commands)
        futures = [future for _, future in queued]

        def next_command():
            if not queued:
                raise Empty
            return queued.popleft()

        controller = SimpleNamespace(state="IDLE")
        controller.start = lambda: setattr(controller, "state", "TAKEOFF")
        controller.stop = lambda: setattr(controller, "state", "STOPPED")
        harness = SimpleNamespace(
            next_command=next_command, controller=controller, runtime_error=None,
            connected=lambda: True, publish_state=lambda: None,
        )
        harness.apply_command = MethodType(FleetRuntime.apply_command, harness)
        harness.record_command = MethodType(FleetRuntime.record_command, harness)
        return harness, futures

    def test_broken_logger_does_not_change_command_results_or_strand_futures(self):
        harness, futures = self.harness([
            {"action": "start"}, {"action": "unknown"}, {"action": "stop"},
        ])

        def broken_log(_message):
            raise RuntimeError("injected ROS logging failure")

        harness.get_logger = lambda: SimpleNamespace(info=broken_log, warning=broken_log)
        with redirect_stderr(io.StringIO()) as fallback:
            FleetRuntime.process_commands(harness)
            FleetRuntime.process_commands(harness)
        self.assertTrue(all(future.done() for future in futures))
        self.assertTrue(futures[0].result()["accepted"])
        self.assertIsInstance(futures[1].exception(), RuntimeError)
        self.assertIn("Unknown action", str(futures[1].exception()))
        self.assertTrue(futures[2].result()["accepted"])
        self.assertEqual(harness.controller.state, "STOPPED")
        self.assertIn("injected ROS logging failure", fallback.getvalue())

    def test_rejection_future_completes_before_unexpected_diagnostic_failure(self):
        harness, futures = self.harness([{"action": "unknown"}])

        def broken_record(*_args):
            raise RuntimeError("unexpected diagnostic failure")

        harness.record_command = broken_record
        with self.assertRaisesRegex(RuntimeError, "unexpected diagnostic failure"):
            FleetRuntime.process_commands(harness)
        self.assertTrue(futures[0].done())
        self.assertIn("Unknown action", str(futures[0].exception()))


if __name__ == "__main__":
    unittest.main()
