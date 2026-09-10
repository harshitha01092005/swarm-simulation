#!/usr/bin/env python3
"""Exercise the full local HTTP application against measured Gazebo feedback.

Start full_system with auto_start:=false before invoking this acceptance test.
The default full run finishes with 400 connected drones in IDLE for a GUI demo.
"""

import argparse
import base64
import json
import math
from pathlib import Path
import sys
import time
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import numpy as np


class APIAcceptance:
    def __init__(self, args):
        self.args = args
        self.url = args.url if "://" in args.url else "http://" + args.url
        self.url = self.url.rstrip("/")
        parsed = urlsplit(self.url)
        if parsed.scheme != "http" or parsed.hostname not in {"localhost", "127.0.0.1", "::1"}:
            raise ValueError("Use an HTTP URL on localhost, 127.0.0.1, or ::1")
        if not 1 <= args.count <= 400:
            raise ValueError("count must be between 1 and 400")
        self.created = time.monotonic()
        self.deadline = self.created + args.timeout
        self.state = None
        self.checks = []
        self.skipped = []
        self.request_count = 0
        self.poll_count = 0
        self.last_pose_wall = None
        self.last_pose_sequence = None
        self.previous_positions = None
        self.previous_pose_time = None
        self.reset_grace_until = 0.0
        self.minimum_separation = math.inf
        self.minimum_separation_margin = math.inf
        self.maximum_interval_speed = 0.0
        self.maximum_batch_speed = 0.0
        self.maximum_tracking_error = 0.0
        self.rates = {"control_update_hz": [], "pose_update_hz": [], "real_time_factor": []}
        self.leave_running = False

    def request(self, method, path, body=None, expected=200, headers=None, decode=True, allow_after_deadline=False):
        if not allow_after_deadline and time.monotonic() >= self.deadline:
            raise TimeoutError("HTTP acceptance exceeded its overall wall-time limit")
        request_headers = dict(headers or {})
        if isinstance(body, dict):
            body = json.dumps(body, allow_nan=False).encode()
            request_headers.setdefault("Content-Type", "application/json")
        request = Request(self.url + path, data=body, method=method, headers=request_headers)
        try:
            with urlopen(request, timeout=20 if method == "POST" else 5) as response:
                status, response_body = response.status, response.read(12 * 1024 * 1024 + 1)
        except HTTPError as error:
            status, response_body = error.code, error.read(64 * 1024)
        if status != expected:
            raise AssertionError(f"{method} {path} returned HTTP {status}, expected {expected}: {response_body[:500]!r}")
        if len(response_body) > 12 * 1024 * 1024:
            raise AssertionError("HTTP response exceeded the acceptance test's size limit")
        return json.loads(response_body) if decode else response_body

    def command(self, action, **parameters):
        self.request_count += 1
        result = self.request("POST", "/api/command", {
            "action": action, "request_id": f"api-acceptance-{self.request_count}", **parameters,
        })
        if result.get("ok") is not True or result.get("accepted") is not True:
            raise AssertionError(f"Command was not acknowledged: {result}")
        return result

    def mark(self, name, **details):
        self.checks.append({"name": name, "passed": True, **details})
        print(f"PASS: {name}" + (" " + json.dumps(details) if details else ""), flush=True)

    def poll(self):
        started = time.monotonic()
        state = self.request("GET", "/api/state")
        self.poll_count += 1
        if state.get("state") == "ERROR":
            raise AssertionError(f"Runtime entered ERROR: {state.get('error', state.get('message'))}")
        count = state.get("count")
        if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 400:
            raise AssertionError("State contains an invalid active count")
        if state.get("capacity") != 400:
            raise AssertionError(f"Full application capacity must be 400, got {state.get('capacity')}")
        colors = np.asarray(state.get("colors"), dtype=float)
        if colors.shape != (count, 3) or not np.isfinite(colors).all() or (colors < 0).any() or (colors > 1).any():
            raise AssertionError("Commanded colors must contain one finite RGB triple in [0,1] per drone")
        if state.get("colors_source") != "controller":
            raise AssertionError("Color provenance must identify controller output")
        for key in self.rates:
            value = state.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise AssertionError(f"Invalid measured rate {key}: {value!r}")
            if value > 0:
                self.rates[key].append(float(value))
        actual = state.get("actual_positions")
        if actual is not None:
            positions = np.asarray(actual, dtype=float)
            desired = np.asarray(state.get("desired_positions"), dtype=float)
            targets = np.asarray(state.get("targets"), dtype=float)
            for label, value in (("actual", positions), ("desired", desired), ("targets", targets)):
                if value.shape != (count, 3) or not np.isfinite(value).all():
                    raise AssertionError(f"Invalid {label} pose array")
            if state.get("positions") != actual:
                raise AssertionError("Canonical positions must be actual Gazebo observations")
            reset_window = state.get("layout_pending") or started < self.reset_grace_until
            if not reset_window:
                if count > 1:
                    distances = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
                    np.fill_diagonal(distances, np.inf)
                    separation = float(distances.min())
                    minimum = float(state["config"]["min_distance"])
                    self.minimum_separation = min(self.minimum_separation, separation)
                    self.minimum_separation_margin = min(self.minimum_separation_margin, separation - minimum)
                    if state["config"]["collision_avoidance"] and separation < minimum - 1e-5:
                        raise AssertionError(f"Observed separation {separation:.6f} m is below configured {minimum:.6f} m")
                error = float(np.linalg.norm(positions - desired, axis=1).max())
                self.maximum_tracking_error = max(self.maximum_tracking_error, error)
                pose_time = state.get("actual_simulation_time")
                if not isinstance(pose_time, (int, float)) or not math.isfinite(pose_time):
                    raise AssertionError("Measured poses need a finite Gazebo timestamp")
                if self.previous_positions is not None and self.previous_positions.shape == positions.shape and pose_time > self.previous_pose_time:
                    speed = float(np.linalg.norm(positions - self.previous_positions, axis=1).max() / (pose_time - self.previous_pose_time))
                    self.maximum_interval_speed = max(self.maximum_interval_speed, speed)
                self.previous_positions, self.previous_pose_time = positions.copy(), pose_time
                batch_speed = float(state.get("actual_max_speed_m_s", 0))
                if not math.isfinite(batch_speed) or batch_speed < 0:
                    raise AssertionError("Invalid measured pose-batch speed")
                self.maximum_batch_speed = max(self.maximum_batch_speed, batch_speed)
            else:
                self.previous_positions = self.previous_pose_time = None
            sequence = state.get("pose_sequence")
            if sequence != self.last_pose_sequence:
                self.last_pose_sequence = sequence
                self.last_pose_wall = started
        if self.last_pose_wall is not None and started - self.last_pose_wall > 3 and started >= self.reset_grace_until:
            raise AssertionError("Measured Gazebo poses stopped advancing")
        self.state = state
        time.sleep(max(0, 0.1 - (time.monotonic() - started)))
        return state

    def wait(self, predicate, description, timeout=120):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while time.monotonic() < deadline:
            state = self.poll()
            if predicate(state):
                return state
        latest = None if self.state is None else {key: self.state.get(key) for key in ("state", "formation", "count", "connected", "message")}
        raise TimeoutError(f"Timed out waiting for {description}; last state={latest}")

    def observe(self, seconds, assertion=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            state = self.poll()
            if assertion is not None:
                assertion(state)

    @staticmethod
    def complete(state, formation):
        if not (state.get("connected") and state.get("formation") == formation
                and state.get("state") in {"FORMATION_COMPLETE", "ANIMATING"}
                and state.get("progress", 0) >= 0.99 and state.get("pending_formation") is None):
            return False
        return float(np.linalg.norm(np.asarray(state["actual_positions"]) - np.asarray(state["targets"]), axis=1).max()) <= 0.35

    def pause_and_hold(self, label):
        self.command("pause")
        self.wait(lambda state: state["state"] == "PAUSED", "pause acknowledgement", 10)
        self.observe(0.35)
        initial = np.asarray(self.state["actual_positions"]).copy()
        initial_colors = np.asarray(self.state["colors"]).copy()
        simulation_time = self.state["simulation_time"]
        maximum_drift = 0.0

        def held(state):
            nonlocal maximum_drift
            drift = float(np.linalg.norm(np.asarray(state["actual_positions"]) - initial, axis=1).max())
            maximum_drift = max(maximum_drift, drift)
            if drift > 0.05 or state["state"] != "PAUSED":
                raise AssertionError(f"Paused swarm did not hold its measured position: drift={drift:.4f} m")
            if abs(state["simulation_time"] - simulation_time) > 1e-6:
                raise AssertionError("Controller simulation time advanced while paused")
            if not np.allclose(np.asarray(state["colors"]), initial_colors, atol=1e-7, rtol=0):
                raise AssertionError("Commanded colors changed while paused")

        self.observe(1.1, held)
        self.mark(label, measured_drift_m=maximum_drift)

    def reject_invalid_requests(self, active=False):
        invalid = self.request("POST", "/api/command", b"{", expected=400,
                               headers={"Content-Type": "application/json"})
        if not invalid.get("error"):
            raise AssertionError("Malformed JSON rejection did not explain the error")
        forbidden = self.request("POST", "/api/command", {"action": "stop"}, expected=403,
                                 headers={"Origin": "http://different-origin.invalid"})
        if not forbidden.get("error"):
            raise AssertionError("Cross-origin rejection did not explain the error")
        if active:
            previous_count = self.state["count"]
            rejected = self.request("POST", "/api/command", {
                "action": "configure", "settings": {"count": 20 if previous_count != 20 else 21},
            }, expected=400)
            if not rejected.get("error") or self.poll()["count"] != previous_count:
                raise AssertionError("Active count change was not rejected without changing the scene")
        self.mark("invalid JSON and cross-origin requests rejected" + ("; active count change rejected" if active else ""))

    def upload_sample(self):
        sample = self.request("GET", "/api/sample", decode=False)
        if not sample or len(sample) > 8 * 1024 * 1024:
            raise AssertionError("Sample image is empty or exceeds the upload limit")
        result = self.command("image", data=base64.b64encode(sample).decode("ascii"),
                              mode="edges", threshold=127, invert=False)
        if not result.get("image") or not result.get("preview"):
            raise AssertionError("Image upload response lacks metadata or preview")
        preview = base64.b64decode(result["preview"], validate=True)
        if not preview.startswith(b"\x89PNG\r\n\x1a\n"):
            raise AssertionError("Processed image preview is not a PNG")
        self.wait(lambda state: state.get("image") is not None, "processed image metadata", 15)
        self.mark("sample uploaded, processed, and preview returned", sample_bytes=len(sample), preview_bytes=len(preview))

    def reset_scene(self, action, **parameters):
        self.reset_grace_until = time.monotonic() + 2.5
        self.previous_positions = self.previous_pose_time = None
        self.command(action, **parameters)

    def validate_grid(self, count):
        state = self.wait(lambda value: value["count"] == count and value.get("connected")
                          and value.get("actual_count") == count and not value.get("layout_pending"),
                          f"{count} active measured poses after scene reset", 20)
        self.observe(max(0, self.reset_grace_until - time.monotonic()) + 0.1)
        state = self.state
        columns, rows = math.ceil(math.sqrt(count)), math.ceil(count / math.ceil(math.sqrt(count)))
        index = np.arange(count)
        spacing = state["config"]["min_distance"]
        grid = np.column_stack(((index % columns - (columns - 1) / 2) * spacing,
                                (index // columns - (rows - 1) / 2) * spacing, np.full(count, 0.5)))
        error = float(np.linalg.norm(np.asarray(state["actual_positions"]) - grid, axis=1).max())
        if error > 0.005 or state["state"] != "IDLE":
            raise AssertionError(f"Reset did not produce the measured IDLE ground grid: error={error:.5f} m, state={state['state']}")
        if state.get("image") is None:
            raise AssertionError("Scene reset discarded the uploaded image")
        return error

    def run(self):
        initial = self.wait(lambda state: state.get("connected") and state.get("actual_count") == state["count"],
                            "initial Gazebo connection", 60)
        if initial["state"] != "IDLE":
            raise AssertionError("Start this test with a fresh IDLE application (auto_start:=false)")
        if initial["count"] != self.args.count:
            self.reset_scene("configure", settings={"count": self.args.count})
            self.wait(lambda state: state.get("connected") and state["actual_count"] == self.args.count,
                      "requested starting drone count", 20)
        self.mark("initial IDLE application connected", count=self.args.count, capacity=400)
        if not self.args.skip_long:
            self.command("start")
            self.wait(lambda state: state["state"] in {"TAKEOFF", "TRANSITIONING"}, "automatic motion", 15)
            self.observe(0.5)
            self.reject_invalid_requests(active=True)
            self.pause_and_hold("moving swarm pauses with frozen poses, clock, and colors")
            self.command("resume")
            self.wait(lambda state: self.complete(state, "circle"), "circle completion", 120)
            self.mark("resume completes measured circle formation")
        else:
            self.skipped.extend(["automatic circle motion", "moving pause/resume", "active count rejection",
                                 "image formation motion", "pulse animation/pause", "spiral transition"])
            self.reject_invalid_requests()

        self.upload_sample()
        if not self.args.skip_long:
            self.command("configure", settings={"color_mode": "image"})
            self.command("apply", formation="image")
            self.wait(lambda state: self.complete(state, "image"), "measured image formation", 120)
            unique = len(np.unique(np.round(np.asarray(self.state["colors"]), 3), axis=0))
            if unique <= 3:
                raise AssertionError(f"Image lighting only contains {unique} distinct RGB colors")
            self.mark("image formation reaches measured targets with source colors", distinct_colors=unique)
            self.command("configure", settings={"color_mode": "pulse"})
            self.wait(lambda state: state["state"] == "ANIMATING", "pulse animation", 10)
            baseline = np.asarray(self.state["actual_positions"]).copy()
            colors = np.asarray(self.state["colors"]).copy()
            maximum_change = 0.0

            def pulsing(state):
                nonlocal maximum_change
                if float(np.linalg.norm(np.asarray(state["actual_positions"]) - baseline, axis=1).max()) > 0.05:
                    raise AssertionError("Lighting animation moved the measured drone positions")
                maximum_change = max(maximum_change, float(np.abs(np.asarray(state["colors"]) - colors).max()))

            self.observe(1.2, pulsing)
            if maximum_change < 0.02:
                raise AssertionError("Pulse mode did not change the commanded RGB values")
            self.mark("pulse changes colors while measured positions hold", maximum_rgb_change=maximum_change)
            self.pause_and_hold("animation pause freezes both poses and RGB")
            self.command("resume")
            self.command("apply", formation="spiral")
            self.wait(lambda state: self.complete(state, "spiral"), "measured spiral formation", 120)
            self.mark("resumed animation transitions to a measured spiral")

        self.command("stop")
        self.wait(lambda state: state["state"] == "STOPPED", "STOPPED acknowledgement", 10)
        self.reset_scene("configure", settings={"count": 20})
        self.wait(lambda state: state.get("connected") and state["count"] == 20 and state["actual_count"] == 20,
                  "20 active measured drones after configure", 20)
        self.reset_scene("reset")
        grid_error = self.validate_grid(20)
        self.mark("count20 and reset restore measured grid while preserving image", grid_error_m=grid_error)
        self.reset_scene("configure", settings={"count": 400})
        grid_error = self.validate_grid(400)
        self.mark("capacity400 restored in connected IDLE", grid_error_m=grid_error)
        if self.args.start_final:
            self.command("start")
            self.wait(lambda state: state["state"] in {"TAKEOFF", "TRANSITIONING"}, "final show start", 15)
        self.leave_running = True

    def report(self, passed, failure):
        def rate_summary(values):
            return None if not values else {"minimum_nonzero": min(values), "mean": sum(values) / len(values), "maximum": max(values)}

        result = {
            "passed": passed, "scope": "quick API checks" if self.args.skip_long else "full HTTP-controlled application acceptance",
            "failure": failure, "url": self.url, "initial_test_count": self.args.count,
            "wall_seconds": time.monotonic() - self.created, "state_polls": self.poll_count,
            "checks": self.checks, "skipped_checks": self.skipped,
            "observed_minimum_separation_m": self.minimum_separation if math.isfinite(self.minimum_separation) else None,
            "observed_minimum_separation_margin_m": self.minimum_separation_margin if math.isfinite(self.minimum_separation_margin) else None,
            "observed_maximum_interval_speed_m_s": self.maximum_interval_speed,
            "maximum_reported_pose_batch_speed_m_s": self.maximum_batch_speed,
            "observed_maximum_tracking_error_m": self.maximum_tracking_error,
            "rates": {name: rate_summary(values) for name, values in self.rates.items()},
            "measurement_note": "Positions are Gazebo observations; RGB values are controller commands. Metrics exclude explicit scene-reset grace periods.",
            "final_state": None if self.state is None else {key: self.state.get(key) for key in ("state", "count", "capacity", "connected", "formation")},
        }
        path = Path(self.args.report).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(f"{'PASS' if passed else 'FAIL'}: API acceptance; report={path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="127.0.0.1:8765")
    parser.add_argument("--report", default="artifacts/api-acceptance.json")
    parser.add_argument("--count", type=int, default=400)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--skip-long", action="store_true", help="Run API, image-upload, count, and reset checks; explicitly skip flight/animation checks.")
    parser.add_argument("--start-final", action="store_true", help="Start the final 400-drone show; default leaves the application in IDLE.")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a positive finite number")
    test = None
    failure = None
    code = 1
    try:
        test = APIAcceptance(args)
        test.run()
        code = 0
    except KeyboardInterrupt:
        failure, code = "Interrupted", 130
    except Exception as error:
        failure = f"{type(error).__name__}: {error}"
        print(f"FAIL: {failure}", file=sys.stderr, flush=True)
    finally:
        if test is not None:
            if not test.leave_running:
                try:
                    test.request("POST", "/api/command", {"action": "stop"}, allow_after_deadline=True)
                except Exception as error:
                    print(f"Final stop could not be confirmed: {error}", file=sys.stderr, flush=True)
            try:
                test.report(code == 0, failure)
            except Exception as error:
                print(f"Could not write acceptance report: {error}", file=sys.stderr, flush=True)
                code = 1
    return code


if __name__ == "__main__":
    sys.exit(main())
