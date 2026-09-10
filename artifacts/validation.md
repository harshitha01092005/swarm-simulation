# Phase 1 validation record

Date: 2026-09-10. Host: macOS arm64, Python 3.10.13. Docker engine:
27.1.1 in the existing Colima VM. Native ROS, Gazebo and colcon are absent.
The verified Linux container runs Ubuntu 24.04 arm64 (`aarch64`), Python 3.12.3,
ROS 2 Jazzy and Gazebo Harmonic 8.11.0. The installed `ros-jazzy-ros-gz` package
version is `1.0.22-1noble.20260615.095917`.

**Result: Phase 1 builds and passes real headless ROS/Gazebo communication and
mission tests.** Visible Gazebo rendering remains unverified.

## Native checks — passed

```bash
python3 -m unittest discover -s tests -v
python3 setup.py check
python3 -m compileall -q drone_swarm launch scripts tests
bash -n docker/entrypoint.sh scripts/run_integration.sh
```

All 18 unit tests passed. They cover the measured-position mission loop,
endpoint/speed behavior, state transitions, invalid inputs, coordinate-frame
rotation, model structure, package resource names and required world systems.
Package metadata, Python compilation and shell syntax checks also passed.
Integration-runner process fixtures separately checked success/failure handling,
early launch exit, logs and process-group cleanup.

## Docker and colcon build — passed

```bash
docker build -t drone-swarm:phase1 -f docker/Dockerfile .
```

The default `ros:jazzy-ros-base-noble` image from Docker Hub built successfully,
including the colcon package build. No registry override was needed.

## Real ROS/Gazebo communication — passed

```bash
docker run --rm drone-swarm:phase1 bash -c 'python3 -m unittest discover -s tests -v && bash scripts/run_integration.sh'
```

Exit status: **0**. All 18 tests passed again inside Ubuntu with Python 3.12.3.
The integration runner launched a real Gazebo server and ROS bridge. The probe
verified advancing clock, fresh finite odometry, commanded motion and zero hold.

| Measurement | Observed result |
| --- | --- |
| Vertical displacement | 0.805 m |
| Horizontal displacement | 0.604 m |
| Zero-command hold | Passed |

These are simulator odometry measurements, not mocked position changes.

## Automatic mission and pause/resume — passed

```bash
docker run --rm drone-swarm:phase1 env SWARM_TEST_MODE=mission bash scripts/run_integration.sh
```

Exit status: **0**. Mission mode uses `scripts/test_mission_live.py` with the
actual demo controller and Gazebo simulation. It verifies the takeoff / waypoint /
return mission, a ROS pause and resume, and completion.

| Measurement | Observed result |
| --- | --- |
| Paused position drift | 0.0000 m |
| Final position error | 0.0125 m |
| Maximum measured speed | 0.926 m/s |
| Observed states | `IDLE`, `TAKEOFF`, `MOVING`, `PAUSED`, `COMPLETE` |

The runner accepts `SWARM_TEST_MODE=communication` (default) or `mission` and
cleans up the simulation it launches in either mode.

## Initial environment failures and recovery

Initial attempts encountered a Docker Hub CDN certificate error and alternative
registry DNS failures. Restarting Colima restored registry resolution in
approximately 11 ms. Unrelated local services were restored, and the active
Docker context was restored to `colima`. The default build and the tests above
then succeeded.

No CA/trust-store, custom DNS, proxy or registry-mirror configuration changes
were required. TLS verification remained enabled. See the
[registry troubleshooting guide](../TROUBLESHOOTING.md#registry-dns-or-tls-errors)
for recovery guidance.

## Remaining verification and scope

Run the visible Gazebo launch on an Ubuntu desktop or graphics-capable VM to
verify rendering. Headless tests do not validate a display, rendered FPS or
GPU/VM performance. See [INSTALLATION.md](../INSTALLATION.md).

An optional native wheel build was not verified because the host's `wheel`
build command was unavailable. This does not affect the successful colcon path.

This is the single-drone Phase 1 milestone. Image processing, multi-drone
formations, 400-drone scaling, swarm collision avoidance, RGB animations and the
custom PySide6 GUI remain later work. These tests do not establish completion of
the full project described in the original brief.
