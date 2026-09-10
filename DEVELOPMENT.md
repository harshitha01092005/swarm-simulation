# Development and validation

Use [DEVELOPMENT_PROGRESS.md](DEVELOPMENT_PROGRESS.md) as the acceptance index. It links measured reports and records completed gates and their limits. Unit tests, live Gazebo checks and visible UI checks establish different things; none substitutes for the others.

[PERFORMANCE.md](PERFORMANCE.md) records the successful optimized scale, HTTP, native desktop and Gazebo LED checks, CPU/memory methodology, historical failures and actual-speed limitations. [The full validation record](artifacts/full-validation.md) includes the passed final visual rerun and distinguishes functional acceptance from measurement limits.

## Local tests and CPU profiling

Install the native desktop environment, then run from the repository:

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/benchmark_core.py --output artifacts/core-performance.json
```

Tests cover configuration, formations, image bounds and sampling, trajectory assignment and separation, state transitions, lighting, and API contracts. The benchmark exercises 100, 200 and 400 drones and records image processing, assignment/planning, controller-step p95 and snapshot-serialization p95 timing with Python/NumPy/platform metadata. It excludes ROS, Gazebo and rendering.

Recorded native result: **85 tests discovered; 82 passed and 3 skipped** on macOS arm64/Python 3.10.13. The three skips are actual-rclpy logging/error-path regressions because ROS is absent from the native environment. They are not verified by that host run. All **85 passed with no skips** inside the Ubuntu ROS image, including those regressions. [Validation summary](artifacts/full-validation.md#gate-status).

Run the complete ROS-aware suite separately:

```bash
docker run --rm drone-swarm:latest python3 -m unittest discover -s tests -v
```

For a read-only environment report, use `python3 scripts/doctor.py`. Missing native ROS/Gazebo produces a nonzero result on the Mac host; this is expected and does not invalidate native algorithm tests.

## Build and C++ tests

```bash
docker build -t drone-swarm:latest -f docker/Dockerfile .
```

The Docker build compiles the Gazebo plugin, runs CTest and builds the selected ament package with colcon. Native Ubuntu developers can repeat the plugin checks directly:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S gazebo_plugin -B build/gazebo_plugin -DCMAKE_BUILD_TYPE=Release
cmake --build build/gazebo_plugin -j2
ctest --test-dir build/gazebo_plugin --output-on-failure
```

The C++ suites validate pose/RGB protocols, active membership and material command contents; the recorded result is **2/2 CTests passed**. They do not establish visible rendering. Install the rebuilt plugin and rebuild/source the colcon workspace as described in [INSTALLATION.md](INSTALLATION.md).

Docker copies source at build time. After editing code or assets, rebuild, stop the application container and start it again. An existing container continues using its original image.

The managed launcher keeps stopped containers for diagnosis, archives their logs to `artifacts/last-session.log` before replacing them, and rejects reuse of a running container from an older image. It waits for measured connectivity rather than HTTP liveness. Integration commands below still use disposable `--rm` containers; mount reports when retaining those logs matters.

## Live ROS/Gazebo integration

The runner supports `communication`, `mission`, `fleet` and `show`. Each invocation owns and cleans up its launched process groups, uses a separate Gazebo partition/ROS domain, retains failure status and fails when its runtime is missing. Run with the current rebuilt image:

Record both active count and allocated model capacity. The fleet/show runner allocates the selected count; the full application pre-spawns 400 models regardless of active count. Its initial 2 GiB/2-CPU Colima run was OOM-killed (Docker event at 18:19:22 UTC). The optimized 100/200/400 runs subsequently passed on the recommended 4-CPU/6-GiB VM allocation; record the actual allocation with any new report.

The current fleet backend disables the Physics system, updates owned ECM Pose components directly and marks them as periodic changes. Keep the 120 Hz SimulationRunner timing profile and verify advancing `/clock`, actual ECM movement, scene updates and LED materials after backend changes. Current scale reports measure this backend; earlier dynamics-enabled performance numbers do not describe it. Preserve the original Physics-based one-drone tests as a separate regression gate.

```bash
docker run --rm drone-swarm:latest bash -c 'python3 -m unittest discover -s tests -v && bash scripts/run_integration.sh'
docker run --rm drone-swarm:latest env SWARM_TEST_MODE=mission bash scripts/run_integration.sh
docker run --rm drone-swarm:latest env SWARM_TEST_MODE=fleet SWARM_TEST_COUNT=5 bash scripts/run_integration.sh
docker run --rm drone-swarm:latest env SWARM_TEST_MODE=show SWARM_TEST_COUNT=20 bash scripts/run_integration.sh
```

| Mode | Checks |
| --- | --- |
| `communication` | Legacy drone discovery, advancing clock/odometry, measured vertical/horizontal movement and zero-command hold |
| `mission` | Legacy takeoff/waypoint/return mission and pause/resume |
| `fleet` | Active fleet identities, actual Gazebo movement and circle completion |
| `show` | Requested formation sequence, measured completion/error, pose rate and observed motion metrics |

The show probe defaults to circle, star, spiral, wave and infinity. To include an image and retain output outside a disposable container:

```bash
mkdir -p artifacts
for SWARM_COUNT in 100 200 400; do
  docker run --rm -v "$PWD/artifacts:/reports" \
    -e SWARM_TEST_MODE=show -e SWARM_TEST_COUNT="$SWARM_COUNT" \
    -e SWARM_TEST_LOG_DIR="/reports/integration-$SWARM_COUNT" \
    drone-swarm:latest bash scripts/run_integration.sh \
    --ros-args -p formations:=circle,image,spiral \
    -p image_path:=/ws/src/drone_swarm_simulator/assets/sample.png \
    -p "report_path:=/reports/show-$SWARM_COUNT.json" -p timeout:=300.0
done
```

These are reproducible acceptance commands, not a claim that every count has passed. Check each JSON `passed` value and error, along with [the acceptance index](DEVELOPMENT_PROGRESS.md). A successful show test's observed non-overlap requirement must not be confused with a strict configured-separation gate; read the report's `separation_requirement` and the relevant dedicated validation results.

Measured-position speed intervals use `actual_simulation_time`, the timestamp attached to the Gazebo observation. Do not divide actual displacement by the controller's separate mission `simulation_time`. An early 100-drone probe did that and reported an apparent speed violation; use corrected-probe rerun evidence when assessing the speed gate. Reference speed bounds and measured backend interval speed remain distinct quantities.

Without a mount, logs under `artifacts/integration/<UTC>-<PID>/` disappear when `docker run --rm` removes the container. Each run writes `launch.log` and `smoke.log`; `SWARM_TEST_LOG_DIR` overrides their location. The show probe accepts `count`, `formations`, `image_path`, `report_path` and `timeout` ROS parameters. The communication probe accepts `readiness_timeout` and `phase_timeout`.

On a sourced Ubuntu installation, run the same script from the source directory without Docker, for example `SWARM_TEST_MODE=show SWARM_TEST_COUNT=20 bash scripts/run_integration.sh`.

## End-to-end acceptance

The full HTTP application probe passed **11/11 checks with no skips** in 87.03 seconds. It validated 400-drone circle/image/spiral motion, pause/resume and frozen animation, source RGB commands, invalid requests, sampled configured separation, and count/reset/image-retention behavior. Its report records 1.5 m minimum observed separation and zero pause drift. See [artifacts/api-acceptance.json](artifacts/api-acceptance.json).

Start a fresh full application in connected IDLE, then run from the host:

```bash
bash scripts/start.sh --headless
.venv/bin/python scripts/test_api_live.py --url http://127.0.0.1:8765 \
  --report artifacts/api-acceptance-rerun.json
```

This operates the current show and normally restores connected IDLE at count/capacity 400. Run API, desktop and LED control tests sequentially. The optional `--skip-long` mode lists omitted checks explicitly and does not replace the full test. Its speed metrics are observations, not a strict-speed assertion: the recorded HTTP interval maximum was 5.364 m/s and the reported pose-batch maximum was 8.075 m/s against a 5 m/s reference limit. [Measurement details](PERFORMANCE.md#full-http-application-acceptance).

The final native desktop exercise passed **6/6 checks** in 66.20 seconds: healthy 400-drone WebGL connectivity, Start/circle, sample processing and both previews, measured image completion, image colors and Front/Fit camera controls. Circle/image target errors were 1.301/0.813 mm; the final viewer reading was 60 FPS with no JavaScript console errors. This is a final frame-rate observation, not a sustained benchmark. Visual inspection confirmed the revised RGB glows and uncropped previews in the 1512 × 949 window (3024 × 1898 Retina capture). [Desktop report](artifacts/desktop-workflow.json), [final screenshot](artifacts/desktop-workflow.png).

The independent material probe passed actual red/blue ECS Material reads on the same owned Gazebo LED visual and restored image lighting. [LED report](artifacts/gazebo-leds.json). It verifies the material transport/update path, not a per-visual audit of all 400 LEDs.

After the API test leaves the application in IDLE with Circle selected, run these checks sequentially:

```bash
.venv/bin/python scripts/test_desktop.py --exercise --output artifacts/desktop-workflow-rerun.png
.venv/bin/python scripts/test_gazebo_leds.py --report artifacts/gazebo-leds-rerun.json
```

The desktop exercise opens a native window and leaves the completed image visible. For further manual regression, start takeoff/circle before processing the sample; processing automatically selects Image. Apply the image, select image colors, change to spiral, pause/resume, stop, change count and reset. Check queued edits while moving, brightness/effects, camera controls and useful errors for rejected uploads or disconnected feedback.

The UI must show measured Gazebo positions and stale/offline status. Compare rendered count with active feedback, verify that controls acknowledge real outcomes, and test the Gazebo LED material path separately from the client color display. A screenshot alone cannot prove ROS communication or material updates.

Record count/capacity, platform, CPU/RAM allocation, headless/rendered mode, Physics-system enabled/disabled status, simulation update schedule, test duration and exact command with every performance result. Report observed pose/control rates, real-time factor, tracking error, actual minimum separation and viewer FPS independently. Neither the 120 Hz simulator schedule nor core planning time establishes full-system frame rate. Record any software-rendering fallback.

## Development contracts

Keep algorithms deterministic and independent of ROS/Qt. New formations must return exactly N finite targets. Image processing must reject oversized headers before decoding and fail clearly when distinct samples are insufficient. Endpoint depth feasibility and continuous trajectory separation are separate checks.

HTTP image decoding/sampling runs in a bounded background worker so it does not block the ROS update loop. The completed result is committed on the ROS executor against the relevant configuration. Validate this responsiveness through live image-upload tests; implementation alone is not a passed runtime gate.

Motion edits during travel remain queued until a completed boundary. Count/separation resets must synchronize the new layout with Gazebo feedback before accepting motion. Preserve the distinction between desired and actual positions in snapshots, tests and the viewer. Use simulation time for motion and wall time for watchdogs and bounded readiness.

The full runtime latches errors on stalled feedback or a backwards clock. Use application Pause/Resume when testing holds; a Gazebo world pause freezes its time source. Historical Phase 1 measurements are preserved in [artifacts/validation.md](artifacts/validation.md); current full-system evidence belongs in the acceptance index and its linked reports.
