# Performance and measurement limits

The optimized **400-drone headless Gazebo run passed** its image → star show test in **76.10 seconds**, with mean real-time factor **0.9938**, **29.77 received pose batches/s** and **29.93 controller updates/s**. This establishes near-real-time operation for that workload on the recorded machine. It does not establish 60 FPS rendering or a strict measured 5 m/s motion limit. [Raw 400-drone report](artifacts/phase6-400.json).

## Test environment and workload

| Item | Recorded setup |
| --- | --- |
| Host | macOS 26.6.2, Apple Silicon arm64, 36 GiB RAM, 11 CPUs |
| Docker VM | Colima, 4 CPUs, 6 GiB configured RAM |
| Container | Ubuntu 24.04 arm64, Python 3.12.3, ROS 2 Jazzy, Gazebo Harmonic 8.11.0 |
| Fleet backend | Static models, Physics system disabled, direct owned ECM Pose updates |
| Simulation schedule | Gazebo SimulationRunner at 120 Hz, requested real-time factor 1 |
| Transport/control | Central ROS controller and batched pose bridge, target 30 Hz |
| Rendering during scale test | Headless; no desktop/browser rendering workload included |
| Formation settings | Size 50 m, altitude 35 m, reference speed 5 m/s, separation 1.5 m, avoidance enabled |
| Show sequence | Initial takeoff/circle, then bundled sample image and star |
| Transition setting | Initially 10 s; the show probe requests 3 s for subsequent formations, extended by the planner when required |

Each scale test allocates exactly its active drone count. This differs from the normal full application, which allocates capacity for 400 models even when fewer are active. Unrelated local services also remained on the shared Docker VM. Per-container measurements below exclude their own resource usage but do not eliminate scheduling contention.

## Optimized scale results

| Active / capacity | Result | Total wall time (s) | Mean RTF | Pose batches/s | Mean control Hz | Mean CPU cores | Peak memory (MiB) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| [100 / 100](artifacts/scale-100.json) | Passed | 61.78 | 0.9998 | 29.99 | 29.98 | 0.409 | 916.87 |
| [200 / 200](artifacts/scale-200.json) | Passed | 67.24 | 0.9991 | 29.96 | 29.98 | 0.549 | 1600.38 |
| [400 / 400](artifacts/phase6-400.json) | Passed | 76.10 | 0.9938 | 29.77 | 29.93 | 1.042 | 3055.52 |

| Active count | Minimum observed separation (m) | Image final error (m) | Star final error (m) | Maximum tracking error (m) | Maximum observed interval speed (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 100 | 1.500000 | 0.000194 | 0.000000 | 0.204566 | 6.249011 |
| 200 | 1.500000 | 0.000734 | 0.000001 | 0.166664 | 4.999607 |
| 400 | 1.500000 | 0.001215 | 0.002316 | 0.333313 | 6.432481 |

Final error is the maximum Euclidean target error across drones at the probe's completion observation. The zero in the 100-drone star row is the recorded floating-point comparison result, not a claim of physical measurement precision. Maximum tracking error includes transient differences between the latest observed and requested poses.

These are individual runs, not repeated-trial distributions. The report does not contain confidence intervals, latency percentiles or worst-case sustained-load guarantees. Additional acceptance gates are tracked in [DEVELOPMENT_PROGRESS.md](DEVELOPMENT_PROGRESS.md).

## What the numbers measure

- **RTF:** the arithmetic mean of positive runtime real-time-factor telemetry samples. Each sample compares advancing Gazebo simulation time with wall time. It is not viewer FPS.
- **Pose rate:** received Gazebo pose-batch count divided by wall time between the first and last observed batch. Each batch contains the fleet, not one drone.
- **Control rate:** mean reported measured controller update rate over the same accepted telemetry samples. The requested 30 Hz timer is not substituted for the measurement.
- **CPU cores:** container cgroup CPU seconds divided by test wall time. This includes the simulator, ROS processes and acceptance probe; it is not Gazebo-only usage or a percentage of all host CPUs.
- **Peak memory:** Linux cgroup `memory.peak` for the container, expressed in MiB. It includes startup and is distinct from the VM allocation, other containers and host GUI memory.
- **Position metrics:** actual Gazebo ECM world-pose readback, not the controller's desired-position array. The backend is still explicitly kinematic, without motor or aerodynamic dynamics.

The continuous planner bounds its **reference** velocity to the configured 5 m/s. Actual Gazebo poses are updated from discrete ROS batches and held between arrivals. Batch delivery and observation timing can concentrate displacement into shorter observation intervals. The 400-drone test recorded **6.432 m/s**, about **28.6% above the reference limit**, using `actual_simulation_time` from the observed poses. This is a backend limitation consistent with batched updates, not evidence of a strict 5 m/s bound on actual motion.

The live show probe allows interval speeds up to **1.3× the configured reference limit** and rejects exact position overlaps. Its passing result therefore does not prove a strict actual-speed limit or continuous observed clearance between samples. The observed minimum separation was 1.5 m in these runs; continuous reference separation is certified separately by the trajectory planner. GUI interpolation cannot repair the backend timing guarantee.

## Optimization history

| 400-drone backend | World schedule | Mean RTF | Received pose batches/s | Outcome |
| --- | ---: | ---: | ---: | --- |
| [Physics-enabled static pose commands](artifacts/phase6-400-before-optimization.json) | 1000 Hz | 0.1373 | 4.10 | Initial formation timed out after 90.28 s |
| [Physics-enabled, reduced schedule](artifacts/phase6-400-120hz-physics.json) | 120 Hz | 0.2490 | 7.47 | Initial formation timed out after 90.29 s |
| [Direct ECM Pose, Physics disabled](artifacts/phase6-400.json) | 120 Hz | 0.9938 | 29.77 | Image and star completed; 76.10 s total |

Reducing the schedule alone did not resolve the cost. The fleet now omits the unused Physics system and writes owned model Pose components directly, marking periodic changes for scene clients. Gazebo still advances its clock and provides actual world-pose readback. The separate one-drone velocity demo retains Physics. The timed-out runs did not complete the same workload as the optimized run, so this is a diagnostic history rather than a controlled equal-work speedup benchmark. [Implementation and runtime findings](artifacts/runtime-fixes.md).

An earlier full-capacity application was OOM-killed in a 2 GiB/2-CPU Colima VM. The user approved resizing to 4 CPUs/6 GiB and restoration of the six existing service containers. The successful optimized 400-drone run peaked at about 2.98 GiB for its container alone. **4 CPUs/6 GiB remains a practical VM recommendation, not a measured minimum**, especially when sharing the VM with other workloads.

## Native CPU algorithm benchmark

[The core benchmark](artifacts/core-performance.json) ran on macOS arm64, Python 3.10.13 and NumPy 2.2.6. It excludes ROS, Gazebo and rendering.

| Drones | Image processing (ms) | Assignment/planning (ms) | Controller step p95 (ms) | Snapshot JSON p95 (ms) |
| --- | ---: | ---: | ---: | ---: |
| 100 | 139.819 | 1.946 | 0.0106 | 0.265 |
| 200 | 21.525 | 5.361 | 0.0108 | 0.513 |
| 400 | 28.768 | 28.406 | 0.0157 | 1.057 |

The first image call includes lazy-import/warm-up overhead, so the image-processing column is not a controlled scaling comparison. The core benchmark also uses its own FleetConfig defaults; these are not the full-system workload settings above. The planner's speed and separation certificates do not establish ROS delivery or rendering performance.

## Full HTTP application acceptance

The full HTTP-controlled workflow passed **all 11 checks, with no skips**, in **87.03 seconds** across 833 state polls. It began with 400 active drones/capacity 400 and ended connected in IDLE after a 400 → 20 → 400 count cycle. The test exercised takeoff/circle, moving pause/resume, sample upload and preview, image formation with 177 distinct commanded colors, pulse animation, animation pause, spiral transition, reset/image retention and invalid-request rejection. Both pause checks measured **0.000 m drift**. Pausing freezes mission time and commanded RGB while Gazebo time continues. [Raw API acceptance report](artifacts/api-acceptance.json).

| API-run metric | Recorded value |
| --- | ---: |
| Mean real-time factor | 0.993862 |
| Mean measured control rate | 29.8313 Hz |
| Mean reported pose rate | 29.8080 Hz |
| Minimum observed separation | 1.500000 m |
| Maximum observed tracking error | 0.333321 m |
| Maximum speed between HTTP-observed poses | 5.364290 m/s |
| Maximum reported consecutive pose-batch speed | 8.075378 m/s |

The two speed maxima use different observation intervals. The HTTP test samples state about ten times per second; the runtime computes its batch metric from consecutive Gazebo pose observations. **Both exceed the configured 5 m/s reference limit**, by approximately 7.3% and 61.5% respectively. The API probe records these speeds without imposing the show probe's 1.3× speed threshold. Its pass does not establish a strict actual-speed bound.

The API check enforces configured separation at its observed samples, allowing 0.00001 m floating-point tolerance. Position/speed metrics exclude explicit scene-reset grace periods. RGB values in this report are controller commands; independent Gazebo LED material validation remains separate. Successful image processing and workflow completion establish functional responsiveness, but no per-request latency distribution was measured.

## Native desktop and Gazebo LED acceptance

The final native PySide6/Qt WebEngine workflow passed **6/6 exercised UI checks** in **66.20 seconds** with 400 connected drones. The test clicked Start, reached the measured circle, processed the sample with both previews loaded, applied the image formation and image colors, and exercised Front/Fit camera controls. Final target errors were **1.301 mm for circle** and **0.813 mm for image**, with 177 distinct commanded RGB values, healthy completed state and no JavaScript console errors. [Desktop report](artifacts/desktop-workflow.json), [captured workspace](artifacts/desktop-workflow.png).

The viewer reported **60 measured render FPS at the final observation**. This is a native Qt WebEngine reading during this run, not a sustained frame-rate distribution, a separate browser measurement or a guarantee across hardware. The final observation also recorded 1.519022 m minimum separation, 0.999972 real-time factor and 29.9992 Hz control/pose rates. The 1512 × 949 desktop window produced a 3024 × 1898 Retina screenshot. Visual inspection confirmed the revised RGB glows and uncropped source/point previews, without overlapping or clipped interface elements.

The independent Gazebo material probe also passed. It verified ownership of `drone_001/base_link/show_light` (visual entity 50), then read actual ECS Material bytes for red `[1, 0, 0, 1]` and blue `[0, 0, 1, 1]` emissive RGBA on that same visual. The material hashes differed, and original image lighting was restored. This validates the live Gazebo material update path independently of controller RGB and the Three.js display; it samples one owned visual, not every LED. [Gazebo LED report](artifacts/gazebo-leds.json).

## Measurement coverage

| Measurement | Status |
| --- | --- |
| Native PySide6 visible workflow acceptance | Passed: 6/6 exercised checks |
| Native Qt WebEngine measured render FPS | 60 at final observation; sustained distribution not measured |
| Separate browser measured render FPS | Not measured |
| Independent Gazebo LED material update | Passed: actual red/blue on the same owned visual |
| Final contrast/preview refinement | Passed: repeated full workflow and visual inspection |
| Full HTTP application workflow, including image processing | Passed: 11/11 checks, no skips |
| HTTP command latency distribution | Not measured |

Measure sustained animation-frame timing in the actual viewer with the backend connected and record viewport, active count/capacity, renderer and test duration. A final 60 FPS reading, headless pose rate or CPU benchmark is not a substitute for that distribution.

## Reproduce

Build the current image, then run a headless image/star workload and retain its report:

```bash
docker build -t drone-swarm:latest -f docker/Dockerfile .
mkdir -p artifacts
docker run --rm -v "$PWD/artifacts:/reports" \
  -e SWARM_TEST_MODE=show -e SWARM_TEST_COUNT=400 \
  -e SWARM_TEST_LOG_DIR=/reports/performance-400-logs \
  drone-swarm:latest bash scripts/run_integration.sh \
  --ros-args -p formations:=image,star \
  -p image_path:=/ws/src/drone_swarm_simulator/assets/sample.png \
  -p report_path:=/reports/performance-400-rerun.json -p timeout:=300.0
.venv/bin/python scripts/benchmark_core.py --output artifacts/core-performance-rerun.json
```

Use counts 100 and 200 with distinct output paths for smaller runs. This command reproduces the workload while preserving the original evidence files; see [DEVELOPMENT.md](DEVELOPMENT.md) for test semantics and other integration modes.

For the full API workflow, start a fresh 400-capacity application in IDLE and run from the host:

```bash
bash scripts/start.sh --headless
.venv/bin/python scripts/test_api_live.py --url http://127.0.0.1:8765 \
  --report artifacts/api-acceptance-rerun.json
```

The API test controls the running show and leaves it connected in IDLE with 400 drones. Run it separately from desktop/LED control tests. `--skip-long` deliberately omits flight/animation checks and is not full acceptance.
