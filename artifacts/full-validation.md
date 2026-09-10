# Full-application validation record

The optimized 100/200/400-drone headless runs, complete HTTP workflow, native desktop exercise, final visual rerun and independent Gazebo LED material checks have passed for version **1.0.0**. Strict actual-speed and sustained rendering guarantees remain outside the demonstrated results.

## Environment

The host is macOS 26.6.2 on Apple Silicon arm64 with 36 GiB RAM and 11 CPUs. Colima is allocated 4 CPUs and 6 GiB RAM. The runtime is Ubuntu 24.04 arm64, Python 3.12.3, ROS 2 Jazzy and Gazebo Harmonic 8.11.0. The native Python environment is 3.10.13.

The final tested code image is `sha256:c835bf64bf504ad7b0697c2212fdeabd3f2a1267e58788b5d28856a48dd21f67`.

The fleet explicitly disables the Physics system and updates owned static-model Pose components directly. Gazebo retains its 120 Hz SimulationRunner timing profile and actual ECM world-pose readback. The separate one-drone velocity demo retains Physics. The historical 2 GiB OOM and runtime fixes are documented in [runtime-fixes.md](runtime-fixes.md).

## Gate status

| Check | Result | Evidence / limits |
| --- | --- | --- |
| Native Python suite | 82 passed, 3 skipped; 85 discovered | Skipped tests require rclpy; see reproduction commands below |
| Ubuntu ROS Python suite | 85 passed, no skips | Includes the three actual-rclpy logging/error-path regressions |
| C++ protocol/material command CTests | 2/2 passed | Build-time tests; not a visible-material check |
| Optimized 100-drone image/star run | Passed | [scale-100.json](scale-100.json) |
| Optimized 200-drone image/star run | Passed | [scale-200.json](scale-200.json) |
| Optimized 400-drone image/star run | Passed | [phase6-400.json](phase6-400.json) |
| Full HTTP application workflow | 11/11 passed, no skips | [api-acceptance.json](api-acceptance.json) |
| Native desktop exercised workflow | 6/6 passed | [desktop-workflow.json](desktop-workflow.json), [screenshot](desktop-workflow.png); healthy completed image state |
| Native Qt WebEngine render FPS | 60 at final observation | Sustained distribution and separate browser FPS not measured |
| Independent Gazebo LED material validation | Passed | [gazebo-leds.json](gazebo-leds.json); actual red/blue ECS Material on the same owned visual; original lighting restored |
| Final contrast/preview refinement | Passed | Repeated full desktop workflow and visual inspection of revised RGB glows and uncropped previews |
| HTTP command latency distribution | Not measured | Functional API completion is verified separately |

The native skips are `test_actual_rclpy_accept_reject_accept_uses_separate_call_sites`, `test_broken_logger_does_not_change_command_results_or_strand_futures`, and `test_rejection_future_completes_before_unexpected_diagnostic_failure`. The Ubuntu suite executed and passed them. No native skipped test is counted as a native pass.

## Full HTTP acceptance

The run lasted 87.0348 seconds and made 833 state polls, beginning with 400 connected drones/capacity 400. All checks completed without `--skip-long`:

1. Initial connected IDLE state.
2. Malformed JSON, cross-origin commands and active-flight count changes rejected.
3. Moving pause froze observed positions, mission time and commanded colors; measured drift 0.000 m.
4. Resume completed the measured circle formation.
5. Sample upload returned a processed preview.
6. Image formation reached measured targets with 177 distinct commanded colors.
7. Pulse changed RGB while measured positions held.
8. Animation pause froze positions and RGB; measured drift 0.000 m.
9. Resumed animation transitioned to measured spiral formation.
10. Count change to 20 and reset restored the measured grid while retaining the image.
11. Count/capacity 400 returned to connected IDLE.

Pause freezes controller mission time; Gazebo's world clock continues. RGB evidence here comes from controller commands, not from querying Gazebo's visual materials. The final state was connected IDLE, count 400, capacity 400, circle selected.

Mean real-time factor was 0.993862; mean control and reported pose rates were 29.8313 Hz and 29.8080 Hz. Minimum observed separation matched the configured 1.500000 m, giving a minimum observed margin of 0 m. Maximum observed tracking error was 0.333321 m. Metrics exclude explicit scene-reset grace periods.

## Native desktop and independent LED acceptance

The final native desktop exercise ran for 66.2000 seconds with 400 measured drones. Six checks passed: WebGL connectivity, Start/circle completion, sample processing with both image previews, image formation completion, image color application and Front/Fit camera controls. Circle and image target errors were 0.001301 m and 0.000813 m. The final state was connected `FORMATION_COMPLETE`, with 177 distinct commanded colors, a 60 FPS viewer reading and no JavaScript console errors. Its final measured minimum separation was 1.519022 m, real-time factor 0.999972, and control/pose rates 29.9992 Hz. These are final observations, not run-wide averages.

The [final workspace screenshot](desktop-workflow.png) is a 3024 × 1898 Retina capture of the 1512 × 949 native window. Visual inspection confirmed the completed image, revised RGB glows, uncropped source/point previews and readable 400-drone telemetry without interface overlap or clipping.

The Gazebo LED probe completed in 12.681 seconds. It resolved the actual world and full parent chain to `drone_001/base_link/show_light` (visual entity 50), then observed emissive RGBA `[1, 0, 0, 1]` and `[0, 0, 1, 1]` from live ECS Material data on that same owned visual. The serialized material hashes differed. The original Image lighting was restored. This is independent backend material evidence, not a conclusion drawn from controller RGB or cached scene materials. One visual was sampled; this is not a complete per-LED audit.

## Limits that remain explicit

The continuous reference planner bounds speed at the configured 5 m/s. Discrete backend updates can exceed that limit over observed intervals: the API run recorded 5.364290 m/s between HTTP-observed poses and a maximum reported consecutive pose-batch speed of 8.075378 m/s. These are different sampling intervals; neither result is a strict actual-speed pass. The API probe records speed without enforcing a maximum.

The separate 400-drone show probe recorded 6.432481 m/s and permits up to 1.3× the reference limit. Its pass must be interpreted with that tolerance. Continuous reference separation is analytically certified; actual separation checks are observations at sampled times, not a continuous physical-clearance proof. See [PERFORMANCE.md](../PERFORMANCE.md) for all scale metrics and methodology.

The desktop's final 60 FPS reading does not establish sustained frame rate or separate-browser performance. No result establishes rotor dynamics or autopilot behavior. The passed desktop report uses `exercise: true` and verifies healthy measured state; an early capture-only result is not counted as workflow acceptance.

## Reproduce the completed suites

Run from the repository with the current image. The API command requires a fresh connected full application in IDLE and controls that application, so run it separately from desktop and LED control tests.

```bash
.venv/bin/python -m unittest discover -s tests -v
docker run --rm drone-swarm:latest python3 -m unittest discover -s tests -v
bash scripts/start.sh --headless
.venv/bin/python scripts/test_api_live.py --url http://127.0.0.1:8765 \
  --report artifacts/api-acceptance-rerun.json
.venv/bin/python scripts/test_desktop.py --exercise --output artifacts/desktop-workflow-rerun.png
.venv/bin/python scripts/test_gazebo_leds.py --report artifacts/gazebo-leds-rerun.json
```

The Docker build runs the two CTests. Full scale reproduction commands are in [DEVELOPMENT.md](../DEVELOPMENT.md) and [PERFORMANCE.md](../PERFORMANCE.md). The [phase index](../DEVELOPMENT_PROGRESS.md) remains the project-wide source for current acceptance status.
