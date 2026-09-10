# Integration findings and fixes

During full-project acceptance on 2026-09-10:

- The initial speed probe divided observed movement by controller simulation time. It now uses the actual Gazebo pose timestamp. The corrected 100-drone run observed a maximum interval speed of 5.141 m/s for a 5 m/s reference limit (sampled transport timing); planned continuous velocity remains bounded by the configured speed.
- First-use OpenCV loading blocked the ROS executor for about 0.67 seconds. Image decoding, processing and preview encoding now run in a bounded background worker. OpenCV is loaded before control subscriptions and timers. Processed results commit on the ROS executor only if count/size/altitude still match.
- Docker recorded an `oom` event when the 400-model world started in the original 2 GiB / 2 CPU Colima VM. Colima was restarted with 4 CPUs and 6 GiB. Docker reports 4 CPUs and 6,195,118,080 usable bytes; unrelated local services were restored afterward.
- Startup now waits for measured fleet connectivity, preserves a stopped application container for diagnosis, and saves launch errors in `artifacts/last-session.log`.
- Review found queued-setting cancellation, full trajectory bounds, atomic rejection of invalid edits, and stale image metadata after recounting. Regression tests cover these fixes.

Final acceptance results are recorded separately in the phase and API reports.

## 400-drone performance resolution

The original Physics-based static pose command path achieved 0.137× real time
at 1,000 Hz and 0.249× at 120 Hz. Reducing the clock schedule alone did not
resolve its scaling cost. The generated fleet now disables only the Physics
system and updates its owned static model Pose components directly, marking
PeriodicChange for scene clients. SimulationRunner still advances the 120 Hz
Gazebo clock; feedback reads actual worldPose from the ECM in PostUpdate.
The separate velocity-controlled one-drone world still uses Physics.

The first optimized 400-drone run completed image and star formations in
76.10 seconds total, at mean RTF 0.9938 and 29.77 received pose batches/s.
Observed minimum separation was 1.5 m and final errors were 1.2/2.3 mm.
Container peak memory was 3055.5 MiB and mean CPU usage was 1.04 cores on the
4-CPU/6-GiB Colima VM. See phase6-400.json; previous slow reports are retained
as phase6-400-before-optimization.json and phase6-400-120hz-physics.json.

## Mixed-severity ROS command logging

Full HTTP acceptance rejected an active drone-count edit after accepting Start.
rclpy rejected switching severity at one dynamic logging call site, which then
stranded the rejected command future and produced HTTP 504. Accepted and rejected
events now use distinct source lines. Futures complete before rejection logging,
and command diagnostics have an independent stderr fallback. Three regressions
exercise the real rclpy logger and failure injection; all 85 Linux tests passed.
The original failed report is retained as api-logging-regression.json.
