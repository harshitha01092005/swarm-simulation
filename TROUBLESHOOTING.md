# Troubleshooting

## Application does not start

Run from the repository and check the Docker service and selected context:

```bash
docker info
docker context show
docker ps
```

Start your installed Docker runtime if it is stopped. For a missing Qt environment, run `bash scripts/install_desktop.sh`. `bash scripts/start.sh --browser` uses the same interface without native PySide6; `--headless` exposes the service without opening a viewer.

If startup exceeds its readiness timeout, inspect the application logs:

```bash
docker logs --tail 100 drone-swarm-app
curl --fail http://127.0.0.1:8765/api/state
```

An HTTP response proves the web service is available, not that Gazebo feedback is ready. The launcher waits for `connected: true`, which requires fresh poses, clock and bridge endpoints. Startup failure/timeout logs are saved to `artifacts/last-session.log`.

The managed container is retained after exit, so `docker logs drone-swarm-app` remains available. The next start archives its logs and replaces the stopped container. Foreground examples and integration runs that explicitly use `--rm` remain disposable; preserve their output or mount an artifact directory.

## Container is OOM-killed or disappears during startup

The full 400-model world exceeded the original Colima VM's **2 GiB RAM/2 CPUs**: Docker recorded an `oom` event at 18:19:22 UTC during validation. The host has 36 GiB RAM and 11 CPUs, but the VM cannot use host resources beyond its allocation. A container exit or missing feedback after this event is a memory failure, not proof of a bad trajectory.

We recommend starting with **4 CPUs and 6 GiB RAM allocated to the VM**, leaving memory headroom for the host desktop and other workloads. This is not a measured minimum for the new Physics-disabled backend; the observed failure establishes that the earlier workload exceeded its 2 GiB allocation. The full-system launch allocates 400 models even when `--count 20` selects only 20 active drones. Use `swarm.launch.py count:=20 capacity:=20` for an actually smaller world; the complete command is in [Installation](INSTALLATION.md#smaller-worlds-on-constrained-machines).

For Colima, resizing requires stopping its VM. **This interrupts every container in that VM, including unrelated services.** Inspect and record the running containers first, arrange the interruption and restore those workloads afterward. The application scripts do not resize the VM. The following is a manual recovery example to run only after accounting for those services:

```bash
docker ps
colima stop
colima start --cpu 4 --memory 6
docker context use colima
docker info
```

Restart the containers that were running before the stop and verify them with `docker ps`. The acceptance record tracks subsequent runtime verification; resizing the VM alone does not establish that a show has passed.

## Source changes or launch options have no effect

Start reuses a running managed container only when it uses the current image. It refuses to continue with a running container from an older image. It does not rebuild an existing image or change a running container's count. Use:

```bash
bash scripts/stop.sh
docker build -t drone-swarm:latest -f docker/Dockerfile .
bash scripts/start.sh
```

The scripts refuse to replace or stop a container named `drone-swarm-app` unless it has the application's ownership label. Choose the existing container's normal management workflow if a different application owns that name. If port 8765 is already in use, identify the process before changing or stopping it.

Closing the viewer intentionally leaves Gazebo running. Use `stop.sh` when finished.

## View connects but no drones move

Press **Start show**; the standard application starts idle. Check status, feedback age and the connected drone count. Pause requires Resume, Stop holds position, and a completed formation remains stationary until another formation is applied.

Inspect ROS inside the same application container:

```bash
docker exec -it drone-swarm-app bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 topic echo /clock --once
ros2 topic echo /swarm/poses --once
ros2 topic echo /swarm/state --once
```

Missing clock indicates a simulator/bridge problem. Missing pose batches indicates a model, plugin or pose bridge problem. Read the earliest error in Docker logs. The viewer never substitutes desired positions for missing measured feedback.

The generated fleet world intentionally has no Physics system. Its plugin writes model Pose components directly and marks periodic changes; SimulationRunner still supplies the 120 Hz clock schedule. Adding Physics back changes this backend and reintroduces dynamics overhead. The legacy one-drone velocity demo still requires its own Physics system, so use the launch and topics matching the test.

## Commands are queued, rejected or time out

Formation and motion-setting edits during travel take effect at the next formation boundary. Resume a paused show to reach that boundary. Lighting and brightness changes can apply immediately. Stop the show before changing drone count or minimum separation; applying those changes resets the ground grid.

Image formation requires a successfully processed image. Targets extending below Z=0.5 m need greater altitude or smaller size. Transition time is a requested minimum: the planner extends it when the selected speed or separation needs more time. Dense artwork may gain depth layers, visible from the side.

HTTP errors report malformed input, a full command queue, controller shutdown or a response timeout. Wait for a pending command to resolve and inspect the state/error instead of repeatedly submitting commands. After a timeout, inspect the current state before retrying; work already executing may still have completed. Access the service through `localhost` or `127.0.0.1`; cross-origin or nonlocal Host requests are rejected.

Image processing uses a background worker with bounded pending work. A processing indicator can remain active while the simulation continues. Wait for the processed preview/acknowledgement; repeated uploads can fill the pending-image limit.

## Image processing fails or the preview is sparse

Use PNG or JPEG with a distinct foreground. Limits are 8 MiB, 16 megapixels and 8192 pixels per side, checked before decoding. Renaming another format's extension does not make it a valid input.

For a silhouette, try **Filled silhouette**, adjust the threshold and, if needed, invert foreground. For outlined artwork, try **Edge contours** and adjust sensitivity. Edge inversion often preserves the same geometry. Blank images and inputs with fewer usable points than drones return an error; the processor never fills the count by repeating positions.

The preview shows sampled source colors. Choose **Image colors** and apply settings to use them in the show. A black source detail can remain dark; brightness scales the selected colors rather than adding missing detail. Reduce count or choose clearer artwork if the sample lacks sufficient structure.

## Stale feedback or ERROR after pausing Gazebo

Use the application's **Pause/Resume** controls to hold a show. Pausing the Gazebo world freezes simulation time and eventually trips the two-second feedback/clock watchdog. The full application latches ERROR and stops advancing references. After unpausing Gazebo and restoring fresh feedback, use Reset; relaunch if recovery is unavailable or the clock moved backwards.

The legacy Phase 1 controller differs: its latched ERROR requires restarting its launch after Gazebo is unpaused. ROS Resume does not clear that legacy error. This behavior is independent of the product's mission pause.

## Blank viewport or desktop window failure

Try the browser view at http://127.0.0.1:8765. If it works while the desktop fails, check `.venv` and the PySide6/Qt WebEngine installation. If the page loads but the 3D canvas fails, inspect its displayed WebGL error and graphics support. A working HTTP page alone does not establish rendering success.

Use **Fit** or **Front** if the measured fleet is outside the camera view. Front view preserves the X/Z artwork; depth-layered targets look different from Top or an oblique view. The native viewer runs on the host and does not require a macOS-to-Linux display connection.

Gazebo's own `gui:=true` window is separate and requires a graphics-capable Linux desktop. For a driver diagnostic on Ubuntu, try `LIBGL_ALWAYS_SOFTWARE=1` before the ROS launch. Software rendering can be slow and must be identified in performance reports. Do not infer visible rendering from a headless integration pass.

## ROS package or Python module is missing

Run ROS/Gazebo inside the supplied image or the documented Ubuntu 24.04 environment. A native Mac Python environment does not provide Ubuntu `rclpy`. Source `/opt/ros/jazzy/setup.bash` and your colcon workspace overlay in each Linux terminal. The container overlay is `/ws/install/setup.bash`.

Rebuild the package after adding resources, and rebuild/install the C++ plugin after plugin changes. Missing `libswarm_fleet_system.so` points to its build/install or `GZ_SIM_SYSTEM_PLUGIN_PATH`; the default installed directory is `/usr/local/lib/drone_swarm`.

Use `python3 scripts/doctor.py` for a read-only report. Its nonzero result on the Mac host is expected when native ROS/Gazebo is absent. Keep the host desktop virtual environment separate from Ubuntu's system Python 3.12. Do not install ROS Python bindings from PyPI.

## Registry DNS or TLS errors

The initial Colima setup encountered a Docker Hub blob TLS error and DNS failures for alternative registries. A Colima restart restored registry resolution, and the active Docker context was restored to `colima`. The default Docker Hub image then built and real Gazebo tests passed. No custom CA, trust-store, DNS or mirror changes were needed.

For a recurrence, inspect resolution and HTTPS from the Docker daemon's VM, not only from the host browser. Check the selected context, VPN/proxy and VM resolver. Restarting Colima interrupts its existing containers, so inspect those workloads and arrange restoration before a restart; the project scripts do not restart the Docker VM.

If a verified organizational proxy requires a CA, obtain the approved certificate and configure the daemon VM's trust store using Docker/runtime guidance. A CA added inside this project's Dockerfile cannot fix a failure downloading its base image. Keep TLS verification enabled. [Docker CA guidance](https://docs.docker.com/engine/network/ca-certs/).

## Integration fails or performance is slow

Use the current rebuilt image and the mode appropriate to the test. Check `launch.log` and `smoke.log`; mount a host artifact directory when running a disposable container. The runner uses isolated discovery and cleans up its own process groups. A missing runtime is a failure, not a skipped pass.

Compare actual clock progress, received pose rate and controller rate. CPU allocation, concurrent containers and graphics rendering can affect wall time. The configured 30 Hz control timer and 120 Hz SimulationRunner schedule are targets; actual timing is measured. Check that the generated fleet world omits the Physics plugin and that the rebuilt direct-Pose fleet plugin is loaded. The UI's FPS is separate from Gazebo real-time factor and ROS pose rate. A CPU-only planning benchmark cannot establish 400-drone runtime or GUI performance.

When diagnosing speed, pair measured positions with `actual_simulation_time`. Using the controller's mission-time field for those intervals can create an apparent spike; the show probe was corrected to use observation timestamps. If a corrected measurement still exceeds the configured reference speed, preserve the report and investigate backend delivery timing rather than changing the reported metric.

Include the exact command, OS/architecture, Docker or native mode, active/capacity counts, image/settings, first error and preserved report in a bug report. Note whether the failure is in processing, planning, simulator feedback or rendering. Current acceptance results are indexed in [DEVELOPMENT_PROGRESS.md](DEVELOPMENT_PROGRESS.md).
