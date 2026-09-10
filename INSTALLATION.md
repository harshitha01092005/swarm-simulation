# Installation

The recommended desktop arrangement runs the viewer on the host and ROS/Gazebo in Docker. It needs no Linux display forwarding. The simulator uses **Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic**, the documented compatible pairing. [Gazebo installation guide](https://gazebosim.org/docs/harmonic/ros_installation/).

## CPU and memory

For the full 400-model world, start with **4 CPUs and 6 GiB RAM allocated to Docker's Linux VM**, with at least 8 GiB of host memory headroom for the VM, desktop and other processes. This is a practical resource recommendation, not a measured minimum or a guaranteed frame rate. The development host has 36 GiB RAM and 11 CPUs; its original Colima allocation was only 2 GiB/2 CPUs. Docker recorded an OOM event for the full-capacity application at 18:19:22 UTC during validation.

`full_system.launch.py` always allocates 400 models, even with only 20 active drones. The launcher does not resize Docker or Colima automatically. See [troubleshooting](TROUBLESHOOTING.md#container-is-oom-killed-or-disappears-during-startup) before changing an existing VM that runs other containers. Completed runtime checks remain in [DEVELOPMENT_PROGRESS.md](DEVELOPMENT_PROGRESS.md).

## macOS desktop or browser

Install and start Docker Desktop or Colima, and make sure `docker info` succeeds. The native development environment uses **Python 3.10.13 on Apple Silicon**. Its exact dependencies are recorded in [requirements-desktop.txt](requirements-desktop.txt): NumPy 2.2.6, SciPy 1.15.3, OpenCV headless 4.12.0.88 and PySide6 6.11.2. PySide6 includes Qt WebEngine through its Addons package. These pins preserve Python 3.10 compatibility; they are not a policy of tracking every newest release. [PySide6 package](https://pypi.org/project/PySide6/6.11.2/), [OpenCV package](https://pypi.org/project/opencv-python-headless/4.12.0.88/).

Run in this repository:

```bash
python3 --version
docker info
bash scripts/install_desktop.sh
docker build -t drone-swarm:latest -f docker/Dockerfile .
bash scripts/start.sh
```

`install_desktop.sh` creates `.venv` using `python3` and installs the pinned desktop environment. Use the selected Python before creating it. Initial package installation and the Docker build require network access; application assets, including Three.js, are served locally afterward.

For the browser workflow, the desktop virtual environment and Qt are optional. The launcher still needs host `python3` for its readiness check:

```bash
docker build -t drone-swarm:latest -f docker/Dockerfile .
bash scripts/start.sh --browser
```

`--headless` starts only the service. `--count 100` changes the initial active count; the standard launch still pre-spawns capacity for 400. Open http://127.0.0.1:8765 from the same host. The Docker port is bound to loopback, and the HTTP API accepts local, same-origin requests.

```bash
bash scripts/stop.sh
```

This stops only the managed `drone-swarm-app` container. It does not stop Docker or Colima. Closing the desktop window does not shut down the backend. Rebuild the image after source changes, then stop and start the application to use the new image. Start refuses to reuse a running container from an older image, and does not apply new options to a running container.

The managed application container is created without `--rm`, preserving logs after an exit. On its next start, the script saves the stopped container's logs to `artifacts/last-session.log`, removes that stopped container and creates a replacement. Startup errors/timeouts also save this log. Readiness requires `/api/state` to report `connected: true`; an HTTP response alone is insufficient.

## Docker runtime directly

The Dockerfile builds the C++ Gazebo plugin, runs its CTests, and installs the `drone_swarm_simulator` package with colcon. The image uses Ubuntu's Python 3.12 and apt-provided NumPy 1.26.4, SciPy 1.11.4 and OpenCV 4.6.0. Keep this ROS interpreter separate from the host desktop virtual environment.

The generated fleet world retains Gazebo's 120 Hz SimulationRunner timing profile but omits the Physics system. Its plugin applies kinematic references directly to model Pose components and reads back Gazebo world poses. SceneBroadcaster, UserCommands and LED material updates remain available. The legacy one-drone world still loads Physics. This configuration does not yet establish a measured 400-drone real-time result; consult the acceptance reports.

```bash
docker run --rm -p 127.0.0.1:8765:8765 drone-swarm:latest \
  ros2 launch drone_swarm_simulator full_system.launch.py \
  count:=400 gui:=false auto_start:=false
```

This foreground form stops with `Ctrl+C` and, unlike the managed launcher, removes its container on exit because it uses `--rm`. The entrypoint sources `/opt/ros/jazzy/setup.bash` and `/ws/install/setup.bash`. The installed source directory is `/ws/src/drone_swarm_simulator`.

The default base is `ros:jazzy-ros-base-noble`. `ROS_BASE_IMAGE` is an optional build argument for an equivalent Jazzy/Noble image; the default Docker Hub source is the verified setup. Historical registry failures and their resolution are documented in [troubleshooting](TROUBLESHOOTING.md#registry-dns-or-tls-errors).

## Smaller worlds on constrained machines

Stop the full application before using the same HTTP port. This launches only 20 models, with a maximum active count of 20:

```bash
docker run --rm -p 127.0.0.1:8765:8765 drone-swarm:latest \
  ros2 launch drone_swarm_simulator swarm.launch.py \
  count:=20 capacity:=20 gui:=false web:=true auto_start:=false
```

Open http://127.0.0.1:8765 in a browser, or connect the installed desktop with `.venv/bin/python -m drone_swarm.gui.main_window`. To raise the active count beyond this capacity, relaunch with a larger capacity and suitable memory allocation. `start.sh --count 20` is different: its full-system launch keeps capacity at 400.

## Native Ubuntu 24.04

Run the following Bash commands **inside Ubuntu**, not on macOS. This path supports the web interface and, with working graphics, Gazebo's own GUI.

Install locale and repository prerequisites:

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl ca-certificates python3
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
```

Configure the ROS repository using its `ros2-apt-source` package, which maintains the repository and signing key. [Official ROS repository setup](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst).

```bash
export SWARM_ROS_APT_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')"
test -n "$SWARM_ROS_APT_VERSION"
curl -fL --retry 3 \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${SWARM_ROS_APT_VERSION}/ros2-apt-source_${SWARM_ROS_APT_VERSION}.noble_all.deb" \
  -o /tmp/drone-swarm-ros2-apt-source.deb
sudo dpkg -i /tmp/drone-swarm-ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-ros-gz \
  python3-colcon-common-extensions python3-rosdep python3-setuptools \
  python3-numpy python3-scipy python3-opencv python3-yaml python3-venv \
  build-essential cmake
source /opt/ros/jazzy/setup.bash
```

Initialize rosdep once with `sudo rosdep init`, then run `rosdep update` as your regular user. If its sources already exist, keep them and only update. The Jazzy `ros-gz` package installs the matching Harmonic vendor libraries; a separate Gazebo repository is unnecessary.

From this repository, build and install the Gazebo plugin:

```bash
cmake -S gazebo_plugin -B build/gazebo_plugin -DCMAKE_BUILD_TYPE=Release
cmake --build build/gazebo_plugin -j2
ctest --test-dir build/gazebo_plugin --output-on-failure
sudo cmake --install build/gazebo_plugin
```

The default install destination is `/usr/local/lib/drone_swarm`, which the launch adds to Gazebo's plugin search path. Next create a colcon workspace, linking this repository once:

```bash
export SWARM_SOURCE="$PWD"
mkdir -p "$HOME/drone_swarm_ws/src"
ln -s "$SWARM_SOURCE" "$HOME/drone_swarm_ws/src/drone_swarm_simulator"
cd "$HOME/drone_swarm_ws"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --packages-select drone_swarm_simulator
source install/setup.bash
ros2 launch drone_swarm_simulator full_system.launch.py count:=400 gui:=false
```

If the repository is already in a colcon workspace, use that workspace and omit the symlink step. Open http://127.0.0.1:8765 in your browser. Set `gui:=true` to additionally open Gazebo's GUI on a graphics-capable Ubuntu desktop. The PySide6 viewer can also connect to this local server after installing the desktop environment and running `.venv/bin/python -m drone_swarm.gui.main_window` from the repository.

Use Ubuntu system Python for ROS launch and colcon. Do not replace it with Homebrew/Conda Python or install `rclpy` from PyPI. See [DEVELOPMENT.md](DEVELOPMENT.md) for smoke tests. `python3 scripts/doctor.py` is a read-only prerequisites check; a nonzero result is expected on a Mac host without native ROS/Gazebo.
