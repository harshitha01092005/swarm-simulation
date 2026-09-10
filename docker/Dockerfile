ARG ROS_BASE_IMAGE=ros:jazzy-ros-base-noble
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        python3-colcon-common-extensions \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-opencv python3-numpy python3-scipy \
    && rm -rf /var/lib/apt/lists/*
COPY gazebo_plugin /ws/src/drone_swarm_simulator/gazebo_plugin
RUN source /opt/ros/jazzy/setup.bash \
    && cmake -S /ws/src/drone_swarm_simulator/gazebo_plugin -B /tmp/fleet-plugin-build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/fleet-plugin-build -j1 \
    && ctest --test-dir /tmp/fleet-plugin-build --output-on-failure \
    && cmake --install /tmp/fleet-plugin-build
COPY . /ws/src/drone_swarm_simulator
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select drone_swarm_simulator \
    && chmod +x /ws/src/drone_swarm_simulator/docker/entrypoint.sh \
        /ws/src/drone_swarm_simulator/scripts/run_integration.sh

WORKDIR /ws/src/drone_swarm_simulator

ENTRYPOINT ["/ws/src/drone_swarm_simulator/docker/entrypoint.sh"]
CMD ["/ws/src/drone_swarm_simulator/scripts/run_integration.sh"]
