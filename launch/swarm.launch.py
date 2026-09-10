"""Launch a generated drone fleet, one Gazebo bridge, and one ROS controller."""

import os
from pathlib import Path
import shlex
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_swarm.backends.fleet_world import render_world


def launch_setup(context):
    count = int(LaunchConfiguration("count").perform(context))
    capacity = int(LaunchConfiguration("capacity").perform(context))
    if not 1 <= count <= capacity <= 400:
        raise ValueError("count must be at least 1, capacity at least count, and capacity at most 400")
    gui = LaunchConfiguration("gui").perform(context) == "true"
    auto_start = LaunchConfiguration("auto_start").perform(context) == "true"
    web_enabled = LaunchConfiguration("web").perform(context) == "true"
    share = Path(get_package_share_directory("drone_swarm_simulator"))
    bridge_config = share / "config" / "fleet_bridge.yaml"
    if not bridge_config.is_file():
        raise RuntimeError(f"Missing fleet bridge configuration: {bridge_config}")
    with tempfile.NamedTemporaryFile(mode="w", suffix=".sdf", prefix="drone-fleet-", delete=False) as handle:
        handle.write(render_world(capacity, spacing=1.5))
        world_path = Path(handle.name)
    arguments = ["-r", "-v", "3"]
    if not gui:
        arguments.append("-s")
    arguments.append(str(world_path))
    gazebo_share = Path(get_package_share_directory("ros_gz_sim"))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": shlex.join(arguments), "gz_version": "8", "on_exit_shutdown": "true"}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="fleet_bridge",
        parameters=[{"config_file": str(bridge_config), "use_sim_time": True}], output="screen",
    )
    runtime = Node(
        package="drone_swarm_simulator", executable="fleet_runtime", name="fleet_runtime",
        parameters=[{
            "config_file": LaunchConfiguration("config_file").perform(context),
            "count": count, "capacity": capacity, "auto_start": auto_start,
            "web_enabled": web_enabled, "use_sim_time": True,
        }], output="screen",
    )

    def required_exited(event, launch_context):
        if not launch_context.is_shutdown:
            raise RuntimeError(f"Required fleet process exited with status {event.returncode}; inspect preceding logs")
        return []

    def remove_world(event, launch_context):
        world_path.unlink(missing_ok=True)
        return []

    plugin_path = "/usr/local/lib/drone_swarm"
    if os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH"):
        plugin_path += os.pathsep + os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"]
    return [
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        RegisterEventHandler(OnShutdown(on_shutdown=remove_world)),
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=required_exited)),
        RegisterEventHandler(OnProcessExit(target_action=runtime, on_exit=required_exited)),
        gazebo, bridge, runtime,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("count", default_value="5", description="Number of simulated drones (1–400)."),
        DeclareLaunchArgument("capacity", default_value=LaunchConfiguration("count"), description="Pre-spawned model capacity; count can change while stopped up to this limit."),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("auto_start", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("web", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("config_file", default_value=str(Path(get_package_share_directory("drone_swarm_simulator")) / "config" / "swarm.json")),
        OpaqueFunction(function=launch_setup),
    ])
