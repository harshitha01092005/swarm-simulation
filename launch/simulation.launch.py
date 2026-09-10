"""Launch the Phase 1 world, one quadrotor, ROS bridge, and optional demo."""

from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    share = Path(get_package_share_directory("drone_swarm_simulator"))
    gazebo_share = Path(get_package_share_directory("ros_gz_sim"))
    world = share / "worlds" / "drone_swarm.sdf"
    model = share / "models" / "drone" / "model.sdf"
    bridge_config = share / "config" / "bridge.yaml"
    simulation_config = share / "config" / "simulation.yaml"
    for required in (world, model, bridge_config, simulation_config):
        if not required.is_file():
            raise RuntimeError(f"Missing installed simulation asset: {required}")

    # ros_gz_sim's launch file accepts a shell command string. Quote the installed
    # asset path so a workspace containing spaces is supported safely.
    gui = LaunchConfiguration("gui").perform(context) == "true"
    gz_args = ["-r", "-v", "3"]
    if not gui:
        gz_args.append("-s")
    gz_args.append(str(world))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": shlex.join(gz_args),
            "gz_version": "8",
            "on_exit_shutdown": "true",
        }.items(),
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="swarm_bridge",
        parameters=[{"config_file": str(bridge_config), "use_sim_time": use_sim_time}],
        output="screen",
    )
    spawner = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_drone_001",
        arguments=[
            "-world", "swarm_world",
            "-file", str(model),
            "-name", "drone_001",
            "-allow_renaming=false",
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
        output="screen",
    )
    controller = Node(
        package="drone_swarm_simulator",
        executable="drone_demo",
        name="drone_demo",
        parameters=[str(simulation_config), {"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("auto_demo")),
        output="screen",
    )
    startup = {"spawned": False}

    def on_spawn_exit(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError(
                f"Drone spawn failed (exit {event.returncode}). "
                "Check the Gazebo log and /world/swarm_world/create service."
            )
        startup["spawned"] = True
        return [LogInfo(msg="drone_001 spawned; simulator feedback is now available."), controller]

    def check_spawn_timeout(launch_context):
        if not startup["spawned"] and not launch_context.is_shutdown:
            raise RuntimeError(
                "Drone spawn timed out after 60 seconds. "
                "Check that Gazebo Harmonic started and loaded the UserCommands system."
            )
        return []

    def on_required_process_exit(event, launch_context):
        if not launch_context.is_shutdown:
            raise RuntimeError(
                f"A required simulation process exited (exit {event.returncode}); "
                "stopping Gazebo. See preceding process logs."
            )
        return []

    # Register handlers before starting any process: even an immediate spawn
    # failure must be observed. The demo is only added after a successful spawn.
    return [
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=on_spawn_exit)),
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=on_required_process_exit)),
        RegisterEventHandler(OnProcessExit(target_action=controller, on_exit=on_required_process_exit)),
        gazebo,
        bridge,
        spawner,
        TimerAction(period=60.0, actions=[OpaqueFunction(function=check_spawn_timeout)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true", choices=["true", "false"],
            description="Open the Gazebo window; false runs the simulator server only.",
        ),
        DeclareLaunchArgument(
            "auto_demo", default_value="true", choices=["true", "false"],
            description="Start the one-drone demonstration after successful spawning.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true"],
            description="Phase 1 motion is always driven by Gazebo simulation time.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
