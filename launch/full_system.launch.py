"""Launch the full 400-capacity simulator and local desktop HTTP interface."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('drone_swarm_simulator'))
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='400'),
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('auto_start', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/swarm.launch.py')),
            launch_arguments={'count': LaunchConfiguration('count'), 'capacity': '400',
                              'gui': LaunchConfiguration('gui'), 'auto_start': LaunchConfiguration('auto_start'),
                              'web': 'true'}.items()),
    ])
