import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_table = get_package_share_directory('table')
    base_launch = os.path.join(pkg_table, 'launch', 'world.launch.py')
    vending_world = os.path.join(pkg_table, 'worlds', 'vending_task_world.sdf')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'world': vending_world}.items(),
        ),
    ])
