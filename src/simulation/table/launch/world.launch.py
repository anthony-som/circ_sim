import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_world = get_package_share_directory('table')

    # Let Gazebo find model:// URIs (table models) and package meshes
    models_dir = os.path.join(pkg_my_world, 'models')
    arm_mesh_pkg = get_package_share_directory('mini_arm_ros2')
    resource_dirs = os.pathsep.join([
        models_dir,
        os.path.dirname(arm_mesh_pkg),
    ])
    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_dirs,
    )
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_dirs,
    )

    # Path to your world file
    world_file = os.path.join(pkg_my_world, 'worlds', 'my_world.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Bridge contact sensor topics from Gazebo to ROS2
    button_bridge_args = []
    for i in range(1, 7):
        gz_topic = (
            f'/world/empty/model/ButtonPanel/model/Button_{i}'
            f'/link/link/sensor/button_contact/contact'
        )
        bridge_arg = (
            f'{gz_topic}@ros_gz_interfaces/msg/Contacts'
            f'[ignition.msgs.Contacts'
        )
        button_bridge_args.append(bridge_arg)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=button_bridge_args,
        output='screen',
    )

    button_monitor_node = Node(
        package='table',
        executable='button_monitor.py',
        name='button_monitor',
        output='screen',
    )

    return LaunchDescription([
        set_gz_resource,
        set_ign_resource,
        gz_sim,
        bridge_node,
        button_monitor_node,
    ])
