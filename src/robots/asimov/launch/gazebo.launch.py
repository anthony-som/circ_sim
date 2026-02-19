import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("asimov")
    urdf_path = os.path.join(pkg, "urdf", "ASIMOV_FINAL_2.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Replace package:// URIs with file:// paths for Gazebo mesh loading
    mesh_dir = os.path.join(pkg, "meshes")
    robot_description = robot_description.replace(
        "package://asimov/meshes/",
        "file://" + mesh_dir + "/",
    )

    # Resolve $(find asimov) for the ros2_control plugin
    robot_description = robot_description.replace(
        "$(find asimov)",
        pkg,
    )

    # Resource paths for Gazebo to find meshes
    resource_path = os.path.dirname(pkg)
    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=resource_path,
    )
    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "asimov",
            "-z", "0.2",
        ],
        output="screen",
    )

    spawner_controllers = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "--controller-manager /controller_manager "
            "--controller-manager-timeout 120 "
            "&& "
            "ros2 run controller_manager spawner diff_drive_controller "
            "--controller-manager /controller_manager "
            "--controller-manager-timeout 120",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        set_ign_resource,
        set_gz_resource,
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=15.0, actions=[spawner_controllers]),
        TimerAction(period=20.0, actions=[rviz]),
    ])
