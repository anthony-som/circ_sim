import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    """Deferred setup so we can read launch arg values as Python strings."""
    pkg = get_package_share_directory("full_arm_ros2")
    mini_arm_pkg = get_package_share_directory("mini_arm_ros2")
    table_pkg = get_package_share_directory("table")
    xacro_path = os.path.join(pkg, "urdf", "full_arm_ros2.urdf.xacro")

    # Compile xacro with sim:=gz so IgnitionSystem and the Gazebo plugin are active
    robot_description = subprocess.check_output(
        ["xacro", xacro_path, "sim:=gz"],
        text=True,
    )

    # Replace package:// URIs with file:// for Gazebo mesh loading
    mesh_dir = os.path.join(pkg, "meshes")
    robot_description = robot_description.replace(
        "package://full_arm_ros2/meshes/",
        "file://" + mesh_dir + "/",
    )
    robot_description = robot_description.replace(
        "package://mini_arm_ros2/",
        "file://" + mini_arm_pkg + "/",
    )

    # Resolve $(find full_arm_ros2) for controllers.yaml path in the Gazebo plugin
    robot_description = robot_description.replace(
        "$(find full_arm_ros2)",
        pkg,
    )

    # Read spawn position from launch args
    x = context.launch_configurations.get("x", "-2.8")
    y = context.launch_configurations.get("y", "-0.656")
    z = context.launch_configurations.get("z", "1.12")
    # yaw = context.launch_configurations.get("yaw", "3.1416")

    rviz_config = os.path.join(pkg, "config", "full_arm_ros2.rviz")

    # Resource paths for Gazebo mesh/model resolution
    models_dir = os.path.join(table_pkg, "models")
    resource_path = os.path.dirname(pkg)
    resource_dirs = os.pathsep.join([models_dir, resource_path])
    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=resource_dirs
    )
    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=resource_dirs
    )

    # Publish robot description so Gazebo can spawn from the topic
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # Spawn from the topic (like mini_arm) so RSP and Gazebo use the same description
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "full_arm_ros2",
            "-world",
            LaunchConfiguration("world"),
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            # "-Y",
            # yaw,
        ],
        output="screen",
    )

    # Controller spawners with long timeouts (Gazebo's ign_ros2_control plugin
    # creates the controller manager; it needs time to come up)
    spawner_controllers = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "--controller-manager /controller_manager "
            "--controller-manager-timeout 120 "
            "&& "
            "ros2 run controller_manager spawner arm_controller "
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

    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/arm_camera/image_raw"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Conditionally launch Gazebo
    world_file = os.path.join(table_pkg, "worlds", "my_world.sdf")
    gz_sim_conditional = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
        condition=IfCondition(LaunchConfiguration("launch_gazebo")),
    )

    return [
        set_ign_resource,
        set_gz_resource,
        robot_state_publisher,
        gz_sim_conditional,
        clock_bridge,
        camera_bridge,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=15.0, actions=[spawner_controllers]),
        TimerAction(period=22.0, actions=[rviz]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value="empty", description="Gazebo world name"
            ),
            DeclareLaunchArgument(
                "launch_gazebo",
                default_value="true",
                description="Set to false if Gazebo is already running",
            ),
            DeclareLaunchArgument("x", default_value="2.0"),
            DeclareLaunchArgument("y", default_value="-0.656"),
            DeclareLaunchArgument("z", default_value="2.3"),
            # DeclareLaunchArgument("yaw", default_value="0.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
