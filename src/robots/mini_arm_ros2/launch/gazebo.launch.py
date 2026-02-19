import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("mini_arm_ros2")
    urdf_path = os.path.join(pkg, "urdf", "mini_arm.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Replace package:// URIs with absolute file:// paths so Gazebo can
    # find meshes without needing IGN_GAZEBO_RESOURCE_PATH tricks.
    mesh_dir = os.path.join(pkg, "meshes")
    robot_description = robot_description.replace(
        "package://mini_arm_ros2/meshes/",
        "file://" + mesh_dir + "/",
    )

    # Resolve $(find mini_arm_ros2) so the ign_ros2_control plugin can
    # locate controllers.yaml at runtime.
    robot_description = robot_description.replace(
        "$(find mini_arm_ros2)",
        pkg,
    )

    # Rename the "world" link so Gazebo doesn't anchor the model to the
    # world origin — this lets the -x/-y/-z spawn position work correctly.
    robot_description = robot_description.replace(
        '<link name="world"/>',
        '<link name="arm_anchor"/>',
    )
    robot_description = robot_description.replace(
        '<parent link="world"/>',
        '<parent link="arm_anchor"/>',
    )

    rviz_config = os.path.join(pkg, "config", "mini_arm.rviz")

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        "world", default_value="empty",
        description="Gazebo world name (must match a running world)",
    )
    headless_gz_arg = DeclareLaunchArgument(
        "launch_gazebo", default_value="true",
        description="Set to false if Gazebo is already running from another package",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1")

    # Set resource path so Gazebo can find the STL meshes
    resource_path = os.path.dirname(pkg)
    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=resource_path,
    )
    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path,
    )

    # robot_state_publisher (ign_ros2_control needs this)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # Spawn the arm into Gazebo using the robot_description topic
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "mini_arm_ros2",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-world", LaunchConfiguration("world"),
        ],
        output="screen",
    )

    # Spawn controllers sequentially
    spawner_controllers = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "--controller-manager /controller_manager "
            "--controller-manager-timeout 120 "
            "&& "
            "ros2 run controller_manager spawner arm_forward_controller "
            "--controller-manager /controller_manager "
            "--controller-manager-timeout 120",
        ],
        output="screen",
    )

    # Bridge /clock from Gazebo so use_sim_time nodes get TF updates
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
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

    # Conditionally launch Gazebo or just spawn into existing world
    from launch.conditions import IfCondition
    actions = [
        world_arg,
        headless_gz_arg,
        x_arg, y_arg, z_arg,
        set_ign_resource,
        set_gz_resource,
        robot_state_publisher,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ),
            launch_arguments={"gz_args": "-s -r empty.sdf"}.items(),
            condition=IfCondition(LaunchConfiguration("launch_gazebo")),
        ),
        clock_bridge,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=15.0, actions=[spawner_controllers]),
        TimerAction(period=22.0, actions=[rviz]),
    ]

    return LaunchDescription(actions)
