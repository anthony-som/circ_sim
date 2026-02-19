import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context):
    """Deferred setup so we can read launch arg values as Python strings."""
    pkg = get_package_share_directory("mini_arm_ros2")
    urdf_path = os.path.join(pkg, "urdf", "mini_arm.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Replace package:// URIs with absolute file:// paths for Gazebo
    mesh_dir = os.path.join(pkg, "meshes")
    robot_description = robot_description.replace(
        "package://mini_arm_ros2/meshes/",
        "file://" + mesh_dir + "/",
    )

    # Resolve $(find mini_arm_ros2) for controllers.yaml
    robot_description = robot_description.replace(
        "$(find mini_arm_ros2)",
        pkg,
    )

    # Read spawn position from launch args
    x = context.launch_configurations.get("x", "-2.8")
    y = context.launch_configurations.get("y", "-0.656")
    z = context.launch_configurations.get("z", "1.12")

    # Set the world_fixed joint origin to the desired spawn position.
    # This keeps the "world" link (so Gazebo anchors the arm) while
    # positioning it at the correct location.
    robot_description = robot_description.replace(
        '<joint name="world_fixed" type="fixed">\n'
        '    <parent link="world"/>\n'
        '    <child link="base_link"/>',
        '<joint name="world_fixed" type="fixed">\n'
        f'    <origin xyz="{x} {y} {z}" rpy="0 0 1.5708"/>\n'
        '    <parent link="world"/>\n'
        '    <child link="base_link"/>',
    )

    rviz_config = os.path.join(pkg, "config", "mini_arm.rviz")

    resource_path = os.path.dirname(pkg)
    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=resource_path)
    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=resource_path)

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
            "-name", "mini_arm_ros2",
            "-world", LaunchConfiguration("world"),
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
            "ros2 run controller_manager spawner arm_forward_controller "
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

    from launch.conditions import IfCondition
    gz_sim_conditional = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-s -r empty.sdf"}.items(),
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
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty",
            description="Gazebo world name"),
        DeclareLaunchArgument("launch_gazebo", default_value="true",
            description="Set to false if Gazebo is already running"),
        DeclareLaunchArgument("x", default_value="-2.8"),
        DeclareLaunchArgument("y", default_value="-0.656"),
        DeclareLaunchArgument("z", default_value="1.12"),
        OpaqueFunction(function=_launch_setup),
    ])
