import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context):
    """Deferred setup so we can read launch arg values as Python strings."""
    pkg = get_package_share_directory("full_arm_ros2")
    mini_arm_pkg = get_package_share_directory("mini_arm_ros2")
    table_pkg = get_package_share_directory("table")
    xacro_path = os.path.join(pkg, "urdf", "full_arm_ros2.urdf.xacro")
    controllers_yaml = os.path.join(pkg, "config", "combined_controllers.yaml")

    # Compile xacro with mock hardware (same as combined.launch.py)
    robot_description = subprocess.check_output(
        ["xacro", xacro_path, "sim:=true"],
        text=True,
    )

    # Replace package:// URIs with file:// for Gazebo mesh loading
    mesh_dir = os.path.join(pkg, "meshes")
    robot_description_gz = robot_description.replace(
        "package://full_arm_ros2/meshes/",
        "file://" + mesh_dir + "/",
    )
    robot_description_gz = robot_description_gz.replace(
        "package://mini_arm_ros2/",
        "file://" + mini_arm_pkg + "/",
    )

    rviz_config = os.path.join(pkg, "config", "full_arm_ros2.rviz")

    # Resource paths for Gazebo mesh/model resolution
    models_dir = os.path.join(table_pkg, "models")
    resource_path = os.path.dirname(pkg)
    resource_dirs = os.pathsep.join([models_dir, resource_path])
    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=resource_dirs)
    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=resource_dirs)

    # --- Nodes (same control pattern as combined.launch.py) ---

    # Standalone controller manager with mock hardware
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
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
            "-string", robot_description_gz,
            "-name", "full_arm_ros2",
            "-world", LaunchConfiguration("world"),
        ],
        output="screen",
    )

    # Controller spawners (event-chained like combined.launch.py)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
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

    # Conditionally launch Gazebo (skip when world is already running)
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

    # Chain spawners: arm + diff_drive + rviz start after joint_state_broadcaster
    delay_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz],
        )
    )

    return [
        set_ign_resource,
        set_gz_resource,
        gz_sim_conditional,
        clock_bridge,
        camera_bridge,
        robot_state_publisher,
        control_node,
        TimerAction(period=3.0, actions=[spawn_entity]),
        joint_state_broadcaster_spawner,
        delay_arm,
        delay_diff_drive,
        delay_rviz,
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
