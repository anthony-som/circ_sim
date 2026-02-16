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

    rviz_config = os.path.join(pkg, "config", "mini_arm.rviz")

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        "world", default_value="empty",
        description="Gazebo world name (must match a running world, or SDF file to launch)",
    )
    headless_gz_arg = DeclareLaunchArgument(
        "launch_gazebo", default_value="true",
        description="Set to false if Gazebo is already running from another package",
    )

    # Set resource path so Gazebo can find the STL meshes
    # IGN_GAZEBO_RESOURCE_PATH for Fortress (Gazebo Sim 6.x),
    # GZ_SIM_RESOURCE_PATH for Garden+ (Gazebo Sim 7+)
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
    # This preserves <gazebo> plugin tags (including ign_ros2_control) that
    # ign sdf -p would strip during URDF-to-SDF conversion.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "mini_arm_ros2",
            "-z", "0.1",
            "-world", LaunchConfiguration("world"),
        ],
        output="screen",
    )

    # Spawn controllers strictly sequentially (JSB must finish before arm starts).
    # Running them as parallel Nodes can deadlock the CM's synchronous service
    # handling inside the Gazebo physics loop.
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
    actions = [
        world_arg,
        headless_gz_arg,
        set_ign_resource,
        set_gz_resource,
        robot_state_publisher,
    ]

    # Always include Gazebo launch — if already running from another package,
    # set launch_gazebo:=false and only the spawn + controllers run
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

    actions.append(gz_sim_conditional)
    actions.append(clock_bridge)
    actions.append(TimerAction(period=3.0, actions=[spawn_entity]))
    # Wait for Gazebo to fully load the model before spawning controllers
    actions.append(TimerAction(period=15.0, actions=[spawner_controllers]))
    actions.append(TimerAction(period=22.0, actions=[rviz]))
    return LaunchDescription(actions)
