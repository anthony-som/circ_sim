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


def _prepare_urdf(pkg_path, urdf_rel, pkg_name, namespace):
    """Read a URDF, resolve paths, and inject a ROS namespace into the
    ign_ros2_control plugin so each robot gets its own controller_manager."""
    urdf_path = os.path.join(pkg_path, urdf_rel)
    with open(urdf_path, "r") as f:
        desc = f.read()

    # Resolve mesh package:// URIs to file:// for Gazebo
    mesh_dir = os.path.join(pkg_path, "meshes")
    desc = desc.replace(
        f"package://{pkg_name}/meshes/",
        "file://" + mesh_dir + "/",
    )

    # Resolve $(find ...) for the controllers.yaml path
    desc = desc.replace(f"$(find {pkg_name})", pkg_path)

    # Inject ROS namespace into the ign_ros2_control plugin so each robot
    # gets its own /ns/controller_manager instead of a shared root one.
    desc = desc.replace(
        "name=\"ign_ros2_control::IgnitionROS2ControlPlugin\">",
        "name=\"ign_ros2_control::IgnitionROS2ControlPlugin\">\n"
        f"      <ros><namespace>{namespace}</namespace></ros>",
    )

    # Give each ros2_control block a unique name to avoid SDF conflicts
    desc = desc.replace(
        "name=\"GazeboSimSystem\"",
        f"name=\"{namespace}_GazeboSimSystem\"",
    )

    # Rename the special "world" link so Gazebo doesn't anchor the model
    # to the world origin, allowing the spawn position to be respected.
    desc = desc.replace(
        '<link name="world"/>',
        f'<link name="{namespace}_anchor"/>',
    )
    desc = desc.replace(
        '<parent link="world"/>',
        f'<parent link="{namespace}_anchor"/>',
    )

    return desc


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────────────
    pkg_table = get_package_share_directory("table")
    pkg_asimov = get_package_share_directory("asimov")
    pkg_arm = get_package_share_directory("mini_arm_ros2")
    pkg_gz = get_package_share_directory("ros_gz_sim")

    # ── Resource paths so Gazebo can find all meshes and model:// URIs
    models_dir = os.path.join(pkg_table, "models")
    resource_dirs = os.pathsep.join([
        models_dir,
        os.path.dirname(pkg_asimov),
        os.path.dirname(pkg_arm),
    ])
    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=resource_dirs)
    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=resource_dirs)

    # ── Gazebo: launch the table world ─────────────────────────────
    world_file = os.path.join(pkg_table, "worlds", "my_world.sdf")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # Clock bridge (needed for use_sim_time)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # ── Button bridge & monitor (from table package) ───────────────
    button_bridge_args = []
    for i in range(1, 7):
        gz_topic = (
            f"/world/empty/model/ButtonPanel/model/Button_{i}"
            f"/link/link/sensor/button_contact/contact"
        )
        button_bridge_args.append(
            f"{gz_topic}@ros_gz_interfaces/msg/Contacts"
            f"[ignition.msgs.Contacts"
        )
    button_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=button_bridge_args,
        output="screen",
    )
    button_monitor = Node(
        package="table",
        executable="button_monitor.py",
        name="button_monitor",
        output="screen",
    )

    # ── Asimov (diff-drive base) ───────────────────────────────────
    asimov_desc = _prepare_urdf(
        pkg_asimov, "urdf/ASIMOV_FINAL_2.urdf", "asimov", "asimov")

    asimov_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="asimov",
        parameters=[
            {"robot_description": asimov_desc},
            {"use_sim_time": True},
        ],
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        output="screen",
    )

    asimov_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/asimov/robot_description",
            "-name", "asimov",
            "-z", "0.2",
            "-world", "empty",
        ],
        output="screen",
    )

    asimov_controllers = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "--controller-manager /asimov/controller_manager "
            "--controller-manager-timeout 120 "
            "&& "
            "ros2 run controller_manager spawner diff_drive_controller "
            "--controller-manager /asimov/controller_manager "
            "--controller-manager-timeout 120",
        ],
        output="screen",
    )

    # ── Mini arm ───────────────────────────────────────────────────
    arm_desc = _prepare_urdf(
        pkg_arm, "urdf/mini_arm.urdf", "mini_arm_ros2", "arm")

    arm_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="arm",
        parameters=[
            {"robot_description": arm_desc},
            {"use_sim_time": True},
        ],
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        output="screen",
    )

    # Spawn the arm on the table (table at x=-3 y=-0.656, surface z≈1.015)
    # Arm base_link origin is ~0.088m above the bottom of the mesh,
    # so z = 1.015 (table surface) + 0.1 (clearance) = 1.115
    arm_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/arm/robot_description",
            "-name", "mini_arm",
            "-x", "-2.8",
            "-y", "-0.656",
            "-z", "1.13",
            "-world", "empty",
        ],
        output="screen",
    )

    arm_controllers = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "--controller-manager /arm/controller_manager "
            "--controller-manager-timeout 120 "
            "&& "
            "ros2 run controller_manager spawner arm_forward_controller "
            "--controller-manager /arm/controller_manager "
            "--controller-manager-timeout 120",
        ],
        output="screen",
    )

    # ── Teleop: keyboard for arm (opens xterm) ─────────────────────
    arm_keyboard_teleop = ExecuteProcess(
        cmd=[
            "ros2", "run", "mini_arm_teleop", "keyboard_teleop",
            "--ros-args", "-p", "cmd_topic:=/arm/arm_forward_controller/commands",
        ],
        prefix="xterm -e",
        output="screen",
    )

    # ── Teleop: joy node for controllers ───────────────────────────
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "deadzone": 0.05,
            "autorepeat_rate": 30.0,
        }],
    )

    # ── RViz ───────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ── Sequence ───────────────────────────────────────────────────
    # t=0s   Gazebo world + clock bridge + resource paths + RSPs
    # t=5s   Spawn both robots into Gazebo
    # t=20s  Spawn controllers for both robots
    # t=28s  RViz + teleops
    return LaunchDescription([
        set_ign_resource,
        set_gz_resource,
        gz_sim,
        clock_bridge,
        button_bridge,
        button_monitor,
        # Robot state publishers start immediately
        asimov_rsp,
        arm_rsp,
        # Spawn robots into Gazebo (give world time to load)
        TimerAction(period=5.0, actions=[asimov_spawn]),
        TimerAction(period=8.0, actions=[arm_spawn]),
        # Spawn controllers after models are loaded
        TimerAction(period=20.0, actions=[asimov_controllers]),
        TimerAction(period=23.0, actions=[arm_controllers]),
        # Teleops + RViz after controllers are up
        TimerAction(period=30.0, actions=[
            rviz,
            joy_node,
            arm_keyboard_teleop,
        ]),
    ])
