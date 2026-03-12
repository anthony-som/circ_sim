from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("sim", default_value="true", description="Enable simulation mode"),
    ]

    gui = LaunchConfiguration("gui")
    sim = LaunchConfiguration("sim")

    # URDF
    urdf_file = PathJoinSubstitution([FindPackageShare("full_arm_ros2"), "urdf", "full_arm_ros2.urdf"])
    robot_description_content = Command([FindExecutable(name="xacro"), " ", urdf_file, " sim:=", sim])
    robot_description = {"robot_description": robot_description_content}

    # Controllers YAML
    robot_controllers = PathJoinSubstitution([FindPackageShare("full_arm_ros2"), "config", "diffbot_controllers.yaml"])

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(gui),
        arguments=["-d", PathJoinSubstitution([FindPackageShare("full_arm_ros2"), "config", "full_arm_ros2.rviz"])],
    )

    # Delay the robot_controller until joint_state_broadcaster finishes
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay RViz until joint_state_broadcaster finishes
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)



