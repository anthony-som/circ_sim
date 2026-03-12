from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    gui = LaunchConfiguration("gui")

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch RViz",
    )

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("full_arm_ros2"),
            "urdf",
            "full_arm_ros2.urdf",
        ]
    )

    from launch.substitutions import FindExecutable
    robot_description_content = Command([FindExecutable(name="xacro"), " ", urdf_path])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("full_arm_ros2"),
            "config",
            "full_arm_ros2.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(gui),
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            declare_gui,
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz_node,
        ]
    )


