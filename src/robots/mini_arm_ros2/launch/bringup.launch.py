from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("mini_arm_ros2")
    urdf_path = os.path.join(pkg, "urdf", "mini_arm.urdf")
    controllers_path = os.path.join(pkg, "config", "controllers.yaml")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Swap Gazebo hardware plugin for mock_components so bringup works without Gazebo
    robot_description = robot_description.replace(
        "ign_ros2_control/IgnitionSystem",
        "mock_components/GenericSystem",
    )

    rviz_config = os.path.join(pkg, "config", "mini_arm.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_path],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_forward_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

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
    joystick_teleop = Node(
        package="mini_arm_teleop",
        executable="keyboard_teleop",
        name="joystick_arm_teleop",
        output="screen",
        parameters=[{
            "cmd_topic": "/arm_forward_controller/commands",
        }],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        spawner_jsb,
        spawner_arm,
        rviz,
        joy_node,
        TimerAction(period=1.0, actions=[joystick_teleop]),
    ])
