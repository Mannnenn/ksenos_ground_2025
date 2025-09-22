from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "ksenos_ground"
    xacro_file_name = "ksenos_model.urdf"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher as a composable node
    robot_state_pub_component = ComposableNode(
        package="robot_state_publisher",
        plugin="robot_state_publisher::RobotStatePublisher",
        parameters=[robot_description],
        name="robot_state_publisher"
    )

    # Composable Node Container
    container = ComposableNodeContainer(
        name="robot_state_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            robot_state_pub_component,
        ],
        output="both",
    )

    nodes = [
        container,
    ]

    return LaunchDescription(nodes)