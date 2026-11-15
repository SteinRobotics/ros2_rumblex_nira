from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot = "nira"
    config = PathJoinSubstitution(
        [
            get_package_share_directory("nira_communication"),
            "config",
            robot,
            "params.yaml",
        ]
    )
    node = Node(
        package="nira_communication",
        name="node_communication",
        executable="node_communication",
        output="screen",
        parameters=[config],
    )
    return LaunchDescription([node])
