from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nira_teleop",
                executable="node_teleop_simulated",
                name="node_teleop_simulated",
                output="screen",
            )
        ]
    )
