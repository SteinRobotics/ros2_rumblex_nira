from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = PathJoinSubstitution(
        [get_package_share_directory("nira_brain"), "config", "nira", "parameter.yaml"]
    )
    node = Node(
        package="nira_brain",
        name="node_brain",
        executable="node_brain",
        output="screen",
        parameters=[
            config,
            PathJoinSubstitution(
                [
                    get_package_share_directory("nira_brain"),
                    "config",
                    "nira",
                    "gait.yaml",
                ]
            ),
            PathJoinSubstitution(
                [
                    get_package_share_directory("nira_description"),
                    "config",
                    "nira",
                    "anatomy.yaml",
                ]
            ),
            {
                "face_tracking.desktop_preview": ParameterValue(
                    LaunchConfiguration("face_tracking_preview"), value_type=bool
                ),
                "autostart_listening": ParameterValue(
                    LaunchConfiguration("autostart_listening"), value_type=bool
                ),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("face_tracking_preview", default_value="false"),
            DeclareLaunchArgument(
                "autostart_listening",
                default_value="false",
                description="Activate speech-to-text listening when the brain starts",
            ),
            node,
        ]
    )
