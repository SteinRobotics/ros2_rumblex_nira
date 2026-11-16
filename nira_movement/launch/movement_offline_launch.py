from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = [get_package_share_directory("nira_movement"), "config", "nira"]
    anatomy = PathJoinSubstitution(
        [
            get_package_share_directory("nira_description"),
            "config",
            "nira",
            "anatomy.yaml",
        ]
    )
    servo_description = PathJoinSubstitution(config_dir + ["servo_description.yaml"])
    node = Node(
        package="nira_movement",
        name="node_movement",
        executable="node_movement",
        output="screen",
        parameters=[anatomy, servo_description, {"servo.offline": True}],
        remappings=[("joint_states", LaunchConfiguration("joint_states_topic"))],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("joint_states_topic", default_value="joint_states"),
            node,
        ]
    )
