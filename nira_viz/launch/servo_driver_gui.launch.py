from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        Node(package="nira_viz", executable="servo_driver_gui_node",
             name="servo_driver_gui", namespace=LaunchConfiguration("namespace"),
             output="screen"),
    ])
