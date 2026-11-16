"""Launch Nira's lidar and optional head camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_driver", default_value="true"),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="false",
                description="Enable Nira's head camera",
            ),
            DeclareLaunchArgument("camera_device", default_value=""),
            DeclareLaunchArgument("camera_params_file", default_value=""),
            DeclareLaunchArgument("camera_info_url", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nira_lidar_2d"), "launch", "lidar_launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("enable_driver")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nira_camera"), "launch", "camera_launch.py"]
                    )
                ),
                launch_arguments={
                    "video_device": LaunchConfiguration("camera_device"),
                    "params_file": LaunchConfiguration("camera_params_file"),
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_camera")),
            ),
        ]
    )
