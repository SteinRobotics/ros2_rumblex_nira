"""Select sensor drivers and optional processing from the robot profile."""

from pathlib import Path
import re
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_sensors(context):
    robot = "nira"
    profile = (
        Path(get_package_share_directory("nira_bringup"))
        / "config"
        / robot
        / "sensors.yaml"
    )
    if not profile.is_file():
        raise RuntimeError(f"No sensor profile for robot {robot!r}: {profile}")
    config = yaml.safe_load(profile.read_text())
    driver = (
        Path(get_package_share_directory(config["lidar_package"]))
        / "launch"
        / "lidar_launch.py"
    )
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver)),
            launch_arguments={}.items(),
            condition=IfCondition(LaunchConfiguration("enable_driver")),
        )
    ]
    if config.get("camera_package") and IfCondition(
        LaunchConfiguration("enable_camera")
    ).evaluate(context):
        camera = (
            Path(get_package_share_directory(config["camera_package"]))
            / "launch"
            / "camera_launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(camera)),
                launch_arguments={
                    "video_device": LaunchConfiguration("camera_device"),
                    "params_file": LaunchConfiguration("camera_params_file"),
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_driver", default_value="true"),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="false",
                description="Enable the camera when the robot profile provides one",
            ),
            DeclareLaunchArgument("camera_device", default_value=""),
            DeclareLaunchArgument("camera_params_file", default_value=""),
            DeclareLaunchArgument("camera_info_url", default_value=""),
            OpaqueFunction(function=_launch_sensors),
        ]
    )
