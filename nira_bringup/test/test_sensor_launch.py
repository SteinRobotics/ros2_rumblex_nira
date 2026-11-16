"""Check robot pipeline selection without starting hardware or the vendor driver."""

import importlib.util
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]


def evaluate(ctx, value):
    return perform_substitutions(ctx, normalize_to_list_of_substitutions(value))


def load(package, name):
    spec = importlib.util.spec_from_file_location(
        name, ROOT / package / "launch" / f"{name}.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(driver="true", camera="false"):
    result = LaunchContext()
    result.launch_configurations.update(
        enable_driver=driver,
        enable_camera=camera,
        camera_device="/dev/v4l/by-id/test-meet-se",
        camera_params_file="",
        camera_info_url="",
    )
    return result


@pytest.mark.parametrize("driver", ["true", "false"])
@pytest.mark.parametrize("camera", ["true", "false"])
def test_nira_sensor_conditions_and_camera_overrides(driver, camera):
    description = load("nira_bringup", "sensors_launch").generate_launch_description()
    ctx = context(driver=driver, camera=camera)
    ctx.launch_configurations.update(
        camera_params_file="/tmp/camera.yaml",
        camera_info_url="file:///tmp/calibration.yaml",
    )
    actions = [
        a for a in description.entities if isinstance(a, IncludeLaunchDescription)
    ]
    assert len(actions) == 2
    for action, package, enabled in zip(
        actions, ("nira_lidar_2d", "nira_camera"), (driver, camera)
    ):
        location = evaluate(ctx, action.launch_description_source.location)
        assert package in location
        assert action.condition.evaluate(ctx) == (enabled == "true")
    arguments = dict(actions[1].launch_arguments)
    for argument, configuration in (
        ("video_device", "camera_device"),
        ("params_file", "camera_params_file"),
        ("camera_info_url", "camera_info_url"),
    ):
        assert (
            evaluate(ctx, arguments[argument])
            == ctx.launch_configurations[configuration]
        )


@pytest.mark.parametrize(
    "package,launch_name",
    [
        ("nira_bringup", "target_launch"),
    ],
)
def test_camera_arguments_forwarded_through_bringup(package, launch_name):
    description = load(package, launch_name).generate_launch_description()
    defaults = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(defaults)
    assert defaults.launch_configurations["enable_camera"] == "false"
    values = dict(
        enable_camera="true",
        camera_device="/dev/v4l/by-id/test-meet-se",
        camera_params_file="/tmp/camera.yaml",
        camera_info_url="file:///tmp/calibration.yaml",
    )
    defaults.launch_configurations.update(values)
    includes = [
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    forwarded = [
        dict(action.launch_arguments)
        for action in includes
        if "enable_camera" in dict(action.launch_arguments)
    ]
    assert len(forwarded) == 1
    for name, value in values.items():
        assert evaluate(defaults, forwarded[0][name]) == value


@pytest.mark.parametrize("robot", ["nira"])
def test_hardware_bringup_publishes_imu_mount_without_navigation(robot):
    description = load("nira_bringup", "target_launch").generate_launch_description()
    ctx = context()
    ctx.launch_configurations["enable_navigation"] = "false"
    publishers = [
        action
        for action in description.entities
        if isinstance(action, Node)
        and evaluate(ctx, action.node_package) == "robot_state_publisher"
    ]
    assert len(publishers) == 1
    publisher = publishers[0]
    assert publisher.condition is None
    parameters = evaluate_parameters(ctx, publisher._Node__parameters)
    model = ET.fromstring(parameters[0]["robot_description"])
    assert model.get("name") == robot
    joint = model.find("joint[@name='imu_joint']")
    assert joint is not None
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "base_link"
    assert joint.find("child").get("link") == "imu_link"
    assert model.find("ros2_control") is None


def test_navigation_has_no_sensor_or_head_controller():
    module = load("nira_navigation", "navigation_launch")
    with patch.object(
        module, "get_package_share_directory", side_effect=lambda name: str(ROOT / name)
    ):
        nodes = [
            action
            for action in module.generate_launch_description().entities
            if isinstance(action, Node)
        ]
        assert all("head_scan" not in str(node.node_executable) for node in nodes)
        assert all("lidar" not in str(node.node_package) for node in nodes)
    for robot in ["nira"]:
        config = yaml.safe_load(
            (
                ROOT / "nira_navigation" / "config" / robot / "navigation.yaml"
            ).read_text()
        )
        assert set(config) == {"node_navigation"}


def test_nira_uses_vendor_driver_and_urdf_frame():
    module = load("nira_lidar_2d", "lidar_launch")
    nodes = [
        action
        for action in module.generate_launch_description().entities
        if isinstance(action, Node)
    ]
    assert len(nodes) == 1  # No duplicate static transform or head controller.
    assert "ydlidar_ros2_driver" in str(nodes[0].node_package)
    config = yaml.safe_load((ROOT / "nira_lidar_2d/config/nira/lidar.yaml").read_text())
    assert (
        config["ydlidar_ros2_driver_node"]["ros__parameters"]["frame_id"]
        == "lidar_link"
    )
