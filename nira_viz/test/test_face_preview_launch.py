"""Verify desktop and calibrated robot detector launch paths."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

ROOT = Path(__file__).resolve().parents[2]


def module(name="test_launch", package="nira_viz"):
    spec = importlib.util.spec_from_file_location(
        name, ROOT / f"{package}/launch/{name}.py"
    )
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(robot="nira", camera="true", tracking="true", sim="false"):
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        robot=robot,
        enable_camera=camera,
        enable_face_tracking=tracking,
        use_sim_time=sim,
        enable_face_tracking_view="false",
        camera_device="",
        camera_params_file="/tmp/desktop.yaml",
    )
    return ctx


@pytest.mark.parametrize("robot,expected", [("nira", "true")])
def test_defaults_follow_robot(robot, expected):
    launch = module().generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations["robot"] = robot
    # Camera config path is resolved only when launching camera; no installed package needed here.
    for action in launch.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name in (
            "enable_camera",
            "enable_face_tracking",
        ):
            action.execute(ctx)
    assert ctx.launch_configurations["enable_camera"] == expected
    assert ctx.launch_configurations["enable_face_tracking"] == expected


def test_one_capture_and_one_detector_and_optional_view():
    actions = module()._face_preview(context())
    assert len(actions) == 3
    assert sum(isinstance(a, IncludeLaunchDescription) for a in actions) == 2
    assert dict(actions[1].launch_arguments)["desktop_preview"] == "true"
    assert not actions[2].condition.evaluate(context())


@pytest.mark.parametrize("ctx", [context(camera="false"), context(sim="true")])
def test_incompatible_preview_rejected(ctx):
    with pytest.raises(RuntimeError):
        module()._face_preview(ctx)


def test_disabled_does_not_resolve_camera_or_face_packages():
    assert module()._face_preview(context(camera="false", tracking="false")) == []


def test_target_detector_follows_camera_and_uses_calibrated_mode():
    launch = module("target_launch", "nira_bringup").generate_launch_description()
    ctx = context()
    ctx.launch_configurations["enable_navigation"] = "false"
    detector, = [action for action in launch.entities
                 if isinstance(action, IncludeLaunchDescription) and action.condition
                 and action.condition.evaluate(ctx)]
    detector.launch_description_source.get_launch_description(ctx)
    assert Path(detector.launch_description_source.location).name == "face_tracking_launch.py"
    assert not detector.condition.evaluate(context(camera="false"))
    assert "desktop_preview" not in dict(detector.launch_arguments)
