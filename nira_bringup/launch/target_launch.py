import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution(
        [FindPackageShare("nira_description"), "urdf", "nira_mesh.urdf.xacro"]
    )
    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [FindExecutable(name="xacro"), " ", model, " use_sim:=false"]
                    ),
                    value_type=str,
                )
            }
        ],
    )
    communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_communication"), "/launch/communication_launch.py"]
        ),
        launch_arguments={}.items(),
    )
    hmi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_hmi"), "/launch/hmi_launch.py"]
        )
    )
    movement = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [FindPackageShare("nira_movement"), "/launch/movement_launch.py"]
        ),
        launch_arguments={}.items(),
    )
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_teleop"), "/launch/teleop_launch.py"]
        )
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_bringup"), "/launch/sensors_launch.py"]
        ),
        launch_arguments={
            "enable_camera": LaunchConfiguration("enable_camera"),
            "camera_device": LaunchConfiguration("camera_device"),
            "camera_params_file": LaunchConfiguration("camera_params_file"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
        }.items(),
    )
    face_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_perception_face"), "/launch/face_tracking_launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera")),
    )
    bno055_config = PathJoinSubstitution(
        [FindPackageShare("nira_bringup"), "config", "nira", "bno055_params.yaml"]
    )
    bno055 = Node(
        package="bno055",
        executable="bno055",
        name="bno055",
        output="screen",
        parameters=[bno055_config],
    )
    brain = launch.actions.TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("nira_brain"), "/launch/brain_launch.py"]
                ),
                launch_arguments={
                    "autostart_listening": LaunchConfiguration("autostart_listening")
                }.items(),
            )
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_navigation"), "/launch/navigation_launch.py"]
        ),
        launch_arguments={}.items(),
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "autostart_listening",
                default_value="false",
                description="Activate speech-to-text listening when the brain starts",
            ),
            DeclareLaunchArgument(
                "enable_navigation",
                default_value="false",
                description="Launch nira_navigation (Nira sensing + reactive nav)",
            ),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="false",
                description="Enable Nira's head camera",
            ),
            DeclareLaunchArgument("camera_device", default_value=""),
            DeclareLaunchArgument("camera_params_file", default_value=""),
            DeclareLaunchArgument("camera_info_url", default_value=""),
            robot_state,
            communication,
            hmi,
            movement,
            teleop,
            lidar,
            face_detector,
            bno055,
            brain,
            navigation,
        ]
    )
