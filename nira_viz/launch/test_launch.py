import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _face_preview(context):
    camera = IfCondition(LaunchConfiguration("enable_camera")).evaluate(context)
    tracking = IfCondition(LaunchConfiguration("enable_face_tracking")).evaluate(
        context
    )
    if not camera and (not tracking):
        return []
    if IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context):
        raise RuntimeError("Desktop camera preview requires use_sim_time:=false")
    if tracking and (not camera):
        raise RuntimeError("Face preview requires enable_camera:=true")
    actions = []
    if camera:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nira_camera"),
                            "launch",
                            "desktop_camera_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "camera_device": LaunchConfiguration("camera_device"),
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "camera_params_file": LaunchConfiguration("camera_params_file"),
                }.items(),
            )
        )
    if tracking:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nira_perception_face"),
                            "launch",
                            "face_tracking_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "desktop_preview": "true",
                    "publish_debug_image": LaunchConfiguration(
                        "enable_face_tracking_view"
                    ),
                }.items(),
            )
        )
        actions.append(
            Node(
                package="nira_camera",
                executable="face_preview.py",
                condition=IfCondition(LaunchConfiguration("enable_face_tracking_view")),
            )
        )
    return actions


def generate_launch_description():
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_teleop"), "/launch/teleop_simulated_launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("enable_simulated_teleop")),
    )
    hmi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_hmi"), "/launch/hmi_simulated_launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("enable_simulated_hmi")),
    )
    house_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_navigation"), "/launch/map_launch.py"]
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_communication"), "/launch/communication_launch.py"]
        ),
        launch_arguments={}.items(),
    )
    movement = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [FindPackageShare("nira_movement"), "/launch/movement_offline_launch.py"]
        ),
        launch_arguments={
            "joint_states_topic": LaunchConfiguration("movement_joint_states_topic")
        }.items(),
    )
    display_mesh = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("nira_viz"), "/launch/display_mesh.launch.py"]
        ),
        launch_arguments={
            "joint_state_publisher_gui": "false",
            "fixed_frame": "map",
            "joint_states_topic": "visualization_joint_states",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_display")),
    )
    brain = launch.actions.TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("nira_brain"), "/launch/brain_launch.py"]
                ),
                launch_arguments={
                    "autostart_listening": LaunchConfiguration("autostart_listening"),
                    "face_tracking_preview": LaunchConfiguration(
                        "enable_face_tracking"
                    ),
                }.items(),
            )
        ],
    )
    anatomy = PathJoinSubstitution(
        [FindPackageShare("nira_description"), "config", "nira", "anatomy.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "autostart_listening",
                default_value="false",
                description="Activate speech-to-text listening when the brain starts",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "enable_display",
                default_value="true",
                description="Launch the offline mesh state publisher and RViz",
            ),
            DeclareLaunchArgument(
                "movement_joint_states_topic", default_value="joint_states"
            ),
            DeclareLaunchArgument("publish_gazebo_map", default_value="false"),
            DeclareLaunchArgument(
                "start_x",
                default_value="-2.26",
                description="Initial map-frame x position (living-room center)",
            ),
            DeclareLaunchArgument(
                "start_y",
                default_value="-2.13",
                description="Initial map-frame y position (living-room center)",
            ),
            DeclareLaunchArgument(
                "start_yaw",
                default_value="0.0",
                description="Initial map-frame yaw in radians",
            ),
            DeclareLaunchArgument(
                "enable_simulated_teleop",
                default_value="true",
                description="Launch the mouse and keyboard controller GUI",
            ),
            DeclareLaunchArgument(
                "enable_simulated_hmi",
                default_value="true",
                description="Launch the simulated HMI GUI",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("nira_navigation"), "maps", "house_contour.yaml"]
                ),
                description="Occupancy map YAML file",
            ),
            DeclareLaunchArgument(
                "wall_height",
                default_value="1.0",
                description="Height of occupied map walls in metres",
            ),
            DeclareLaunchArgument(
                "simulate_lidar",
                default_value="true",
                description="Publish offline scan_1d readings from map walls; disable for a real lidar",
            ),
            DeclareLaunchArgument(
                "publish_test_map_tf",
                default_value="true",
                description="Estimate pose from supporting feet; disable when providing odometry/localization",
            ),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("enable_face_tracking", default_value="true"),
            DeclareLaunchArgument("enable_face_tracking_view", default_value="true"),
            DeclareLaunchArgument("camera_device", default_value=""),
            DeclareLaunchArgument("camera_info_url", default_value=""),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("nira_camera"),
                        "config",
                        "nira",
                        "camera_desktop.yaml",
                    ]
                ),
            ),
            OpaqueFunction(function=_face_preview),
            house_map,
            Node(
                package="nira_navigation",
                executable="test_map_walls.py",
                output="screen",
                parameters=[
                    {
                        "wall_height": ParameterValue(
                            LaunchConfiguration("wall_height"), value_type=float
                        ),
                        "simulate_lidar": ParameterValue(
                            LaunchConfiguration("simulate_lidar"), value_type=bool
                        ),
                        "publish_gazebo_map": ParameterValue(
                            LaunchConfiguration("publish_gazebo_map"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="nira_navigation", executable="room_labels.py", output="screen"
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="test_map_to_odom",
                output="screen",
                arguments=[
                    "--x",
                    LaunchConfiguration("start_x"),
                    "--y",
                    LaunchConfiguration("start_y"),
                    "--yaw",
                    LaunchConfiguration("start_yaw"),
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
                condition=IfCondition(LaunchConfiguration("publish_test_map_tf")),
            ),
            Node(
                package="nira_navigation",
                executable="offline_odometry.py",
                output="screen",
                parameters=[anatomy],
                condition=IfCondition(LaunchConfiguration("publish_test_map_tf")),
            ),
            communication,
            teleop,
            hmi,
            movement,
            Node(
                package="nira_movement",
                executable="node_offline_visualization",
                output="screen",
                parameters=[anatomy],
            ),
            display_mesh,
            brain,
        ]
    )
