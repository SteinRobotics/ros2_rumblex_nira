# Nira launch commands

Source ROS and this workspace as described in [BUILD.md](BUILD.md).
All commands select Nira automatically. Run only one complete stack at a time.
Use verified resource limits for simulation and tests.

```bash
# Hardware (navigation is opt-in)
ros2 launch nira_bringup target_launch.py
ros2 launch nira_bringup target_launch.py enable_navigation:=true
# Offline desktop stack; no servo connection
ros2 launch nira_bringup test_launch.py
# Gazebo mesh stack
ros2 launch nira_bringup test_gazebo_launch.py
# Model inspection
ros2 launch nira_description display.launch.py
ros2 launch nira_description display_mesh.launch.py
# Individual components
ros2 launch nira_movement movement_offline_launch.py
ros2 launch nira_communication communication_launch.py
# servo gui
ros2 launch nira_movement movement_launch.py            # on robot
ros2 launch nira_movement servo_driver_gui.launch.py    # on remote pc
```

Use `--show-args` on an entry point for its supported options. Hardware launches leave
speech listening disabled unless `autostart_listening:=true` is requested.

Hardware camera capture is opt-in with `enable_camera:=true camera_device:=/dev/v4l/by-id/<device>`. The desktop stack enables camera and face preview by default; disable both with `enable_camera:=false enable_face_tracking:=false` when no camera is available. Face preview remains an offline feature; it does not enable physical face tracking.
