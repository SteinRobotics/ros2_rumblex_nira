# Nira sensors

Sensor composition is owned by `nira_bringup/launch/sensors_launch.py`.
The BNO055 mount and measurement frame are defined in the robot URDF and
`nira_bringup/config/nira/bno055_params.yaml`.

`nira_lidar_2d` launches the external `ydlidar_ros2_driver` with `config/nira/lidar.yaml`, publishing planar `scan` data in `lidar_link`. Install the YDLidar SDK and SteinRobotics driver fork using `scripts/update_ydlidar_driver.sh`; the Raspberry Pi setup script includes these steps. Verify the serial port and tune navigation on hardware. The optional OBSBOT Meet SE camera uses `nira_camera`; camera calibration, device selection and desktop face preview are documented in that package and `nira_perception_face`.

```bash
ros2 launch nira_bringup sensors_launch.py
ros2 launch nira_navigation navigation_launch.py
```

Navigation does not launch sensor drivers itself. Hardware bringup composes the
sensor pipeline and enables navigation only when requested. The offline map-wall
range simulator is a single-beam approximation, not a physical sensor validation.
