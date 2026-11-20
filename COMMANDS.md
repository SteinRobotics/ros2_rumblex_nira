# Nira ROS 2 topic commands

## Inspect topics

```bash
ros2 topic list -t
ros2 topic echo --once /servo_status
ros2 topic echo --csv /servo_status
ros2 topic echo --once --qos-durability transient_local /movement_name
ros2 topic echo --once --qos-durability transient_local /body_pose_actual
ros2 topic echo --once /joint_states
```

## Trigger named sequences

The [virtual joystick](nira_teleop/readme.md) publishes the exact sequence name
as `std_msgs/msg/String` on `/sequence_request`. Send the same requests from a
terminal with the brain running:

```bash
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'standup'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'laydown'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'neutral'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'look_left'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'look_right'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'leg_wave'}"
ros2 topic pub --once /sequence_request std_msgs/msg/String "{data: 'clap'}"
```

Replace `data` with any exact `name` from the installed
[sequence catalog](nira_brain/config/sequences). The brain uses normal joystick
priority, except `standup`, which has high priority. Unknown names are logged and
ignored. Rebuild `nira_brain` and restart the brain and GUI after catalog changes;
see [BUILD.md](BUILD.md#incremental-builds).

## Toggle standing and walking pose features

On either PS3 controller, tap **A / Cross** to toggle Follow Face and **B / Circle**
to toggle Level Torso. Both start off. They can run together during stand-up,
standing hold, and walking. Other gaits pause the corrections until stand-up or walking resumes.

For hardware face tracking, start the head camera and pass its measured calibration URL.

```bash
ros2 launch nira_bringup target_launch.py enable_camera:=true camera_device:=/dev/v4l/by-id/YOUR_CAMERA camera_info_url:=file:///path/to/calibration.yaml
ros2 topic echo --once /perception/face_target
ros2 topic echo --once --qos-durability transient_local /brain/face_tracking_status
```

Leveling uses BNO055 orientation and the mounting transform. Inspect both with:

```bash
ros2 topic echo --once --qos-reliability best_effort /bno055/imu
ros2 run tf2_ros tf2_echo base_link imu_link
```

Configure IMU freshness, correction gain, deadband, angle and rate limits under
`level_torso` in `nira_brain/config/nira/gait.yaml`. The defaults cap corrections
at 8 degrees and 2 degrees per second. Missing or stale observations release the
affected correction. Fresh data resumes the feature while its switch is on.

## Send requests

Inject recognized speech into the running brain:

```bash
ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'steh auf'}"
ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'leg dich hin'}"
ros2 topic pub --once /speech_recognition_online std_msgs/msg/String "{data: 'lauf nach vorne'}"
```

Activate robot-name listening, or play a file from the configured music directory:

```bash
ros2 topic pub --once /request_listening std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /request_music std_msgs/msg/String "{data: 'musicfox_hot_dogs_for_breakfast.mp3'}"
```

Inject joystick input (omitted fields default to zero/false), or send velocity
commands to the brain. Linear velocity is in metres per second; angular velocity
is in radians per second:

```bash
# Stand up (D-pad up):
ros2 topic pub --once /joystick_request nira_interfaces/msg/JoystickRequest "{dpad_vertical: 1}"
# Cycle the gait mode:
ros2 topic pub --once /joystick_request nira_interfaces/msg/JoystickRequest "{button_start: true}"
# Walk with a turn:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.1}}"
# Stop:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

For continuous joystick movement, choose a walking gait with Start if necessary
(Start also cycles through pose mode). Positive vertical stick input means forward.
Run the publisher below, stop it with **Ctrl+C**, then send neutral input to stop
walking; stopping the publisher alone does not send a stop request:

```bash
ros2 topic pub --rate 10 /joystick_request nira_interfaces/msg/JoystickRequest \
  "{left_stick_vertical: 0.4}"
# After stopping the repeating publisher:
ros2 topic pub --once /joystick_request nira_interfaces/msg/JoystickRequest '{}'
```

`/cmd_movement` carries `nira_interfaces/msg/BodyPose`: head orientation,
torso pose, and six toe positions. Angles are in degrees and positions in metres;
toe order is right front, right middle, right back, left front, left middle,
left back. Inspect an existing target before constructing a complete custom pose:

```bash
ros2 topic echo --once /cmd_movement
```

Use offline movement separately when testing custom poses because the active brain
publishes its own targets.

For a direct hardware servo check with movement running and the brain stopped,
send an angle in degrees using a name from Nira's
`nira_movement/config/nira/servo_description.yaml`:

```bash
ros2 topic pub --once /single_servo_request nira_interfaces/msg/ServoAngle \
  "{name: 'LEG_RIGHT_BACK_COXA', angle_deg: 0.0}"
```

## Direct servo maintenance and status testing

With movement running and the brain stopped, read a servo ID (`cmd: 14`) or
change it (`cmd: 13`, new ID in `data1`). Names must exist in Nira's
servo description. Choose an unused ID on the bus; after changing it, update the
matching `servo.serial_ids` entry in
`nira_movement/config/nira/servo_description.yaml` before restarting movement.
The example below changes the left front coxa ID to 10; replace 10 with the
intended unused ID for your robot.

```bash
ros2 topic pub --once /servo_direct_request nira_interfaces/msg/ServoDirectRequest \
  "{name: 'LEG_LEFT_FRONT_COXA', cmd: 14}"
ros2 topic pub --once /servo_direct_request nira_interfaces/msg/ServoDirectRequest \
  "{name: 'LEG_LEFT_FRONT_COXA', cmd: 13, data1: 10, data2: 0}"
```

Read results appear in the movement node's log. The direct-request callback
currently handles only ID read/write (14/13) and LED read/write (34/33), even
though the message defines additional command constants.

For testing status consumers, publish a synthetic status with the real movement
publisher stopped. This does not command a servo or read hardware:

```bash
ros2 topic pub --once /servo_status nira_interfaces/msg/ServoStatus \
  "{servo_error_code: 'LEG_LEFT_FRONT_COXA', error_code: 0,
    servo_max_temperature: 'LEG_LEFT_FRONT_COXA', max_temperature: 40.0,
    servo_max_voltage: 'LEG_LEFT_FRONT_COXA', max_voltage: 12.0,
    servo_min_voltage: 'LEG_LEFT_FRONT_COXA', min_voltage: 12.0}"
```

## Gazebo joint controller

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0]}"
```
