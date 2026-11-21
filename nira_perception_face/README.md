# Nira face observations

C++23 ROS 2 face detector using the bundled YuNet ONNX model and OpenCV CPU
inference. Publishes `/perception/face_target` (`nira_interfaces/FaceTarget`)
from `/camera/image_raw` and `/camera/camera_info`. Desktop launch uses
`/camera/image_raw/compressed` instead, to reduce image transport losses. Target IDs describe temporary
box association, not identity. Three consistent detections acquire a target;
loss or ambiguous overlap invalidates it. Perception never sends servo commands.

Build with the resource controls in [BUILD.md](../BUILD.md), selecting
`nira_interfaces nira_perception_face` (and their dependencies on a fresh
workspace). ROS dependencies include `cv_bridge`, `sensor_msgs`, and OpenCV >=4.8.
Model provenance, hash, and license are in [models](models/README.md); startup
requires no network downloads.

For an already running calibrated camera:

```bash
ros2 launch nira_perception_face face_tracking_launch.py
```

The default mode requires matching measured camera intrinsics and supported
plumb-bob/rational distortion. Without them it emits invalid observations.
`desktop_preview:=true` permits an explicitly uncalibrated FOV approximation.
`publish_debug_image:=true` offers `/perception/face_debug` and its `/compressed`
variant on subscription; status is available
on `/perception/face_status`. The image input keeps only the newest frame. Desktop image timestamps represent
host capture-return time; they are not exposure timestamps.
Images older than 300 ms, non-finite data, and out-of-order timestamps are rejected.
No images or identities are stored.

For a laptop camera and offline torso and head visualization:

```bash
ros2 launch nira_viz test_launch.py
```

Tap A / Cross during stand-up or walking to follow the detected face. Hardware
bringup launches the calibrated detector with `enable_camera:=true`. Supply the
head camera calibration using `camera_info_url`. Physical tuning requires a robot.
