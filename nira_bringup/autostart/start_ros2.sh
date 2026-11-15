#!/usr/bin/env bash
# Run inside the robot user's lingering systemd session.
set -eo pipefail
workspace="$HOME/Workspace/colcon_nira"
source "$workspace/src/ros2_rumblex_nira/scripts/environment.sh"
export SDL_VIDEODRIVER=dummy
export SDL_AUDIODRIVER=pulseaudio
export PYGAME_HIDE_SUPPORT_PROMPT=1
# Use the venv interpreter explicitly even when ros2 itself comes from apt.
exec "$workspace/.venv/bin/python" /opt/ros/lyrical/bin/ros2 launch \
    nira_bringup target_launch.py
