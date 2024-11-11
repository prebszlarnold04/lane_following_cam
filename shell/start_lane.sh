#!/bin/bash

# START


screen -ls | grep driver
screen -ls | grep lane

echo "Strart lane"
screen -m -d -S lane bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch lane_following_cam robot_compressed1.launch.py'