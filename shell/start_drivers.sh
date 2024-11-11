#!/bin/bash

# START


screen -ls | grep driver
screen -ls | grep lane

echo "Strart driver"
screen -m -d -S driver bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py'