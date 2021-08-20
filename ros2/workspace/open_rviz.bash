#!/bin/bash

source install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 launch tb3_rviz display.launch.py