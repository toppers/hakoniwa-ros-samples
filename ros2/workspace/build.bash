#!/bin/bash

colcon build --packages-select ev3_msgs
colcon build --packages-select ev3
colcon build --packages-select tb3
colcon build --packages-select ros_tcp_endpoint
colcon build --packages-select hakoniwa_turtlebot3 hakoniwa_turtlebot3_rviz hakoniwa_turtlebot3_description