#!/bin/bash

colcon build --packages-select ev3_msgs
colcon build --packages-select ev3
colcon build --packages-select tb3
colcon build --packages-select ros_tcp_endpoint
