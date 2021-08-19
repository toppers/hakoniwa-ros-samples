#!/bin/bash

catkin_make --pkg ev3_msgs
catkin_make --pkg ev3
catkin_make --pkg tb3
catkin_make --pkg ros_tcp_endpoint
catkin_make --pkg tb3_rviz
