#!/bin/bash

IP_ADDR=`ifconfig eth0 | grep netmask | awk '{print $2}'`
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=${IP_ADDR}
