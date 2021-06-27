#!/bin/bash

source devel/setup.bash

rosparam load src/ROS-TCP-Endpoint/config/params.yaml
rosrun robotics_demo server_endpoint.py

