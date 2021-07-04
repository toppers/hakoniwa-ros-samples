#!/bin/bash

rm -rf build
rm -rf devel

DIR_PATH=$(cd ../../third-party/ROS-TCP-Endpoint && pwd)
ln -s ${DIR_PATH} src/ROS-TCP-Endpoint

catkin_make
