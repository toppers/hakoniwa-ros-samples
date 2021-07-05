#!/bin/bash

rm -rf build
rm -rf devel

DIR_PATH=$(cd ../../third-party/ROS-TCP-Endpoint && pwd)

if [ -f src/ROS-TCP-Endpoint ]
then
	:
else
	ln -s ${DIR_PATH} src/ROS-TCP-Endpoint
fi

catkin_make
