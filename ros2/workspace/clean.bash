#!/bin/bash

rm -rf src/build
rm -rf src/install
rm -rf src/log

DIR_PATH=$(cd ../../third-party/ros2/ros_tcp_endpoint && pwd)
if [ -d src/ros_tcp_endpoint ]
then
	rm -rf src/ros_tcp_endpoint
else
	cp -rp ${DIR_PATH} src/ros_tcp_endpoint
fi
