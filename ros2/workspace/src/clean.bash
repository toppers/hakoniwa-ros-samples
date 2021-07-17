#!/bin/bash

rm -rf build
rm -rf install

DIR_PATH=$(cd ../../../third-party/ros2/ros_tcp_endpoint && pwd)
if [ -d ros_tcp_end_point ]
then
	rm -rf ros_tcp_endpoint
else
	cp -rp ${DIR_PATH} ros_tcp_endpoint
fi
