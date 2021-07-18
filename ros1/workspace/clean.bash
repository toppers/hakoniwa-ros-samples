#!/bin/bash

rm -rf build
rm -rf devel

DIR_PATH=$(cd ../../third-party/ros1/ros_tcp_endpoint && pwd)
if [ -d src/ros_tcp_endpoint ]
then
	rm -rf src/ros_tcp_endpoint
fi
cp -rp ${DIR_PATH} src/ros_tcp_endpoint
