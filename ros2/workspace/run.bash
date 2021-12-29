#!/bin/bash

source install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
if [ $# -ne 1 ]
then
	echo "Usage: $0 <pkgname>"
	exit 1
fi
ros2 run ${1} ${1}_node
