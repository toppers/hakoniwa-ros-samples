#!/bin/bash

DOCKER_IMAGE=hakoniwa-ros2-builder

HAKONIWA_TOP_DIR=$(cd ../.. && pwd)
DOCKER_IMAGE=${DOCKER_IMAGE}:v1.0.0

sudo docker ps > /dev/null
if [ $? -ne 0 ]
then
        sudo service docker start
        echo "waiting for docker service activation.. "
        sleep 3
fi

sudo docker run -v ${HAKONIWA_TOP_DIR}:/root/workspace/hakoniwa-ros-sim \
		-it --rm --net host --name hakoniwa-ros-sim ${DOCKER_IMAGE} 
