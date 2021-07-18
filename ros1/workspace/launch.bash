source devel/setup.bash

CFG_FILE=src/ros_tcp_endpoint/config/params.yaml
rm -f ${CFG_FILE}

echo "ROS_IP: 127.0.0.1" >> ${CFG_FILE}
echo "ROS_TCP_PORT: 10000" >> ${CFG_FILE}
roslaunch ros_tcp_endpoint endpoint.launch
