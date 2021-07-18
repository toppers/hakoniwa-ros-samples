#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService, UnityService
from ev3.msg import Ev3PduSensor
from ev3.msg import Ev3PduActuator
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)
    
    tcp_server.start({
        'ev3_sensor': RosPublisher('ev3_sensor', Ev3PduSensor, queue_size=1),
        'ev3_actuator': RosSubscriber('ev3_actuator', Ev3PduActuator, tcp_server),
        'scan': RosPublisher('scan', LaserScan, queue_size=1),
        'imu': RosPublisher('imu', Imu, queue_size=1),
        'cmd_vel': RosSubscriber('cmd_vel', Twist, tcp_server),
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()