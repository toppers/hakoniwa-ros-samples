#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>

static void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   	//TODO 
    return;
}
static void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   	//TODO 
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robo");
    ros::NodeHandle n;

    ros::Publisher actuator_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber scan_sub = n.subscribe("scan", 1, scanCallback);
    ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
    	
    	//TODO 
        loop_rate.sleep();
    }
    return 0;
}
