#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>

typedef struct {
	double foward;
	double left;
	double right;
	double back;
} ScanDataType;

static ScanDataType scan_data;

static void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
#if 0
	int i;
	for (i = 0; i < 360; i+= 45) {
		printf("msg IN LaserScan: [%d] = %f\n", i, msg->ranges[i]);
	}
#endif
	scan_data.foward = msg->ranges[0];
	scan_data.left = msg->ranges[90];
	scan_data.back = msg->ranges[180];
	scan_data.right = msg->ranges[270];
	return;
}
static void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   	//TODO 
    return;
}

static ros::Publisher *publisher;
static geometry_msgs::Twist cmd_vel;

static bool do_foward(void)
{
	bool is_stop = false;
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	if (scan_data.foward <= 0.12f) {
		cmd_vel.linear.x = 0;
		is_stop = true;
	}
	else {
		cmd_vel.linear.x = 2;
	}
	
	return is_stop;
}

static bool turn_left(void)
{
	bool is_stop = false;
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	if (scan_data.right <= 0.1f) {
		cmd_vel.angular.z = 0;
		is_stop = true;
	}
	else {
		cmd_vel.angular.z = -3;
	}
	publisher->publish(cmd_vel);
	
	return is_stop;
}

static void do_control(void)
{
	if (do_foward()) {
		turn_left();
	}
	publisher->publish(cmd_vel);
	return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robo");
    ros::NodeHandle n;

    ros::Publisher actuator_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber scan_sub = n.subscribe("scan", 1, scanCallback);
    ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback);

	publisher = &actuator_pub;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
    	
    	do_control(); 
        loop_rate.sleep();
    }
    return 0;
}
