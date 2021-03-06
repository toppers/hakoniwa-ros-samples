#include <stdio.h>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"

typedef struct {
  double ranges[360];
} ScanDataType;

static ScanDataType scan_data;

static void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
#if 0
	int i;
	for (i = 0; i < 360; i+= 45) {
		printf("msg IN LaserScan: [%d] = %f\n", i, msg->ranges[i]);
	}
#endif

  int i;
  for (i = 0; i < 360; i++) {
    scan_data.ranges[i] = msg->ranges[i];
  }
  return;
}
static void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // TODO
  return;
}

static ros::Publisher* publisher;
static geometry_msgs::Twist cmd_vel;

static float get_foward_distance(void) {
  int i;
  float min = 100.0f;
  for (i = 0; i < 15; i++) {
    if (scan_data.ranges[i] < min) {
      min = scan_data.ranges[i];
    }
  }
  for (i = (360 - 15); i < 360; i++) {
    if (scan_data.ranges[i] < min) {
      min = scan_data.ranges[i];
    }
  }
  // printf("foward: %lf\n", min);
  return min;
}
static float get_right_distance(void) {
  int i;
  float min = 100.0f;
  for (i = (90 - 30); i < (90 + 30); i++) {
    if (scan_data.ranges[i] < min) {
      min = scan_data.ranges[i];
    }
  }
  // printf("right: %lf\n", min);
  return min;
}
static float sarch_all(void) {
  int i;
  float min = 100.0f;
  for (i = 0; i < 360; i++) {
    if (scan_data.ranges[i] < min) {
      min = scan_data.ranges[i];
    }
  }
  return min;
}

static bool do_foward(void) {
  bool is_stop = false;
  cmd_vel.linear.x = 0;
  if (get_foward_distance() < 0.2f) {
    cmd_vel.linear.x = 0;
    is_stop = true;
  } else {
    cmd_vel.linear.x = 0.5f;
  }

  return is_stop;
}

static bool turn_left(void) {
  bool is_stop = false;
  cmd_vel.angular.z = 0;
  if (get_right_distance() < 0.05f) {
    cmd_vel.angular.z = 1.0f;
    is_stop = true;
  } else {
    cmd_vel.angular.z = 0;
  }

  return is_stop;
}
static bool turn_right(void) {
  bool is_stop = false;
  cmd_vel.angular.z = 0;
  if (get_right_distance() < 0.05f) {
    cmd_vel.angular.z = -1.0f;
    is_stop = true;
  } else {
    cmd_vel.angular.z = 0;
  }

  return is_stop;
}

static void do_control(void) {
  (void)do_foward();
  (void)turn_right();

  if (cmd_vel.linear.x == 0 && cmd_vel.angular.z == 0) {
    cmd_vel.angular.z = -1.0f;
  }
  return;
}

typedef enum {
  RoboMode_INIT = 0,
  RoboMode_RUN,
} RoboModeType;
int main(int argc, char** argv) {
  RoboModeType mode = RoboMode_INIT;
  ros::init(argc, argv, "tb3ctrl");
  ros::NodeHandle n;

  ros::Publisher actuator_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber scan_sub = n.subscribe("scan", 1, scanCallback);
  ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback);

  publisher = &actuator_pub;
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    if (mode == RoboMode_INIT) {
      float d = sarch_all();
      if (d > 0.0f && d <= 0.08f) {
        printf("d=%f MOVE\n", d);
        mode = RoboMode_RUN;
      } else {
        printf("WATING d=%f\n", d);
      }
    } else {
      do_control();
      //cmd_vel.angular.z = -0.01; //turnl right
      publisher->publish(cmd_vel);
    }

    loop_rate.sleep();
  }
  return 0;
}
