#include <stdio.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <string.h>

using namespace std::chrono_literals;

static void clear_msg(geometry_msgs::msg::Twist *cmd)
{
  cmd->linear.x = 0;
  cmd->linear.y = 0;
  cmd->linear.z = 0;
  cmd->angular.x = 0;
  cmd->angular.y = 0;
  cmd->angular.z = 0;
}

int main(int argc, char **argv) {
  char buffer[4][4096];

  if (argc > 1) {
    sprintf(buffer[0], "%s_robo_node", argv[1]);
    sprintf(buffer[1], "%s_cmd_vel", argv[1]);
    sprintf(buffer[2], "%s_servo_base_angle", argv[1]);
    sprintf(buffer[3], "%s_servo_angle", argv[1]);
  }
  else {
    sprintf(buffer[0], "robo_node");
    sprintf(buffer[1], "cmd_vel");
    sprintf(buffer[2], "servo_base_angle");
    sprintf(buffer[3], "servo_angle");
  }
	printf("START\n");
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(buffer[0]);
  auto publisher_motor =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[1], 1);
  auto publisher_servo_base =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[2], 1);
  auto publisher_servo =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[3], 1);

  rclcpp::WallRate rate(10ms);

  geometry_msgs::msg::Twist cmd_vel_motor;
  geometry_msgs::msg::Twist cmd_vel_servo_base;
  geometry_msgs::msg::Twist cmd_vel_servo;
  clear_msg(&cmd_vel_motor);
  clear_msg(&cmd_vel_servo_base);
  clear_msg(&cmd_vel_servo);

  while (rclcpp::ok()) {
    int key = getchar();
    printf("key=%c\n", key);
    switch (key) {
      case 'l':
        cmd_vel_servo_base.angular.y = -1.0f;
        break;
      case 'r':
        cmd_vel_servo_base.angular.y = 1.0f;
        break;
      case 'u':
        cmd_vel_servo.angular.y = -1.0f;
        break;
      case 'd':
        cmd_vel_servo.angular.y = 1.0f;
        break;
      case 'i':
        cmd_vel_motor.linear.x = 0.5f;
        break;
      case 'n':
        cmd_vel_motor.linear.x = -0.5f;
        break;
      case 'j':
        cmd_vel_motor.angular.z = -0.2f;
        break;
      case 'k':
        cmd_vel_motor.angular.z = 0.2f;
        break;
      case 's':
        clear_msg(&cmd_vel_motor);
        break;
      case ' ':
        clear_msg(&cmd_vel_motor);
        clear_msg(&cmd_vel_servo_base);
        clear_msg(&cmd_vel_servo);
        break;
      default:
        break;
    }

    publisher_motor->publish(cmd_vel_motor);
    publisher_servo_base->publish(cmd_vel_servo_base);
    publisher_servo->publish(cmd_vel_servo);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  return 0;
}
