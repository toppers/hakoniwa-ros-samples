#include <stdio.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string.h>

using namespace std::chrono_literals;
static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_servo_base;

static void clear_msg(geometry_msgs::msg::Twist *cmd)
{
  cmd->linear.x = 0;
  cmd->linear.y = 0;
  cmd->linear.z = 0;
  cmd->angular.x = 0;
  cmd->angular.y = 0;
  cmd->angular.z = 0;
}
static bool current_servo_move_left;
static bool current_servo_move_right;
static void servoBaseMoveLeft(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    current_servo_move_left = true;
    printf("move left enter\n");
  }
  else {
    current_servo_move_left = false;
  }
}
static void servoBaseMoveRight(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    current_servo_move_right = true;
    printf("move right enter\n");
  }
  else {
    current_servo_move_right = false;
  }
}
static bool current_rotate_left;
static bool current_rotate_right;
static void servoRotateLeft(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    current_rotate_left = true;
    printf("rotate left enter\n");
  }
  else {
    current_rotate_left = false;
  }
}
static void servoRotateRight(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    current_rotate_right = true;
    printf("rotate right enter\n");
  }
  else {
    current_rotate_right = false;
  }
}

static bool current_pinch_close;
static void pinchOpen(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    current_pinch_close = false;
    printf("pinch open enter\n");
  }
}
static void pinchClose(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    current_pinch_close = true;
    printf("pinch close enter\n");
  }
}

int main(int argc, char **argv) {
  char buffer[20][4096];

  if (argc > 1) {
    sprintf(buffer[0], "%s_robo_node", argv[1]);
    sprintf(buffer[1], "%s_cmd_vel", argv[1]);
    sprintf(buffer[2], "%s_servo_base_angle", argv[1]);
    sprintf(buffer[3], "%s_servo_angle", argv[1]);
    sprintf(buffer[4], "%s_pincher_cmd", argv[1]);
    sprintf(buffer[5], "%s_servo_base_move_left", argv[1]);
    sprintf(buffer[6], "%s_servo_base_move_right", argv[1]);
    sprintf(buffer[7], "%s_pinch_open", argv[1]);
    sprintf(buffer[8], "%s_pinch_close", argv[1]);
    sprintf(buffer[9], "%s_rotate_right", argv[1]);
    sprintf(buffer[10], "%s_rotate_left", argv[1]);
  }
  else {
    sprintf(buffer[0], "robo_node");
    sprintf(buffer[1], "cmd_vel");
    sprintf(buffer[2], "servo_base_angle");
    sprintf(buffer[3], "servo_angle");
    sprintf(buffer[4], "pincher_cmd");
    sprintf(buffer[5], "servo_base_move_left");
    sprintf(buffer[6], "servo_base_move_right");
    sprintf(buffer[7], "pinch_open");
    sprintf(buffer[8], "pinch_close");
    sprintf(buffer[9], "rotate_right");
    sprintf(buffer[10], "rotate_left");
  }
	printf("START\n");
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(buffer[0]);
  auto publisher_motor =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[1], 1);
  publisher_servo_base =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[2], 1);
  auto publisher_servo =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[3], 1);
  auto publisher_pincher =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[4], 1);

  auto subscriber_sbl = node->create_subscription<std_msgs::msg::Bool>(
      buffer[5], 1, servoBaseMoveLeft);
  auto subscriber_sbr = node->create_subscription<std_msgs::msg::Bool>(
      buffer[6], 1, servoBaseMoveRight);

  auto subscriber_po = node->create_subscription<std_msgs::msg::Bool>(
      buffer[7], 1, pinchOpen);
  auto subscriber_pc = node->create_subscription<std_msgs::msg::Bool>(
      buffer[8], 1, pinchClose);

  auto subscriber_rl = node->create_subscription<std_msgs::msg::Bool>(
      buffer[9], 1, servoRotateLeft);
  auto subscriber_rr = node->create_subscription<std_msgs::msg::Bool>(
      buffer[10], 1, servoRotateRight);

  rclcpp::WallRate rate(10ms);

  geometry_msgs::msg::Twist cmd_vel_motor;
  geometry_msgs::msg::Twist cmd_vel_servo_base;
  geometry_msgs::msg::Twist cmd_vel_servo;
  geometry_msgs::msg::Twist cmd_vel_pinch;
  clear_msg(&cmd_vel_motor);
  clear_msg(&cmd_vel_servo_base);
  clear_msg(&cmd_vel_servo);
  clear_msg(&cmd_vel_pinch);

#if 1
  while (rclcpp::ok()) {
    clear_msg(&cmd_vel_pinch);
    clear_msg(&cmd_vel_servo_base);
    clear_msg(&cmd_vel_servo);
    if (current_servo_move_right) {
        cmd_vel_servo_base.angular.y = 1.0f;
    }
    else if (current_servo_move_left) {
        cmd_vel_servo_base.angular.y = -1.0f;
    }
    if (current_rotate_right) {
        cmd_vel_servo.angular.y = -1.0f;
    }
    else if (current_rotate_left) {
        cmd_vel_servo.angular.y = 1.0f;
    }
    if (current_pinch_close) {
        cmd_vel_pinch.linear.x = 1.0f;
    }
    else  {
        cmd_vel_pinch.linear.x = 0.0f;
    }
    publisher_servo_base->publish(cmd_vel_servo_base);
    publisher_pincher->publish(cmd_vel_pinch);
    publisher_servo->publish(cmd_vel_servo);
    rclcpp::spin_some(node);
    rate.sleep();
  }
#else
  while (rclcpp::ok()) {
    printf("ServoUp   : u, ServeDown  : d\n");
    printf("ServoLeft : l, ServeRight : r\n");
    printf("MoveFoward: i, MoveBack   : n\n");
    printf("MoveLeft  : j, RotateRight: k\n");
    printf("PinchOpen : o, PinchClose : c\n");
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
      case 'o':
        cmd_vel_pinch.linear.x = 0.0f;
        break;
      case 'c':
        cmd_vel_pinch.linear.x = 1.0f;
        break;
      case ' ':
        clear_msg(&cmd_vel_motor);
        clear_msg(&cmd_vel_servo_base);
        clear_msg(&cmd_vel_servo);
        clear_msg(&cmd_vel_pinch);
        break;
      default:
        break;
    }

    publisher_motor->publish(cmd_vel_motor);
    publisher_servo_base->publish(cmd_vel_servo_base);
    publisher_servo->publish(cmd_vel_servo);
    publisher_pincher->publish(cmd_vel_pinch);
    rclcpp::spin_some(node);
    rate.sleep();
  }
#endif
  return 0;
}
