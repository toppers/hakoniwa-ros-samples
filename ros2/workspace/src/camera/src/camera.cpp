#include <stdio.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rule_msgs/msg/hako_env.hpp"
#include "camera_image.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  printf("CAMERA START\n");
  rclcpp::init(argc, argv);

  camera_init("tb3_camera");
  auto node = rclcpp::Node::make_shared("camera_node");
  printf("camera_subscribe start\n");
  auto subscriber_compressed_camera_image = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "HakoEnvCamera", 1, cameraCompressedImageCallback);
  auto subscriber_compressed_camera_image1 = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "image/compressed", 1, cameraCompressedImageCallback1);
  auto subscriber_hakoenv_hakoenv = node->create_subscription<rule_msgs::msg::HakoEnv>(
      "hako_env", 1, hakoEnvCallback);

  rclcpp::WallRate rate(10ms);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  return 0;
}
