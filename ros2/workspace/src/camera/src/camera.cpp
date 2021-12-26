#include <stdio.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tb3camera.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  printf("CAMERA START\n");
  rclcpp::init(argc, argv);

  camera_init();

  auto node = rclcpp::Node::make_shared("camera_node");
  auto subscriber_camera_info = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", 1, cameraInfoCallback);
  auto subscriber_camera_image = node->create_subscription<sensor_msgs::msg::Image>(
      "image", 1, cameraImageCallback);
  auto subscriber_compressed_camera_image = node->create_subscription<sensor_msgs::msg::CompressedImage>(
      "image/compressed", 1, cameraCompressedImageCallback);

  rclcpp::WallRate rate(10ms);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  return 0;
}
