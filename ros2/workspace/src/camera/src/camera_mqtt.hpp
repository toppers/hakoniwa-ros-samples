#ifndef _CAMERA_MQTT_H_
#define _CAMERA_MQTT_H_

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

extern void camera_mqtt_init(void);
extern void camera_mqtt_publish(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

#endif /* _CAMERA_MQTT_H_ */