#ifndef _CAMERA_MQTT_H_
#define _CAMERA_MQTT_H_

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "rule_msgs/msg/hako_env.hpp"

extern void camera_mqtt_init(void);
extern void camera_mqtt_publish(const char* camera_mqtt_topic_name, const sensor_msgs::msg::CompressedImage::SharedPtr msg);
extern void hakoenv_mqtt_publish(const char* camera_mqtt_topic_name, const rule_msgs::msg::HakoEnv::SharedPtr msg);

#endif /* _CAMERA_MQTT_H_ */