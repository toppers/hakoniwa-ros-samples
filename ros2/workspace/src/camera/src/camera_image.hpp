#ifndef _CAMERA_IMAGE_H_
#define _CAMERA_IMAGE_H_

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "rule_msgs/msg/hako_env.hpp"

#define CAMERA_FPS               50
#define CAMERA_IMAGE_SAVE_NUM    (10 * CAMERA_FPS) /* 10sec */
#define CAMERA_IMAGE_DIR_NUM     10

extern void camera_init(const char* base_name);
extern void cameraCompressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
extern void cameraCompressedImageCallback1(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
extern void hakoEnvCallback(const rule_msgs::msg::HakoEnv::SharedPtr msg);
extern const char* camera_get_basename(void);

#endif /* _CAMERA_IMAGE_H_ */