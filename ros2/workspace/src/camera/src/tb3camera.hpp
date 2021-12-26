#ifndef _TB3CAMERA_H_
#define _TB3CAMERA_H_

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

extern void camera_init(void);
extern void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
extern void cameraImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
extern void cameraCompressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

#endif /* _TB3CAMERA_H_ */