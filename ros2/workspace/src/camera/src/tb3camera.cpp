#include "tb3camera.hpp"
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <opencv2/core.hpp>

#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

static sensor_msgs::msg::CameraInfo global_camera_info;
static sensor_msgs::msg::Image global_camera_image;
static sensor_msgs::msg::CompressedImage global_compressed_camera_image;

#define CAMERA_FPS                  50
#define CAMERA_IMPORT_IMAGE_NUM    (10 * CAMERA_FPS) /* 10sec */
typedef struct {
    int camera_info_fd;
    int camera_image_fd;
    FILE *camera_infop;
    char image_name[256];
    int image_index;
    int movie_index;
} CameraSaveInfoType;
static CameraSaveInfoType camera_save_info;
static VideoWriter *global_writer;
static void import_images(void);

void camera_init(void)
{
    camera_save_info.camera_info_fd = open("./camera_info.txt", O_CREAT | O_RDWR, 0644);
    if (camera_save_info.camera_info_fd < 0) {
        printf("ERROR: can not open file: %s errno=%d\n", "./camera_info.txt", errno);
    }
    camera_save_info.camera_infop = fdopen(camera_save_info.camera_info_fd, "rw");
    if (camera_save_info.camera_infop == NULL) {
        printf("ERROR: can not fopen file: %s errno=%d\n", "./camera_info.txt", errno);
    }
}
void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    global_camera_info = *msg;
    return;
}

void cameraImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    global_camera_image = *msg;
    return;
}
static void import_image(int index)
{
    char image_name[256];
    sprintf(image_name, "camera/camera_image_%d.jpeg", index);
    Mat image;
    image = imread(image_name);
	if (image.empty()) {
        printf("ERROR: image is empty\n");
        return;
    }
    global_writer->write(image);
    //printf("IMPORTED: image %s\n", image_name);
    return;
}

static void import_images(void)
{
    //printf("import_images:enter\n");
    char name[256];
    sprintf(name, "./movie/OutVideo%d.wmv", camera_save_info.movie_index);
    int codec = VideoWriter::fourcc('W', 'M', 'V', '1');
    global_writer = new VideoWriter(name, codec, CAMERA_FPS, Size(640, 480), true);
    if (global_writer == NULL) {
        printf("ERROR: can not create VideoWriter: %s errno=%d\n", "./movie/OutVideo.wmv", errno);
        return;
    }
    if (!global_writer->isOpened()) {
        printf("ERROR: writer.isOpened() not opened\n");
        delete global_writer;
        global_writer = NULL;
        return;
    }
    int i;
    for (i = 0; i < CAMERA_IMPORT_IMAGE_NUM; i++) {
        import_image(i);
    }
    global_writer->release();
    delete global_writer;
    global_writer = NULL;
    camera_save_info.movie_index++;
    camera_save_info.image_index = 0;
    return;
}


static void camera_data_save(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    memset(camera_save_info.image_name, 0, sizeof(camera_save_info.image_name));
    sprintf(camera_save_info.image_name, "camera/camera_image_%d.jpeg", camera_save_info.image_index);
    camera_save_info.camera_image_fd = open(camera_save_info.image_name, O_CREAT | O_RDWR, 0644);
    if (camera_save_info.camera_image_fd < 0) {
        printf("ERROR: can not open file: %s errno=%d\n", "./camera_image.jpeg", errno);
        return;
    }
    int size_bytes = (std::end(msg->data) - std::begin(msg->data));
    ftruncate(camera_save_info.camera_image_fd, 0);
    ssize_t size = pwrite(camera_save_info.camera_image_fd, &msg->data[0], size_bytes, 0);
    if (size != size_bytes) {
        printf("ERROR: can not open file: %s errno=%d\n", "./camera_image.jpeg", errno);
        close(camera_save_info.camera_image_fd);
        camera_save_info.camera_image_fd = -1;
        return;
    }
    fsync(camera_save_info.camera_image_fd);
    close(camera_save_info.camera_image_fd);
    camera_save_info.camera_image_fd = -1;
    camera_save_info.image_index++;
    //printf("DONE:image name:%s\n", camera_save_info.image_name);
    return;
}

void cameraCompressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    if (global_camera_info.height > 0) {
        global_compressed_camera_image = *msg;

        //save one image
        camera_data_save(msg);

        //import image to file
        if (camera_save_info.image_index > CAMERA_IMPORT_IMAGE_NUM) {
            import_images();
        }
    }
    return;
}
