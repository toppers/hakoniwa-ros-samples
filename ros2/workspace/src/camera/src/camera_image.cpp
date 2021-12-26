#include "camera_image.hpp"
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <opencv2/core.hpp>
#include "camera_movie.hpp"

typedef struct {
    char image_name[4096];
    const char *base_dirname;
    int dir_index;
    int image_index;
} CameraSaveInfoType;
static CameraSaveInfoType camera_save_info;

const char* camera_get_basename(void)
{
    return camera_save_info.base_dirname;
}

static void update_index(void)
{
    camera_save_info.image_index++;
    if (camera_save_info.image_index >= CAMERA_IMAGE_SAVE_NUM) {
        camera_save_info.image_index = 0;
        camera_movie_request(camera_save_info.dir_index);
        camera_save_info.dir_index++;
        if (camera_save_info.dir_index >= CAMERA_IMAGE_DIR_NUM) {
            camera_save_info.dir_index = 0;
        }
    }
    return;
}

void camera_init(const char* base_dirname)
{
    int i;
    camera_save_info.base_dirname = base_dirname;
    char path[4096];
    sprintf(path, "camera/%s", base_dirname);
    (void)mkdir(path, 0644);

    for (i = 0; i < CAMERA_IMAGE_DIR_NUM; i++) {
        sprintf(path, "camera/%s/camera_%d", base_dirname, i);
        (void)mkdir(path, 0644);
    }
    camera_movie_init();
    return;
}

static void camera_data_save(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    int size_bytes = (std::end(msg->data) - std::begin(msg->data));
    if (size_bytes < 1024) {
        return;
    }

    if (camera_save_info.image_index == 0) {
        camera_movie_cancel(camera_save_info.dir_index);
    }

    memset(camera_save_info.image_name, 0, sizeof(camera_save_info.image_name));
    sprintf(camera_save_info.image_name, "camera/%s/camera_%d/camera_image_%d.jpeg", 
        camera_save_info.base_dirname,
        camera_save_info.dir_index, 
        camera_save_info.image_index);
    int fd = open(camera_save_info.image_name, O_CREAT | O_RDWR, 0644);
    if (fd < 0) {
        printf("ERROR: can not open file: %s errno=%d\n", "./camera_image.jpeg", errno);
        return;
    }
    ftruncate(fd, 0);
    ssize_t size = pwrite(fd, &msg->data[0], size_bytes, 0);
    if (size != size_bytes) {
        printf("ERROR: can not open file: %s errno=%d\n", "./camera_image.jpeg", errno);
        close(fd);
        return;
    }
    fsync(fd);
    close(fd);
    //printf("DONE:image name:%s\n", camera_save_info.image_name);
    update_index();
    return;
}

void cameraCompressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    camera_data_save(msg);
    return;
}
