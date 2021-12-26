#include "camera_movie.hpp"
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <opencv2/core.hpp>
#include <thread>
#include <chrono>
using std::this_thread::sleep_for;

typedef struct {
    char movie_name[4096];
    const char *base_dirname;
    int movie_index;
    bool has_request[CAMERA_MOVIE_SAVE_NUM];
} MovieSaveInfoType;

static MovieSaveInfoType movie_save_info;
constexpr int TIME_TO_SLEEP = 100;

static void camera_movie_run(void);

void camera_movie_init(void)
{
    char path[4096];
    sprintf(path, "movie/%s", camera_get_basename());
    (void)mkdir(path, 0644);

   	std::thread thr(camera_movie_run);
	thr.detach();

    return;
}
void camera_movie_request(int dir_index)
{
    if (dir_index >= CAMERA_MOVIE_SAVE_NUM) {
        return;
    }
    movie_save_info.has_request[dir_index] = true;
    return;
}
void camera_movie_cancel(int dir_index)
{
    if (dir_index >= CAMERA_MOVIE_SAVE_NUM) {
        return;
    }
    movie_save_info.has_request[dir_index] = false;
    return;
}

static void import_images(int dir_index);

static void camera_movie_run(void)
{
    while (true) {
        int i;
        for (i = 0; i < CAMERA_IMAGE_DIR_NUM; i++) {
            if (movie_save_info.has_request[i] == false) {
                continue;
            }
            import_images(i);
            movie_save_info.has_request[i] = false;
        }
        sleep_for(std::chrono::milliseconds(TIME_TO_SLEEP));
    }
	return;
}


static void import_image(cv::VideoWriter *writer, int dir_index, int index)
{
    char image_name[4096];
    sprintf(image_name, "camera/%s/camera_%d/camera_image_%d.jpeg", camera_get_basename(), dir_index, index);
    cv::Mat image;
    image = cv::imread(image_name);
	if (image.empty()) {
        printf("ERROR: image is empty\n");
        return;
    }
    writer->write(image);
    //printf("IMPORTED: image %s\n", image_name);
    return;
}

static void import_images(int dir_index)
{
    //printf("import_images:enter\n");
    char name[4096];
    sprintf(name, "./movie/%s/movie%d.wmv", camera_get_basename(), dir_index);
    int codec = cv::VideoWriter::fourcc('W', 'M', 'V', '1');
    cv::VideoWriter writer = cv::VideoWriter(name, codec, CAMERA_FPS, cv::Size(640, 480), true);
    if (!writer.isOpened()) {
        printf("ERROR: writer.isOpened() not opened\n");
        return;
    }
    int i;
    for (i = 0; i < CAMERA_IMAGE_SAVE_NUM; i++) {
        import_image(&writer, dir_index, i);
    }
    writer.release();
    return;
}