#ifndef _CAMERA_MOVIE_H_
#define _CAMERA_MOVIE_H_

#include "camera_image.hpp"

#define CAMERA_MOVIE_SAVE_NUM    CAMERA_IMAGE_DIR_NUM

extern void camera_movie_init(void);
extern void camera_movie_request(int dir_index);
extern void camera_movie_cancel(int dir_index);
#endif /* _CAMERA_MOVIE_H_ */