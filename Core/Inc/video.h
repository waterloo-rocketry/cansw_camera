#ifndef INC_VIDEO_H_
#define INC_VIDEO_H_

#include <stdbool.h>

typedef enum {
    VIDEO_OFF,
	VIDEO_ON,
	VIDEO_ERR_SD,
	VIDEO_ERR_CAM
} video_state_t;

void video_start();
void video_stop();
video_state_t video_get_state();

bool video_capture_frame();
void video_f_sync();

#endif /* INC_VIDEO_H_ */
