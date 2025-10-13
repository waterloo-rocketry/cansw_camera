#ifndef VIDEO_H
#define VIDEO_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    VIDEO_OFF,
    VIDEO_ON,
    VIDEO_ERR_SD,
    VIDEO_ERR_CAM
} video_state_t;

#define VIDEO_CAPTURE_DMA_WORDS 0x7ff0U
#define VIDEO_FRAME_MAX_BYTES (((VIDEO_CAPTURE_DMA_WORDS + 0x0fU) * 4U))

void video_start();
void video_stop();
video_state_t video_get_state();

bool video_capture_frame();
void video_f_sync();
bool video_get_last_frame(const uint8_t **data, size_t *length);

#endif
