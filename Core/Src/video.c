#include <stdio.h>

#include "fatfs.h"
#include "ov5640.h"
#include "video.h"

const uint32_t MAX_FILE_SIZE = 1 << 31;

FATFS fatfs;
FIL video_file;
uint32_t root_dir_files;

video_state_t state;

#define NUM_BUF 2U
static uint8_t fb[NUM_BUF][VIDEO_FRAME_MAX_BYTES];
static size_t current_frame_length = 0;
static uint8_t current_write_buf = 0;

void video_start() {
    FRESULT r = f_mount(&fatfs, "0:", 0);
    if (r != FR_OK) {
        state = VIDEO_ERR_SD;
        return;
    }

    // count the number of flies in the root directory of the SD card
    root_dir_files = 0;
    DIR dir;
    if (f_opendir(&dir, "/") != FR_OK) {
        state = VIDEO_ERR_SD;
        return;
    }

    FILINFO finfo;
    while (f_readdir(&dir, &finfo) == FR_OK && finfo.fname[0] != '\0') {
        root_dir_files++;
    }
    f_closedir(&dir);

    char path[20];
    sprintf(path, "/mov%04u.mjpg", root_dir_files);
    r = f_open(&video_file, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (r != FR_OK) {
        state = VIDEO_ERR_SD;
        return;
    }
    root_dir_files++;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // Set CAM_RESET low (active)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // Set CAM_EN high
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // Set CAM_RESET high
    HAL_Delay(100);

    if (ov5640_init() != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // Set CAM_EN low
        state = VIDEO_ERR_CAM;
        return;
    }

    current_frame_length = 0;
    current_write_buf = 0;
    state = VIDEO_ON;
}

void video_stop() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // Set CAM_EN low
    f_close(&video_file);
    state = VIDEO_OFF;
}

video_state_t video_get_state() {
    return state;
}

bool video_capture_frame() {
    if (state != VIDEO_ON) {
        return false;
    }

    // Double buffered capture. On each call to capture_frame, capture into capture_buf
    // while writing out write_buf, then swap.
    uint8_t prev_buf = current_write_buf;
    uint8_t capture_buf = prev_buf + 1;
    if (capture_buf >= NUM_BUF) {
        capture_buf = 0;
    }

    HAL_DCMI_Start_DMA(
        &hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)fb[capture_buf], VIDEO_CAPTURE_DMA_WORDS
    );
    // Write out write_buf while DMA is happening in the background
    UINT retval;
    size_t prev_length = current_frame_length;
    FRESULT r = f_write(&video_file, fb[prev_buf], prev_length, &retval);
    if (r != FR_OK || retval != prev_length) {
        state = VIDEO_ERR_SD;
    }
    // Then wait for DMA to finish
    while ((DCMI->CR & DCMI_CR_CAPTURE) != 0)
        ;
    // We have to manually abort the DMA and calculate the length when the camera is done,
    // since it doesn't stop automatically
    HAL_DMA_Abort(hdcmi.DMA_Handle);
    DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance;
    size_t captured_length = (VIDEO_CAPTURE_DMA_WORDS - dma_stream->NDTR) * 4U;
    current_write_buf = capture_buf;
    current_frame_length = captured_length;

    // Do a quick integrity check on the captured frame
    if (captured_length == 0 || fb[capture_buf][6] != 'J' || fb[capture_buf][7] != 'F' ||
        fb[capture_buf][8] != 'I' || fb[capture_buf][9] != 'F') {
        return false;
    }
    return true;
}

void video_f_sync() {
    // Write file metadata periodically to prevent loss of data on poweroff
    FRESULT r = f_sync(&video_file);
    if (r != FR_OK) {
        state = VIDEO_ERR_SD;
        return;
    }

    // And switch to a new file to avoid FATFS 4Gb size limit
    if (f_size(&video_file) >= MAX_FILE_SIZE) {
        f_close(&video_file);

        char path[20];
        sprintf(path, "/mov%04u.mjpg", root_dir_files);
        r = f_open(&video_file, path, FA_WRITE | FA_CREATE_ALWAYS);
        if (r != FR_OK) {
            state = VIDEO_ERR_SD;
            return;
        }
        root_dir_files++;
    }
}

bool video_get_last_frame(const uint8_t **data, size_t *length) {
    if (!data || !length) {
        return false;
    }

    if (current_frame_length == 0) {
        return false;
    }

    *data = fb[current_write_buf];
    *length = current_frame_length;
    return true;
}
