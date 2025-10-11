#include <stdio.h>

#include "lfs.h"
#include "main.h"
#include "ov5640.h"
#include "video.h"

const uint32_t MAX_FILE_SIZE = 1 << 31;

// variables used by the filesystem
lfs_t lfs;
lfs_file_t video_file;

uint32_t root_dir_files;

video_state_t state;

int sd_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer,
			lfs_size_t size) {
	uint32_t timeout_ms = 2000U;
	uint32_t block_addr = block;
	uint32_t num_blocks = (size + c->block_size - 1) / c->block_size;
	HAL_StatusTypeDef hal =
		HAL_SD_ReadBlocks(&hsd2, (uint8_t *)buffer, block_addr, num_blocks, timeout_ms);
	if (hal != HAL_OK) {
		return -1; // LFS_ERR_IO
	}

	// Wait for card to be ready (polling)
	uint32_t start = HAL_GetTick();
	while (HAL_SD_GetCardState(&hsd2) != HAL_SD_CARD_TRANSFER) {
		if ((HAL_GetTick() - start) > timeout_ms) {
			return -1; // timeout -> LFS_ERR_IO
		}
	}

	return 0; // success
}

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
	// block device operations
	.read = sd_read,
	.prog = user_provided_block_device_prog,
	.erase = user_provided_block_device_erase,
	.sync = user_provided_block_device_sync,

	// block device configuration
	.read_size = 16,
	.prog_size = 16,
	.block_size = 4096,
	.block_count = 128,
	.cache_size = 16,
	.lookahead_size = 16,
	.block_cycles = 500,
};

void video_start() {
	FRESULT r = lfs_mount(&lfs, "0:", 0);
	if (r != FR_OK) {
		state = VIDEO_ERR_SD;
		return;
	}

	// count the number of flies in the root directory of the SD card
	root_dir_files = 0;
	DIR dir;
	if (lfs_dir_open(&dir, "/") != FR_OK) {
		state = VIDEO_ERR_SD;
		return;
	}

	FILINFO finfo;
	while (lfs_dir_read(&dir, &finfo) == FR_OK && finfo.fname[0] != '\0') {
		root_dir_files++;
	}
	lfs_dir_close(&dir);

	char path[20];
	sprintf(path, "/mov%04u.mjpg", root_dir_files);
	r = lfs_file_open(&video_file, path, FA_WRITE | FA_CREATE_ALWAYS);
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

	state = VIDEO_ON;
}

void video_stop() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // Set CAM_EN low
	lfs_file_close(&video_file);
	state = VIDEO_OFF;
}

video_state_t video_get_state() {
	return state;
}

#define BUF_SIZE 0x7ff0
#define NUM_BUF 2

bool video_capture_frame() {
	static uint8_t fb[NUM_BUF][(BUF_SIZE + 0xf) * 4];

	static size_t length = 0;
	static uint8_t write_buf = 0;

	if (state != VIDEO_ON) {
		return false;
	}

	// Double buffered capture. On each call to capture_frame, capture into capture_buf
	// while writing out write_buf, then swap.
	uint8_t capture_buf = write_buf + 1;
	if (capture_buf >= NUM_BUF) {
		capture_buf = 0;
	}

	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)fb[capture_buf], BUF_SIZE);
	// Write out write_buf while DMA is happening in the background
	UINT retval;
	FRESULT r = lfs_file_write(&video_file, fb[write_buf], length, &retval);
	if (r != FR_OK) {
		state = VIDEO_ERR_SD;
	}
	// Then wait for DMA to finish
	while ((DCMI->CR & DCMI_CR_CAPTURE) != 0)
		;
	// We have to manually abort the DMA and calculate the length when the camera is done,
	// since it doesn't stop automatically
	HAL_DMA_Abort(hdcmi.DMA_Handle);
	length = (BUF_SIZE - ((DMA_Stream_TypeDef *)hdcmi.DMA_Handle->Instance)->NDTR) * 4;
	write_buf = capture_buf;

	// Do a quick integrity check on the captured frame
	if (fb[capture_buf][6] != 'J' || fb[capture_buf][7] != 'F' || fb[capture_buf][8] != 'I' ||
		fb[capture_buf][9] != 'F') {
		return false;
	}
	return true;
}

void video_file_sync() {
	// Write file metadata periodically to prevent loss of data on poweroff
	FRESULT r = lfs_file_sync(&video_file);
	if (r != FR_OK) {
		state = VIDEO_ERR_SD;
		return;
	}

	// And switch to a new file to avoid FATFS 4Gb size limit
	if (lfs_file_size(&video_file) >= MAX_FILE_SIZE) {
		lfs_file_close(&video_file);

		char path[20];
		sprintf(path, "/mov%04u.mjpg", root_dir_files);
		r = lfs_file_open(&video_file, path, FA_WRITE | FA_CREATE_ALWAYS);
		if (r != FR_OK) {
			state = VIDEO_ERR_SD;
			return;
		}
		root_dir_files++;
	}
}
