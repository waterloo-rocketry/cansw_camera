#include "ov5640.h"
#include "main.h"
#include <stdbool.h>

HAL_StatusTypeDef ov5640_read_reg(uint16_t addr, uint8_t *val) {
	return HAL_I2C_Mem_Read(&hi2c1, OV5640_I2C_ADDR, addr, 2, val, 1, 100);
}

HAL_StatusTypeDef ov5640_write_reg(uint16_t addr, uint8_t val) {
	return HAL_I2C_Mem_Write(&hi2c1, OV5640_I2C_ADDR, addr, 2, &val, 1, 100);
}

HAL_StatusTypeDef ov5640_write_reg16(const uint16_t reg, uint16_t value)
{
    return ov5640_write_reg(reg, value >> 8) | ov5640_write_reg(reg + 1, value);
}

HAL_StatusTypeDef ov5640_write_addr_reg(const uint16_t reg, uint16_t x_value, uint16_t y_value)
{
    return ov5640_write_reg16(reg, x_value) | ov5640_write_reg16(reg + 2, y_value);
}


static int set_pll(bool bypass, uint8_t multiplier, uint8_t sys_div, uint8_t pre_div, bool root_2x, uint8_t pclk_root_div, bool pclk_manual, uint8_t pclk_div){
    int ret = 0;
    if(multiplier > 252 || multiplier < 4 || sys_div > 15 || pre_div > 8 || pclk_div > 31 || pclk_root_div > 3){
        //ESP_LOGE(TAG, "Invalid arguments");
        return -1;
    }
    if(multiplier > 127){
        multiplier &= 0xFE;//only even integers above 127
    }
    //ESP_LOGI(TAG, "Set PLL: bypass: %u, multiplier: %u, sys_div: %u, pre_div: %u, root_2x: %u, pclk_root_div: %u, pclk_manual: %u, pclk_div: %u", bypass, multiplier, sys_div, pre_div, root_2x, pclk_root_div, pclk_manual, pclk_div);

    //calc_sysclk(sensor->xclk_freq_hz, bypass, multiplier, sys_div, pre_div, root_2x, pclk_root_div, pclk_manual, pclk_div);

    ret = ov5640_write_reg(0x3039, bypass?0x80:0x00);
    if (ret == 0) {
        ret = ov5640_write_reg(0x3034, 0x1A);//10bit mode
    }
    if (ret == 0) {
        ret = ov5640_write_reg(0x3035, 0x01 | ((sys_div & 0x0f) << 4));
    }
    if (ret == 0) {
        ret = ov5640_write_reg(0x3036, multiplier & 0xff);
    }
    if (ret == 0) {
        ret = ov5640_write_reg(0x3037, (pre_div & 0xf) | (root_2x?0x10:0x00));
    }
    if (ret == 0) {
        ret = ov5640_write_reg(0x3108, (pclk_root_div & 0x3) << 4 | 0x06);
    }
    if (ret == 0) {
        ret = ov5640_write_reg(0x3824, pclk_div & 0x1f);
    }
    if (ret == 0) {
        ret = ov5640_write_reg(0x460C, pclk_manual?0x22:0x20);
    }
    if (ret == 0) {
        ret = ov5640_write_reg(0x3103, 0x13);// system clock from pll, bit[1]
    }
    if(ret){
        //ESP_LOGE(TAG, "set_sensor_pll FAILED!");
    }
    return ret;
}

HAL_StatusTypeDef ov5640_init() {
	static const uint16_t OV5640_Common[][2] = {
		    {OV5640_SYSTEM_CTROL0, 0x82},  // software reset
		    {OV5640_REG_DLY, 10}, // delay 10ms
		    {OV5640_SYSTEM_CTROL0, 0x42},  // power down

		    //enable pll
		    {0x3103, 0x13},

		    //io direction
		    {0x3017, 0xff},
		    {0x3018, 0xff},

		    {OV5640_DRIVE_CAPABILITY, 0xc3},
		    {OV5640_POLARITY_CTRL, 0x23},

		    {0x4713, 0x02},//jpg mode select

		    {OV5640_ISP_CONTROL01, 0x83}, // turn color matrix, awb and SDE

		    //sys reset
		    {0x3000, 0x20}, // reset MCU
		    {OV5640_REG_DLY, 10}, // delay 10ms
		    {0x3002, 0x1c},

		    //clock enable
		    {0x3004, 0xff},
		    {0x3006, 0xc3},

		    //isp control
		    {0x5000, 0xa7},
		    {OV5640_ISP_CONTROL01, 0xa3},//+scaling?
		    {0x5003, 0x08},//special_effect

		    //unknown
		    {0x370c, 0x02},//!!IMPORTANT
		    {0x3634, 0x40},//!!IMPORTANT

		    //AEC/AGC
		    {0x3a02, 0x03},
		    {0x3a03, 0xd8},
		    {0x3a08, 0x01},
		    {0x3a09, 0x27},
		    {0x3a0a, 0x00},
		    {0x3a0b, 0xf6},
		    {0x3a0d, 0x04},
		    {0x3a0e, 0x03},
		    {0x3a0f, 0x30},//ae_level
		    {0x3a10, 0x28},//ae_level
		    {0x3a11, 0x60},//ae_level
		    {0x3a13, 0x43},
		    {0x3a14, 0x03},
		    {0x3a15, 0xd8},
		    {0x3a18, 0x00},//gainceiling
		    {0x3a19, 0xf8},//gainceiling
		    {0x3a1b, 0x30},//ae_level
		    {0x3a1e, 0x26},//ae_level
		    {0x3a1f, 0x14},//ae_level

		    //vcm debug
		    {0x3600, 0x08},
		    {0x3601, 0x33},

		    //50/60Hz
		    {0x3c01, 0xa4},
		    {0x3c04, 0x28},
		    {0x3c05, 0x98},
		    {0x3c06, 0x00},
		    {0x3c07, 0x08},
		    {0x3c08, 0x00},
		    {0x3c09, 0x1c},
		    {0x3c0a, 0x9c},
		    {0x3c0b, 0x40},

		    {0x460c, 0x22},//disable jpeg footer

		    //BLC
		    {0x4001, 0x02},
		    {0x4004, 0x02},

		    //AWB
		    {0x5180, 0xff},
		    {0x5181, 0xf2},
		    {0x5182, 0x00},
		    {0x5183, 0x14},
		    {0x5184, 0x25},
		    {0x5185, 0x24},
		    {0x5186, 0x09},
		    {0x5187, 0x09},
		    {0x5188, 0x09},
		    {0x5189, 0x75},
		    {0x518a, 0x54},
		    {0x518b, 0xe0},
		    {0x518c, 0xb2},
		    {0x518d, 0x42},
		    {0x518e, 0x3d},
		    {0x518f, 0x56},
		    {0x5190, 0x46},
		    {0x5191, 0xf8},
		    {0x5192, 0x04},
		    {0x5193, 0x70},
		    {0x5194, 0xf0},
		    {0x5195, 0xf0},
		    {0x5196, 0x03},
		    {0x5197, 0x01},
		    {0x5198, 0x04},
		    {0x5199, 0x12},
		    {0x519a, 0x04},
		    {0x519b, 0x00},
		    {0x519c, 0x06},
		    {0x519d, 0x82},
		    {0x519e, 0x38},

		    //color matrix (Saturation)
		    {0x5381, 0x1e},
		    {0x5382, 0x5b},
		    {0x5383, 0x08},
		    {0x5384, 0x0a},
		    {0x5385, 0x7e},
		    {0x5386, 0x88},
		    {0x5387, 0x7c},
		    {0x5388, 0x6c},
		    {0x5389, 0x10},
		    {0x538a, 0x01},
		    {0x538b, 0x98},

		    //CIP control (Sharpness)
		    {0x5300, 0x10},//sharpness
		    {0x5301, 0x10},//sharpness
		    {0x5302, 0x18},//sharpness
		    {0x5303, 0x19},//sharpness
		    {0x5304, 0x10},
		    {0x5305, 0x10},
		    {0x5306, 0x08},//denoise
		    {0x5307, 0x16},
		    {0x5308, 0x40},
		    {0x5309, 0x10},//sharpness
		    {0x530a, 0x10},//sharpness
		    {0x530b, 0x04},//sharpness
		    {0x530c, 0x06},//sharpness

		    //GAMMA
		    {0x5480, 0x01},
		    {0x5481, 0x00},
		    {0x5482, 0x1e},
		    {0x5483, 0x3b},
		    {0x5484, 0x58},
		    {0x5485, 0x66},
		    {0x5486, 0x71},
		    {0x5487, 0x7d},
		    {0x5488, 0x83},
		    {0x5489, 0x8f},
		    {0x548a, 0x98},
		    {0x548b, 0xa6},
		    {0x548c, 0xb8},
		    {0x548d, 0xca},
		    {0x548e, 0xd7},
		    {0x548f, 0xe3},
		    {0x5490, 0x1d},

		    //Special Digital Effects (SDE) (UV adjust)
		    {0x5580, 0x06},//enable brightness and contrast
		    {0x5583, 0x40},//special_effect
		    {0x5584, 0x10},//special_effect
		    {0x5586, 0x20},//contrast
		    {0x5587, 0x00},//brightness
		    {0x5588, 0x00},//brightness
		    {0x5589, 0x10},
		    {0x558a, 0x00},
		    {0x558b, 0xf8},
		    {0x501d, 0x40},// enable manual offset of contrast

			//{0x503d, 0x80}, // color bar test pattern

		    //power on
		    {0x3008, 0x02},

		    //50Hz
		    {0x3c00, 0x04},

		    {OV5640_REG_DLY, 300}
	};

	HAL_StatusTypeDef ok = HAL_OK;
	for (int i = 0; i < sizeof(OV5640_Common)/4U; i++) {
		if (OV5640_Common[i][0] == OV5640_REG_DLY) {
			HAL_Delay(OV5640_Common[i][1]);
		} else {
			ok = ov5640_write_reg(OV5640_Common[i][0], OV5640_Common[i][1]);
			if (ok != HAL_OK) {
				break;
			}
		}
	}

	ok |= ov5640_write_addr_reg(OV5640_TIMING_HS_HIGH, 0, 240);
	ok |= ov5640_write_addr_reg(OV5640_TIMING_HW_HIGH, 2623, 1711);
	ok |= ov5640_write_addr_reg(OV5640_TIMING_DVPHO_HIGH, 1920, 1080);
	ok |= ov5640_write_addr_reg(OV5640_TIMING_HTS_HIGH, 2844, 1488);
	ok |= ov5640_write_addr_reg(OV5640_TIMING_HOFFSET_HIGH, 32, 16);
	// ISP_CONTROL_0 scaling already enabled
	ok |= ov5640_write_reg(OV5640_TIMING_TC_REG20, 0x40);
	ok |= ov5640_write_reg(OV5640_TIMING_TC_REG21, 0x20);
	ok |= ov5640_write_reg(0x4514, 0x88);
	ok |= ov5640_write_reg(0x4520, 0x10);
	ok |= ov5640_write_reg(OV5640_TIMING_X_INC, 0x11);//odd:1, even: 1
	ok |= ov5640_write_reg(OV5640_TIMING_Y_INC, 0x11);//odd:1, even: 1
	ok |= set_pll(0, 160, 4, 2, 0, 2, 1, 4);
    //Set PLL: bypass: 0, multiplier: 200, sys_div: 4, pre_div: 2, root_2x: 0, pclk_root_div: 2, pclk_manual: 1, pclk_div: 4

	static const uint16_t sensor_fmt_jpeg[][2] = {
	    {OV5640_FORMAT_MUX_CTRL, 0x00}, // YUV422
	    {OV5640_FORMAT_CTRL00, 0x30}, // YUYV
	    {0x3002, 0x00},//0x1c to 0x00 !!!
	    {0x3006, 0xff},//0xc3 to 0xff !!!
	    {0x471c, 0x50},//0xd0 to 0x50 !!!
	};
	for (int i = 0; i < sizeof(sensor_fmt_jpeg)/4U; i++) {
		ok |= ov5640_write_reg(sensor_fmt_jpeg[i][0], sensor_fmt_jpeg[i][1]);
		if (ok != HAL_OK) {
			break;
		}
	}

	ok |= ov5640_write_reg(OV5640_JPEG_CTRL07, 10 & 0x3f);

	// Default regs
	// Framesize
	// Pixfmt
	// Quality


	return ok;
}
