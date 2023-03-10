/*
 * Support for OmniVision OV2688 5M camera sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __OV2688_H__
#define __OV2688_H__
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>

#include <linux/atomisp_platform.h>

#define OV2688_NAME		"ov2688"
#define OV2688B_NAME	"ov2688b"
#define OV2688F_NAME	"ov2688f"

/* Defines for register writes and register array processing */
#define I2C_MSG_LENGTH		0x2
#define I2C_RETRY_COUNT		5

#define OV2688_FOCAL_LENGTH_NUM	334	/*3.34mm*/
#define OV2688_FOCAL_LENGTH_DEM	100
#define OV2688_F_NUMBER_DEFAULT_NUM	24
#define OV2688_F_NUMBER_DEM	10

#define OV2688_BIN_FACTOR_MAX 4

#define MAX_FMTS		1

/* sensor_mode_data read_mode adaptation */
#define OV2688_READ_MODE_BINNING_ON	0x0400
#define OV2688_READ_MODE_BINNING_OFF	0x00
#define OV2688_INTEGRATION_TIME_MARGIN	8

#define OV2688_MAX_EXPOSURE_VALUE	0xFFF1
#define OV2688_MAX_GAIN_VALUE		0xFF

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV2688_FOCAL_LENGTH_DEFAULT 0x1B70064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV2688_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define OV2688_F_NUMBER_RANGE 0x180a180a
#define OV2688_ID	0x2680

#define OV2688_FINE_INTG_TIME_MIN 0
#define OV2688_FINE_INTG_TIME_MAX_MARGIN 0
#define OV2688_COARSE_INTG_TIME_MIN 1
#define OV2688_COARSE_INTG_TIME_MAX_MARGIN 6

/*
 * OV2688 System control registers
 */
#define OV2688_SW_SLEEP				0x0100
#define OV2688_SW_RESET				0x0103
#define OV2688_SW_STREAM			0x0100

#define OV2688_SC_CMMN_CHIP_ID_H		0x300A
#define OV2688_SC_CMMN_CHIP_ID_L		0x300B
#define OV2688_SC_CMMN_SCCB_ID			0x302B /* 0x300C*/
#define OV2688_SC_CMMN_SUB_ID			0x302A /* process, version*/

#define OV2688_GROUP_ACCESS							0x3208 /*Bit[7:4] Group control, Bit[3:0] Group ID*/

#define OV2688_EXPOSURE_H							0x3500 /*Bit[3:0] Bit[19:16] of exposure, remaining 16 bits lies in Reg0x3501&Reg0x3502*/
#define OV2688_EXPOSURE_M							0x3501
#define OV2688_EXPOSURE_L							0x3502
#define OV2688_AGC_H								0x350A /*Bit[1:0] means Bit[9:8] of gain*/
#define OV2688_AGC_L								0x350B /*Bit[7:0] of gain*/

#define OV2688_HORIZONTAL_START_H					0x3800 /*Bit[11:8]*/
#define OV2688_HORIZONTAL_START_L					0x3801 /*Bit[7:0]*/
#define OV2688_VERTICAL_START_H						0x3802 /*Bit[11:8]*/
#define OV2688_VERTICAL_START_L						0x3803 /*Bit[7:0]*/
#define OV2688_HORIZONTAL_END_H						0x3804 /*Bit[11:8]*/
#define OV2688_HORIZONTAL_END_L						0x3805 /*Bit[7:0]*/
#define OV2688_VERTICAL_END_H						0x3806 /*Bit[11:8]*/
#define OV2688_VERTICAL_END_L						0x3807 /*Bit[7:0]*/
#define OV2688_HORIZONTAL_OUTPUT_SIZE_H				0x3808 /*Bit[3:0]*/
#define OV2688_HORIZONTAL_OUTPUT_SIZE_L				0x3809 /*Bit[7:0]*/
#define OV2688_VERTICAL_OUTPUT_SIZE_H				0x380a /*Bit[3:0]*/
#define OV2688_VERTICAL_OUTPUT_SIZE_L				0x380b /*Bit[7:0]*/
#define OV2688_TIMING_HTS_H							0x380C  /*High 8-bit, and low 8-bit HTS address is 0x380d*/
#define OV2688_TIMING_HTS_L							0x380D  /*High 8-bit, and low 8-bit HTS address is 0x380d*/
#define OV2688_TIMING_VTS_H							0x380e  /*High 8-bit, and low 8-bit HTS address is 0x380f*/
#define OV2688_TIMING_VTS_L							0x380f  /*High 8-bit, and low 8-bit HTS address is 0x380f*/
#define OV2688_FRAME_OFF_NUM						0x4202

/*Flip/Mirror*/
#define OV2688_FLIP_REG				0x3820
#define OV2688_MIRROR_REG			0x3821
#define OV2688_FLIP_BIT				1
#define OV2688_MIRROR_BIT			2
#define OV2688_FLIP_MIRROR_BIT_ENABLE		4

#define OV2688_MWB_RED_GAIN_H			0x5004/*0x3400*/
#define OV2688_MWB_GREEN_GAIN_H			0x5006/*0x3402*/
#define OV2688_MWB_BLUE_GAIN_H			0x5008/*0x3404*/
#define OV2688_MWB_GAIN_MAX				0x0fff

#define OV2688_START_STREAMING			0x01
#define OV2688_STOP_STREAMING			0x00


#define OV2688_INVALID_CONFIG	0xffffffff


struct regval_list {
	u16 reg_num;
	u8 value;
};

struct ov2688_resolution {
	u8 *desc;
	const struct ov2688_reg *regs;
	int res;
	int width;
	int height;
	int fps;
	int pix_clk_freq;
	u32 skip_frames;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 bin_factor_x;
	u8 bin_factor_y;
	u8 bin_mode;
	bool used;
};

struct ov2688_format {
	u8 *desc;
	u32 pixelformat;
	struct ov2688_reg *regs;
};

struct ov2688_control {
		struct v4l2_queryctrl qc;
		int (*query)(struct v4l2_subdev *sd, s32 *value);
		int (*tweak)(struct v4l2_subdev *sd, s32 value);
	};



	/*
	 * ov2688 device structure.
	 */
	struct ov2688_device {
		struct v4l2_subdev sd;
		struct media_pad pad;
		struct v4l2_mbus_framefmt format;
		struct mutex input_lock;

		struct camera_sensor_platform_data *platform_data;
		struct timespec timestamp_t_focus_abs;
		int vt_pix_clk_freq_mhz;
		int fmt_idx;
		int run_mode;
		u8 res;
		u8 type;
	};

	enum ov2688_tok_type {
		OV2688_8BIT  = 0x0001,
		OV2688_16BIT = 0x0002,
		OV2688_32BIT = 0x0004,
		OV2688_TOK_TERM   = 0xf000,	/* terminating token for reg list */
		OV2688_TOK_DELAY  = 0xfe00,	/* delay token for reg list */
		OV2688_TOK_MASK = 0xfff0
	};

	/**
	 * struct ov2688_reg - MI sensor  register format
	 * @type: type of the register
	 * @reg: 16-bit offset to register
	 * @val: 8/16/32-bit register value
	 *
	 * Define a structure for sensor register initialization values
	 */
	struct ov2688_reg {
		enum ov2688_tok_type type;
		u16 reg;
		u32 val;	/* @set value for read/mod/write, @mask */
	};

	#define to_ov2688_sensor(x) container_of(x, struct ov2688_device, sd)

	#define OV2688_MAX_WRITE_BUF_SIZE	30

	struct ov2688_write_buffer {
		u16 addr;
		u8 data[OV2688_MAX_WRITE_BUF_SIZE];
	};

	struct ov2688_write_ctrl {
		int index;
		struct ov2688_write_buffer buffer;
	};

	static const struct i2c_device_id ov2688_id[] = {
		{OV2688B_NAME, 0},
		{OV2688F_NAME, 0},
        {}
	};

	static struct ov2688_reg const ov2688_global_setting[] = {
	    {OV2688_8BIT, 0x0103, 0x01},
	    {OV2688_8BIT, 0x3002, 0x00},
	    {OV2688_8BIT, 0x3016, 0x1c},
	    {OV2688_8BIT, 0x3018, 0x44},
	    {OV2688_8BIT, 0x3020, 0x00},
	    {OV2688_8BIT, 0x3080, 0x02},
	    {OV2688_8BIT, 0x3082, 0x45},
	    {OV2688_8BIT, 0x3084, 0x09},
	    {OV2688_8BIT, 0x3085, 0x04},
	    {OV2688_8BIT, 0x3503, 0x03},
	    {OV2688_8BIT, 0x350b, 0x36},
	    {OV2688_8BIT, 0x3600, 0xb4},
	    {OV2688_8BIT, 0x3603, 0x39},
	    {OV2688_8BIT, 0x3604, 0x24},
	    {OV2688_8BIT, 0x3605, 0x00},
	    {OV2688_8BIT, 0x3620, 0x26},
	    {OV2688_8BIT, 0x3621, 0x37},
	    {OV2688_8BIT, 0x3622, 0x04},
	    {OV2688_8BIT, 0x3628, 0x00},
	    {OV2688_8BIT, 0x3705, 0x3c},
	    {OV2688_8BIT, 0x370c, 0x50},
	    {OV2688_8BIT, 0x370d, 0xc0},
	    {OV2688_8BIT, 0x3718, 0x88},
	    {OV2688_8BIT, 0x3720, 0x00},
	    {OV2688_8BIT, 0x3721, 0x00},
	    {OV2688_8BIT, 0x3722, 0x00},
	    {OV2688_8BIT, 0x3723, 0x00},
	    {OV2688_8BIT, 0x3738, 0x00},
	    {OV2688_8BIT, 0x3717, 0x58},
	    {OV2688_8BIT, 0x3781, 0x80},
	    {OV2688_8BIT, 0x3789, 0x60},
	    {OV2688_8BIT, 0x3800, 0x00},
	    {OV2688_8BIT, 0x3819, 0x04},
	    {OV2688_8BIT, 0x4000, 0x81},
	    {OV2688_8BIT, 0x4001, 0x40},
	    {OV2688_8BIT, 0x4602, 0x02},
	    {OV2688_8BIT, 0x481f, 0x36},
	    {OV2688_8BIT, 0x4825, 0x36},
	    {OV2688_8BIT, 0x4837, 0x18},
	    {OV2688_8BIT, 0x5002, 0x30},
	    {OV2688_8BIT, 0x5004, 0x04},//manual awb 1x
	    {OV2688_8BIT, 0x5005, 0x00},
	    {OV2688_8BIT, 0x5006, 0x04},
	    {OV2688_8BIT, 0x5007, 0x00},
	    {OV2688_8BIT, 0x5008, 0x04},
	    {OV2688_8BIT, 0x5009, 0x00},
	    {OV2688_8BIT, 0x5080, 0x00},
	    {OV2688_8BIT, 0x3701, 0x64},  //add on 14/05/13
	    {OV2688_8BIT, 0x3784, 0x0c},  //based OV2688_R1A_AM10.ovt add on 14/06/13
	    {OV2688_8BIT, 0x5780, 0x3e},  //based OV2688_R1A_AM10.ovt,Adjust DPC setting (57xx) on 14/06/13
	    {OV2688_8BIT, 0x5781, 0x0f},
	    {OV2688_8BIT, 0x5782, 0x04},
	    {OV2688_8BIT, 0x5783, 0x02},
	    {OV2688_8BIT, 0x5784, 0x01},
	    {OV2688_8BIT, 0x5785, 0x01},
	    {OV2688_8BIT, 0x5786, 0x00},
	    {OV2688_8BIT, 0x5787, 0x04},
	    {OV2688_8BIT, 0x5788, 0x02},
	    {OV2688_8BIT, 0x5789, 0x00},
	    {OV2688_8BIT, 0x578a, 0x01},
	    {OV2688_8BIT, 0x578b, 0x02},
	    {OV2688_8BIT, 0x578c, 0x03},
	    {OV2688_8BIT, 0x578d, 0x03},
	    {OV2688_8BIT, 0x578e, 0x08},
	    {OV2688_8BIT, 0x578f, 0x0c},
	    {OV2688_8BIT, 0x5790, 0x08},
	    {OV2688_8BIT, 0x5791, 0x04},
	    {OV2688_8BIT, 0x5792, 0x00},
	    {OV2688_8BIT, 0x5793, 0x00},
	    {OV2688_8BIT, 0x5794, 0x03}, //based OV2688_R1A_AM10.ovt,Adjust DPC setting (57xx) on 14/06/13
		{OV2688_8BIT, 0x0100, 0x00},	//stream off

		{OV2688_TOK_TERM, 0, 0}
	};


	/*
	 * 176x144 30fps  VBlanking 1lane 10Bit (binning)
	 */
	static struct ov2688_reg const ov2688_QCIF_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x01},
	{OV2688_8BIT, 0x3501, 0x24},
	{OV2688_8BIT, 0x3502, 0x40},
	{OV2688_8BIT, 0x370a, 0x23},
	{OV2688_8BIT, 0x3801, 0xa0},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x78},
	{OV2688_8BIT, 0x3804, 0x05},
	{OV2688_8BIT, 0x3805, 0xaf},
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0x47},
	{OV2688_8BIT, 0x3808, 0x00},
	{OV2688_8BIT, 0x3809, 0xC0},
	{OV2688_8BIT, 0x380a, 0x00},
	{OV2688_8BIT, 0x380b, 0xa0},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xb0},
	{OV2688_8BIT, 0x380e, 0x02},
	{OV2688_8BIT, 0x380f, 0x84},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x04},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x04},
	{OV2688_8BIT, 0x3814, 0x31},
	{OV2688_8BIT, 0x3815, 0x31},
	{OV2688_8BIT, 0x4000, 0x81},
	{OV2688_8BIT, 0x4001, 0x40},
	{OV2688_8BIT, 0x4008, 0x00},
	{OV2688_8BIT, 0x4009, 0x03},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x00}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x10},
	{OV2688_8BIT, 0x5705, 0xa0},
	{OV2688_8BIT, 0x5706, 0x0c},
	{OV2688_8BIT, 0x5707, 0x78},
	{OV2688_8BIT, 0x3820, 0xc2},
	{OV2688_8BIT, 0x3821, 0x01},
    // {OV2688_8BIT, 0x5090, 0x0c},
    {OV2688_TOK_TERM, 0, 0}
	};

	/*
	 * 352x288 30fps  VBlanking 1lane 10Bit (binning)
	 */
	static struct ov2688_reg const ov2688_CIF_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x01},
	{OV2688_8BIT, 0x3501, 0x24},
	{OV2688_8BIT, 0x3502, 0x40},
	{OV2688_8BIT, 0x370a, 0x23},
	{OV2688_8BIT, 0x3801, 0xa0},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x78},
	{OV2688_8BIT, 0x3804, 0x03},
	{OV2688_8BIT, 0x3805, 0x8f},
	{OV2688_8BIT, 0x3806, 0x02},
	{OV2688_8BIT, 0x3807, 0xe7},
	{OV2688_8BIT, 0x3808, 0x01},
	{OV2688_8BIT, 0x3809, 0x70},
	{OV2688_8BIT, 0x380a, 0x01},
	{OV2688_8BIT, 0x380b, 0x30},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xb0},
	{OV2688_8BIT, 0x380e, 0x02},
	{OV2688_8BIT, 0x380f, 0x84},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x04},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x04},
	{OV2688_8BIT, 0x3814, 0x31},
	{OV2688_8BIT, 0x3815, 0x31},
	{OV2688_8BIT, 0x4008, 0x00},
	{OV2688_8BIT, 0x4009, 0x03},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x00}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x10},
	{OV2688_8BIT, 0x5705, 0xa0},
	{OV2688_8BIT, 0x5706, 0x0c},
	{OV2688_8BIT, 0x5707, 0x78},
	{OV2688_8BIT, 0x3820, 0xc2},
	{OV2688_8BIT, 0x3821, 0x01},
    // {OV2688_8BIT, 0x5090, 0x0c},
    {OV2688_TOK_TERM, 0, 0}
	};

	/*
	 * 336x256 30fps  VBlanking 1lane 10Bit (binning)
	 */
	static struct ov2688_reg const ov2688_QVGA_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x01},
	{OV2688_8BIT, 0x3501, 0x24},
	{OV2688_8BIT, 0x3502, 0x40},
	{OV2688_8BIT, 0x370a, 0x23},
	{OV2688_8BIT, 0x3801, 0xa0},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x78},
	{OV2688_8BIT, 0x3804, 0x03},
	{OV2688_8BIT, 0x3805, 0x4f},
	{OV2688_8BIT, 0x3806, 0x02},
	{OV2688_8BIT, 0x3807, 0x87},
	{OV2688_8BIT, 0x3808, 0x01},
	{OV2688_8BIT, 0x3809, 0x50},
	{OV2688_8BIT, 0x380a, 0x01},
	{OV2688_8BIT, 0x380b, 0x00},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xb0},
	{OV2688_8BIT, 0x380e, 0x02},
	{OV2688_8BIT, 0x380f, 0x84},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x04},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x04},
	{OV2688_8BIT, 0x3814, 0x31},
	{OV2688_8BIT, 0x3815, 0x31},
	{OV2688_8BIT, 0x4008, 0x00},
	{OV2688_8BIT, 0x4009, 0x03},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x00}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x10},
	{OV2688_8BIT, 0x5705, 0xa0},
	{OV2688_8BIT, 0x5706, 0x0c},
	{OV2688_8BIT, 0x5707, 0x78},
	{OV2688_8BIT, 0x3820, 0xc2},
	{OV2688_8BIT, 0x3821, 0x01},
    // {OV2688_8BIT, 0x5090, 0x0c},
    {OV2688_TOK_TERM, 0, 0}
	};


	/*
	 * 656x496 30fps  VBlanking 1lane 10Bit (binning)
	 */
	static struct ov2688_reg const ov2688_656x496_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x01},
	{OV2688_8BIT, 0x3501, 0x24},
	{OV2688_8BIT, 0x3502, 0x40},
	{OV2688_8BIT, 0x370a, 0x23},
	{OV2688_8BIT, 0x3801, 0xa0},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x78},
	{OV2688_8BIT, 0x3804, 0x05},
	{OV2688_8BIT, 0x3805, 0xcf},
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0x67},
	{OV2688_8BIT, 0x3808, 0x02},
	{OV2688_8BIT, 0x3809, 0x90},
	{OV2688_8BIT, 0x380a, 0x01},
	{OV2688_8BIT, 0x380b, 0xf0},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xb0},
	{OV2688_8BIT, 0x380e, 0x02},
	{OV2688_8BIT, 0x380f, 0x84},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x04},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x04},
	{OV2688_8BIT, 0x3814, 0x31},
	{OV2688_8BIT, 0x3815, 0x31},
	{OV2688_8BIT, 0x4008, 0x00},
	{OV2688_8BIT, 0x4009, 0x03},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x00}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x10},
	{OV2688_8BIT, 0x5705, 0xa0},
	{OV2688_8BIT, 0x5706, 0x0c},
	{OV2688_8BIT, 0x5707, 0x78},
	{OV2688_8BIT, 0x3820, 0xc2},
	{OV2688_8BIT, 0x3821, 0x01},
    // {OV2688_8BIT, 0x5090, 0x0c},
    {OV2688_TOK_TERM, 0, 0}
	};
	/*
	* 800x600 30fps  VBlanking 1lane 10Bit (binning)
	*/
	static struct ov2688_reg const ov2688_720x592_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x01},
	{OV2688_8BIT, 0x3501, 0x26},
	{OV2688_8BIT, 0x3502, 0x40},
	{OV2688_8BIT, 0x370a, 0x23},
	{OV2688_8BIT, 0x3801, 0x00}, // X_ADDR_START;
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x00}, // Y_ADDR_START;
	{OV2688_8BIT, 0x3804, 0x05},
	{OV2688_8BIT, 0x3805, 0xaf}, // X_ADDR_END;
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0xaf}, // Y_ADDR_END;
	{OV2688_8BIT, 0x3808, 0x02},
	{OV2688_8BIT, 0x3809, 0xd0}, // X_OUTPUT_SIZE;
	{OV2688_8BIT, 0x380a, 0x02},
	{OV2688_8BIT, 0x380b, 0x50}, // Y_OUTPUT_SIZE;
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xac}, // HTS;
	{OV2688_8BIT, 0x380e, 0x02},
	{OV2688_8BIT, 0x380f, 0x84}, // VTS;
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x00},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x00},
	{OV2688_8BIT, 0x3814, 0x31},
	{OV2688_8BIT, 0x3815, 0x31},
	{OV2688_8BIT, 0x4008, 0x00},
	{OV2688_8BIT, 0x4009, 0x03},
	{OV2688_8BIT, 0x5708, 0x00},
	{OV2688_8BIT, 0x5704, 0x02},
	{OV2688_8BIT, 0x5705, 0xd0}, // X_WIN;
	{OV2688_8BIT, 0x5706, 0x02},
	{OV2688_8BIT, 0x5707, 0x50}, // Y_WIN;
	{OV2688_8BIT, 0x3820, 0xc2}, // FLIP_FORMAT;
	{OV2688_8BIT, 0x3821, 0x01}, // MIRROR_FORMAT;
    {OV2688_8BIT, 0x5090, 0x00}, // PRE ISP CTRL16, default value is 0x0C;
                                 // BIT[3]: Mirror order, BG or GB;
                                 // BIT[2]: Flip order, BR or RB;
	{OV2688_8BIT, 0x5081, 0x41},
    {OV2688_TOK_TERM, 0, 0}
	};

	/*
	* 800x600 30fps  VBlanking 1lane 10Bit (binning)
	*/
	static struct ov2688_reg const ov2688_800x600_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x01},
	{OV2688_8BIT, 0x3501, 0x26},
	{OV2688_8BIT, 0x3502, 0x40},
	{OV2688_8BIT, 0x370a, 0x23},
	{OV2688_8BIT, 0x3801, 0x00},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x00},
	{OV2688_8BIT, 0x3804, 0x06},
	{OV2688_8BIT, 0x3805, 0x4f},
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0xbf},
	{OV2688_8BIT, 0x3808, 0x03},
	{OV2688_8BIT, 0x3809, 0x20},
	{OV2688_8BIT, 0x380a, 0x02},
	{OV2688_8BIT, 0x380b, 0x58},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xac},
	{OV2688_8BIT, 0x380e, 0x02},
	{OV2688_8BIT, 0x380f, 0x84},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x00},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x00},
	{OV2688_8BIT, 0x3814, 0x31},
	{OV2688_8BIT, 0x3815, 0x31},
	{OV2688_8BIT, 0x5708, 0x00},
	{OV2688_8BIT, 0x5704, 0x03},
	{OV2688_8BIT, 0x5705, 0x20},
	{OV2688_8BIT, 0x5706, 0x02},
	{OV2688_8BIT, 0x5707, 0x58},
	{OV2688_8BIT, 0x3820, 0xc2},
	{OV2688_8BIT, 0x3821, 0x01},
    {OV2688_8BIT, 0x5090, 0x00},
	{OV2688_8BIT, 0x4008, 0x00},
	{OV2688_8BIT, 0x4009, 0x03},
	{OV2688_8BIT, 0x5081, 0x41},
    {OV2688_TOK_TERM, 0, 0}
	};

	/*
	 * 720p=1280*720 30fps  VBlanking 1lane 10Bit (no-Scaling)
	 */
	static struct ov2688_reg const ov2688_720p_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x00},
	{OV2688_8BIT, 0x3501, 0x48},
	{OV2688_8BIT, 0x3502, 0xe0},
	{OV2688_8BIT, 0x370a, 0x21},
	{OV2688_8BIT, 0x3801, 0xa0},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0xf2},
	{OV2688_8BIT, 0x3804, 0x05},
	{OV2688_8BIT, 0x3805, 0xbf},
	{OV2688_8BIT, 0x3806, 0x03},
	{OV2688_8BIT, 0x3807, 0xdd},
	{OV2688_8BIT, 0x3808, 0x05},
	{OV2688_8BIT, 0x3809, 0x10},
	{OV2688_8BIT, 0x380a, 0x02},
	{OV2688_8BIT, 0x380b, 0xe0},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xa8},
	{OV2688_8BIT, 0x380e, 0x05},
	{OV2688_8BIT, 0x380f, 0x0e},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x08},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x06},
	{OV2688_8BIT, 0x3814, 0x11},
	{OV2688_8BIT, 0x3815, 0x11},
	{OV2688_8BIT, 0x4008, 0x02},
	{OV2688_8BIT, 0x4009, 0x09},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x00}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x10},
	{OV2688_8BIT, 0x5705, 0xa0},
	{OV2688_8BIT, 0x5706, 0x0c},
	{OV2688_8BIT, 0x5707, 0x78},
	{OV2688_8BIT, 0x3820, 0xc0},
	{OV2688_8BIT, 0x3821, 0x00},
    // {OV2688_8BIT, 0x5090, 0x0c},
    {OV2688_TOK_TERM, 0, 0}
	};

	/*
	 * 1296x976 30fps  VBlanking 1lane 10Bit(no-scaling)
	 */
	static struct ov2688_reg const ov2688_1296x976_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x00},
	{OV2688_8BIT, 0x3501, 0x48},
	{OV2688_8BIT, 0x3502, 0xe0},
	{OV2688_8BIT, 0x370a, 0x21},
	{OV2688_8BIT, 0x3801, 0xa0},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x78},
	{OV2688_8BIT, 0x3804, 0x05},
	{OV2688_8BIT, 0x3805, 0xbf},
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0x57},
	{OV2688_8BIT, 0x3808, 0x05},
	{OV2688_8BIT, 0x3809, 0x10},
	{OV2688_8BIT, 0x380a, 0x03},
	{OV2688_8BIT, 0x380b, 0xd0},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xa8},
	{OV2688_8BIT, 0x380e, 0x05},
	{OV2688_8BIT, 0x380f, 0x0e},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x08},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x08},
	{OV2688_8BIT, 0x3814, 0x11},
	{OV2688_8BIT, 0x3815, 0x11},
	{OV2688_8BIT, 0x4008, 0x02},
	{OV2688_8BIT, 0x4009, 0x09},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x00}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x10},
	{OV2688_8BIT, 0x5705, 0xa0},
	{OV2688_8BIT, 0x5706, 0x0c},
	{OV2688_8BIT, 0x5707, 0x78},
	{OV2688_8BIT, 0x3820, 0xc0},
	{OV2688_8BIT, 0x3821, 0x00}, //miror/flip
    // {OV2688_8BIT, 0x5090, 0x0c},
    {OV2688_TOK_TERM, 0, 0}
 	};

	/*
	 *   1456*1096 30fps  VBlanking 1lane 10bit(no-scaling)
	*/
	static struct ov2688_reg const ov2688_1456x1096_30fps[]= {
	{OV2688_8BIT, 0x3086, 0x00},
	{OV2688_8BIT, 0x3501, 0x48},
	{OV2688_8BIT, 0x3502, 0xe0},
	{OV2688_8BIT, 0x370a, 0x21},
	{OV2688_8BIT, 0x3801, 0x90},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x78},
	{OV2688_8BIT, 0x3804, 0x06},
	{OV2688_8BIT, 0x3805, 0x4f},
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0xC0},
	{OV2688_8BIT, 0x3808, 0x05},
	{OV2688_8BIT, 0x3809, 0xb0},
	{OV2688_8BIT, 0x380a, 0x04},
	{OV2688_8BIT, 0x380b, 0x48},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xa8},
	{OV2688_8BIT, 0x380e, 0x05},
	{OV2688_8BIT, 0x380f, 0x0e},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x08},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x00},
	{OV2688_8BIT, 0x3814, 0x11},
	{OV2688_8BIT, 0x3815, 0x11},
	{OV2688_8BIT, 0x4008, 0x02},
	{OV2688_8BIT, 0x4009, 0x09},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x00}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x10},
	{OV2688_8BIT, 0x5705, 0xa0},
	{OV2688_8BIT, 0x5706, 0x0c},
	{OV2688_8BIT, 0x5707, 0x78},
	{OV2688_8BIT, 0x3820, 0xc0},
	{OV2688_8BIT, 0x3821, 0x00},
    // {OV2688_8BIT, 0x5090, 0x0c},
	{OV2688_TOK_TERM, 0, 0}
	};

	/*
	 *1616x916  30fps  VBlanking 1lane 10bit
	 */

	static struct ov2688_reg const ov2688_1616x916_30fps[] = {

	{OV2688_8BIT, 0x3086, 0x00},
	{OV2688_8BIT, 0x3501, 0x48},
	{OV2688_8BIT, 0x3502, 0xe0},
	{OV2688_8BIT, 0x370a, 0x21},
	{OV2688_8BIT, 0x3801, 0x00},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x96},
	{OV2688_8BIT, 0x3804, 0x06},
	{OV2688_8BIT, 0x3805, 0x4f},
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0x39},
	{OV2688_8BIT, 0x3808, 0x06},
	{OV2688_8BIT, 0x3809, 0x50},
	{OV2688_8BIT, 0x380a, 0x03},
	{OV2688_8BIT, 0x380b, 0x94},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xa8},
	{OV2688_8BIT, 0x380e, 0x05},
	{OV2688_8BIT, 0x380f, 0x0e},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x00},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x08},
	{OV2688_8BIT, 0x3814, 0x11},
	{OV2688_8BIT, 0x3815, 0x11},
	{OV2688_8BIT, 0x4008, 0x02},
	{OV2688_8BIT, 0x4009, 0x09},
	{OV2688_8BIT, 0x5081, 0x41},
	{OV2688_8BIT, 0x5708, 0x01}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x06},
	{OV2688_8BIT, 0x5705, 0x50},
	{OV2688_8BIT, 0x5706, 0x03},
	{OV2688_8BIT, 0x5707, 0x94},
	{OV2688_8BIT, 0x3820, 0xc0},
	{OV2688_8BIT, 0x3821, 0x00},
    // {OV2688_8BIT, 0x5090, 0x0C},
    {OV2688_TOK_TERM, 0, 0}
	};

	/*
	 * 1612x1212 30fps VBlanking 1lane 10Bit
	 */
	static struct ov2688_reg const ov2688_1616x1082_30fps[] = {
       {OV2688_8BIT, 0x3086, 0x00},
       {OV2688_8BIT, 0x3501, 0x48},
       {OV2688_8BIT, 0x3502, 0xe0},
       {OV2688_8BIT, 0x370a, 0x21},
       {OV2688_8BIT, 0x3801, 0x00},
       {OV2688_8BIT, 0x3802, 0x00},
       {OV2688_8BIT, 0x3803, 0x86},
       {OV2688_8BIT, 0x3804, 0x06},
       {OV2688_8BIT, 0x3805, 0x4f},
       {OV2688_8BIT, 0x3806, 0x04},
       {OV2688_8BIT, 0x3807, 0xbf},
       {OV2688_8BIT, 0x3808, 0x06},
       {OV2688_8BIT, 0x3809, 0x50},
       {OV2688_8BIT, 0x380a, 0x04},
       {OV2688_8BIT, 0x380b, 0x3a},
       {OV2688_8BIT, 0x380c, 0x06},
       {OV2688_8BIT, 0x380d, 0xa8},
       {OV2688_8BIT, 0x380e, 0x05},
       {OV2688_8BIT, 0x380f, 0x0e},
       {OV2688_8BIT, 0x3810, 0x00},
       {OV2688_8BIT, 0x3811, 0x00},
       {OV2688_8BIT, 0x3812, 0x00},
       {OV2688_8BIT, 0x3813, 0x00},
       {OV2688_8BIT, 0x3814, 0x11},
       {OV2688_8BIT, 0x3815, 0x11},
       {OV2688_8BIT, 0x5708, 0x01}, //add for full size flip off and mirror off 2014/09/11
       {OV2688_8BIT, 0x5704, 0x06},
       {OV2688_8BIT, 0x5705, 0x50},
       {OV2688_8BIT, 0x5706, 0x04},
       {OV2688_8BIT, 0x5707, 0x3a},
       {OV2688_8BIT, 0x3820, 0xc0},
       {OV2688_8BIT, 0x3821, 0x00},
       // {OV2688_8BIT, 0x5090, 0x0C},
       {OV2688_8BIT, 0x4008, 0x02},
       {OV2688_8BIT, 0x4009, 0x09},
       {OV2688_8BIT, 0x5081, 0x41},
		{OV2688_TOK_TERM, 0, 0}
        };
	/*
	 * 1616x1216 30fps VBlanking 1lane 10Bit
	 */
	static struct ov2688_reg const ov2688_1616x1216_30fps[] = {
	{OV2688_8BIT, 0x3086, 0x00},
	{OV2688_8BIT, 0x3501, 0x48},
	{OV2688_8BIT, 0x3502, 0xe0},
	{OV2688_8BIT, 0x370a, 0x21},
	{OV2688_8BIT, 0x3801, 0x00},
	{OV2688_8BIT, 0x3802, 0x00},
	{OV2688_8BIT, 0x3803, 0x00},
	{OV2688_8BIT, 0x3804, 0x06},
	{OV2688_8BIT, 0x3805, 0x4f},
	{OV2688_8BIT, 0x3806, 0x04},
	{OV2688_8BIT, 0x3807, 0xbf},
	{OV2688_8BIT, 0x3808, 0x06},
	{OV2688_8BIT, 0x3809, 0x50},//50},//4line for mirror and flip
	{OV2688_8BIT, 0x380a, 0x04},
	{OV2688_8BIT, 0x380b, 0xc0},//c0},
	{OV2688_8BIT, 0x380c, 0x06},
	{OV2688_8BIT, 0x380d, 0xa8},
	{OV2688_8BIT, 0x380e, 0x05},
	{OV2688_8BIT, 0x380f, 0x0e},
	{OV2688_8BIT, 0x3810, 0x00},
	{OV2688_8BIT, 0x3811, 0x00},
	{OV2688_8BIT, 0x3812, 0x00},
	{OV2688_8BIT, 0x3813, 0x00},
	{OV2688_8BIT, 0x3814, 0x11},
	{OV2688_8BIT, 0x3815, 0x11},
	{OV2688_8BIT, 0x4008, 0x00},
	{OV2688_8BIT, 0x4009, 0x0b},
	{OV2688_8BIT, 0x5081, 0x01},
	{OV2688_8BIT, 0x5708, 0x01}, //add for full size flip off and mirror off 2014/09/11
	{OV2688_8BIT, 0x5704, 0x06},
	{OV2688_8BIT, 0x5705, 0x50},
	{OV2688_8BIT, 0x5706, 0x04},
	{OV2688_8BIT, 0x5707, 0xcc},
	{OV2688_8BIT, 0x3820, 0xc0},
	{OV2688_8BIT, 0x3821, 0x00},
    // {OV2688_8BIT, 0x5090, 0x0C},
    {OV2688_TOK_TERM, 0, 0}
	};

	static struct ov2688_resolution ov2688_res_preview[] = {
	{
		.desc = "ov2688_1616x1216_30fps",
 	  	.width = 1616,
		.height = 1216,
		.pix_clk_freq = 66,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 1698,//1704,
		.lines_per_frame = 1294,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.bin_mode = 0,
		.skip_frames = 3,
		.regs = ov2688_1616x1216_30fps,
	},
   	{
		.desc = "ov2688_1616x916_30fps",
		.width = 1616,
		.height = 916,
		.fps = 30,
		.pix_clk_freq = 66,
		.used = 0,
		.pixels_per_line = 1698,//1704,
		.lines_per_frame = 1294,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.bin_mode = 0,
		.skip_frames = 3,
		.regs = ov2688_1616x916_30fps,
	},
};
#define N_RES_PREVIEW (ARRAY_SIZE(ov2688_res_preview))


static struct ov2688_resolution ov2688_res_still[] = {
	{
		.desc = "ov2688_1616x1216_30fps",
		.width = 1616,
		.height = 1216,
		.pix_clk_freq = 66,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 1698,//1704,
		.lines_per_frame = 1294,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.bin_mode = 0,
		.skip_frames = 3,
		.regs = ov2688_1616x1216_30fps,
	},
   	{
		.desc = "ov2688_1616x916_30fps",
		.width = 1616,
		.height = 916,
		.fps = 30,
		.pix_clk_freq = 66,
		.used = 0,
		.pixels_per_line = 1698,//1704,
		.lines_per_frame = 1294,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.bin_mode = 0,
		.skip_frames = 3,
		.regs = ov2688_1616x916_30fps,
	},
};
#define N_RES_STILL (ARRAY_SIZE(ov2688_res_still))

static struct ov2688_resolution ov2688_res_video[] = {
	{
		.desc = "ov2688_1616x1216_30fps",
		.width = 1616,
		.height = 1216,
		.pix_clk_freq = 66,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 1698,//1704,
		.lines_per_frame = 1294,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.bin_mode = 0,
		.skip_frames = 3,
		.regs = ov2688_1616x1216_30fps,
	},
	{
		.desc = "ov2688_720p_30fps",
		.width = 1616,
		.height = 916,
		.fps = 30,
		.pix_clk_freq = 66,
		.used = 0,
		.pixels_per_line = 1698,//1704,
		.lines_per_frame = 1294,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.bin_mode = 0,
		.skip_frames = 3,
		.regs = ov2688_1616x916_30fps,
	},
};
#define N_RES_VIDEO (ARRAY_SIZE(ov2688_res_video))

static struct ov2688_resolution *ov2688_res = ov2688_res_preview;
static int N_RES = N_RES_PREVIEW;


#endif
