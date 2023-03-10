/*
 * Support for the Omnivision OV8858 camera sensor.
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
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

#ifndef __OV8858_H__
#define __OV8858_H__

#include <linux/atomisp_platform.h>
#include <media/v4l2-ctrls.h>

#define I2C_MSG_LENGTH					0x2

/*
 * This should be added into include/linux/videodev2.h
 * NOTE: This is most likely not used anywhere.
 */
#define V4L2_IDENT_OV8858				V4L2_IDENT_UNKNOWN

/*
 * Indexes for VCM driver lists
 */
#define OV8858_ID_DEFAULT				0
#define OV8858_SUNNY					1

#define OV8858_OTP_START_ADDR			0x7010
#define OV8858_OTP_END_ADDR				0x720E

/*
 * ov8858 System control registers
 */

#define OV8858_OTP_LOAD_CTRL			0x3D81
#define OV8858_OTP_MODE_CTRL			0x3D84
#define OV8858_OTP_START_ADDR_REG		0x3D88
#define OV8858_OTP_END_ADDR_REG			0x3D8A
#define OV8858_OTP_ISP_CTRL2			0x5002

#define OV8858_OTP_MODE_MANUAL			BIT(6)
#define OV8858_OTP_MODE_PROGRAM_DISABLE	BIT(7)
#define OV8858_OTP_LOAD_ENABLE			BIT(0)
#define OV8858_OTP_DPC_ENABLE			BIT(3)

#define OV8858_PLL1_PREDIV0				0x030A
#define OV8858_PLL1_PREDIV				0x0300
#define OV8858_PLL1_MULTIPLIER			0x0301
#define OV8858_PLL1_SYS_PRE_DIV			0x0305
#define OV8858_PLL1_SYS_DIVIDER			0x0306

#define OV8858_PLL1_PREDIV0_MASK		BIT(0)
#define OV8858_PLL1_PREDIV_MASK			(BIT(0) | BIT(1) | BIT(2))
#define OV8858_PLL1_MULTIPLIER_MASK		0x01FF
#define OV8858_PLL1_SYS_PRE_DIV_MASK	(BIT(0) | BIT(1))
#define OV8858_PLL1_SYS_DIVIDER_MASK	BIT(0)

#define OV8858_PLL2_PREDIV0				0x0312
#define OV8858_PLL2_PREDIV				0x030B
#define OV8858_PLL2_MULTIPLIER			0x030C
#define OV8858_PLL2_DAC_DIVIDER			0x0312
#define OV8858_PLL2_SYS_PRE_DIV			0x030F
#define OV8858_PLL2_SYS_DIVIDER			0x030E

#define OV8858_PLL2_PREDIV0_MASK		BIT(4)
#define OV8858_PLL2_PREDIV_MASK			(BIT(0) | BIT(1) | BIT(2))
#define OV8858_PLL2_MULTIPLIER_MASK		0x01FF
#define OV8858_PLL2_DAC_DIVIDER_MASK	(BIT(0) | BIT(1) | BIT(2) | BIT(3))
#define OV8858_PLL2_SYS_PRE_DIV_MASK	(BIT(0) | BIT(1) | BIT(2) | BIT(3))
#define OV8858_PLL2_SYS_DIVIDER_MASK	(BIT(0) | BIT(1) | BIT(2))

#define OV8858_PLL_SCLKSEL1				0x3032
#define OV8858_PLL_SCLKSEL2				0x3033
#define OV8858_SRB_HOST_INPUT_DIS		0x3106

#define OV8858_PLL_SCLKSEL1_MASK		BIT(7)
#define OV8858_PLL_SCLKSEL2_MASK		BIT(1)

#define OV8858_SYS_PRE_DIV_OFFSET		2
#define OV8858_SYS_PRE_DIV_MASK			(BIT(2) | BIT(3))
#define OV8858_SCLK_PDIV_OFFSET			4
#define OV8858_SCLK_PDIV_MASK			(BIT(4) | BIT(5) | BIT(6) | BIT(7))

#define OV8858_TIMING_HTS				0x380C
#define OV8858_TIMING_VTS				0x380E

#define OV8858_HORIZONTAL_START_H		0x3800
#define OV8858_VERTICAL_START_H			0x3802
#define OV8858_HORIZONTAL_END_H			0x3804
#define OV8858_VERTICAL_END_H			0x3806
#define OV8858_HORIZONTAL_OUTPUT_SIZE_H	0x3808
#define OV8858_VERTICAL_OUTPUT_SIZE_H	0x380A

#define OV8858_GROUP_ACCESS				0x3208
#define OV8858_GROUP_ZERO				0x00
#define OV8858_GROUP_ACCESS_HOLD_START	0x00
#define OV8858_GROUP_ACCESS_HOLD_END	0x10
#define OV8858_GROUP_ACCESS_DELAY_LAUNCH 0xA0
#define OV8858_GROUP_ACCESS_QUICK_LAUNCH 0xE0

#define OV_SUBDEV_PREFIX				"ov"
#define OV_ID_DEFAULT					0x0000
#define	OV8858_NAME						"ov8858"
#define OV8858_CHIP_ID					0x8858

#define OV8858_LONG_EXPO				0x3500
#define OV8858_LONG_GAIN				0x3508
#define OV8858_LONG_DIGI_GAIN			0x350A
#define OV8858_SHORT_GAIN				0x350C
#define OV8858_SHORT_DIGI_GAIN			0x350E

#define OV8858_FORMAT1					0x3820
#define OV8858_FORMAT2					0x3821

#define OV8858_FLIP_ENABLE				0x06

#define OV8858_MWB_RED_GAIN_H			0x5032
#define OV8858_MWB_GREEN_GAIN_H			0x5034
#define OV8858_MWB_BLUE_GAIN_H			0x5036
#define OV8858_MWB_GAIN_MAX				0x0FFF

#define OV8858_CHIP_ID_HIGH				0x300B
#define OV8858_CHIP_ID_LOW				0x300C
#define OV8858_STREAM_MODE				0x0100

#define OV8858_FOCAL_LENGTH_NUM			294
#define OV8858_FOCAL_LENGTH_DEM			100
#define OV8858_F_NUMBER_DEFAULT_NUM		2
#define OV8858_F_NUMBER_DEM				10

#define OV8858_H_INC_ODD				0x3814
#define OV8858_H_INC_EVEN				0x3815
#define OV8858_V_INC_ODD				0x382A
#define OV8858_V_INC_EVEN				0x382B

#define OV8858_READ_MODE_BINNING_ON		0x0400
#define OV8858_READ_MODE_BINNING_OFF	0x00
#define OV8858_BIN_FACTOR_MAX			2
#define OV8858_INTEGRATION_TIME_MARGIN	14

#define OV8858_MAX_VTS_VALUE			0xFFFF
#define OV8858_MAX_EXPOSURE_VALUE \
		(OV8858_MAX_VTS_VALUE - OV8858_INTEGRATION_TIME_MARGIN)
#define OV8858_MAX_GAIN_VALUE			0x07FF

#define OV8858_MAX_FOCUS_POS			1023

#define OV8858_TEST_PATTERN_REG			0x5E00

struct ov8858_vcm {
	int (*power_up)(struct v4l2_subdev *sd);
	int (*power_down)(struct v4l2_subdev *sd);
	int (*init)(struct v4l2_subdev *sd);
	int (*t_focus_vcm)(struct v4l2_subdev *sd, u16 val);
	int (*t_focus_abs)(struct v4l2_subdev *sd, s32 value);
	int (*t_focus_rel)(struct v4l2_subdev *sd, s32 value);
	int (*q_focus_status)(struct v4l2_subdev *sd, s32 *value);
	int (*q_focus_abs)(struct v4l2_subdev *sd, s32 *value);
	int (*t_vcm_slew)(struct v4l2_subdev *sd, s32 value);
	int (*t_vcm_timing)(struct v4l2_subdev *sd, s32 value);
};

/*
 * Defines for register writes and register array processing
 * */
#define OV8858_BYTE_MAX					32
#define OV8858_SHORT_MAX				16
#define OV8858_TOK_MASK					0xFFF0

#define MAX_FPS_OPTIONS_SUPPORTED		3

#define OV8858_DEPTH_COMP_CONST			2200
#define OV8858_DEPTH_VTS_CONST			2573

enum ov8858_tok_type {
	OV8858_8BIT	      = 0x0001,
	OV8858_16BIT	  = 0x0002,
	OV8858_TOK_TERM   = 0xF000,	/* terminating token for reg list */
	OV8858_TOK_DELAY  = 0xFE00	/* delay token for reg list */
};

/*
 * If register address or register width is not 32 bit width,
 * user needs to convert it manually
 */
struct s_register_setting {
	u32 reg;
	u32 val;
};

/**
 * struct ov8858_reg - MI sensor register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct ov8858_reg {
	enum ov8858_tok_type type;
	u16 sreg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

struct ov8858_fps_setting {
	int fps;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	const struct ov8858_reg *regs; /* regs that the fps setting needs */
};

struct ov8858_resolution {
	u8 *desc;
	const struct ov8858_reg *regs;
	int res;
	int width;
	int height;
	bool used;
	u8 bin_factor_x;
	u8 bin_factor_y;
	unsigned short skip_frames;
	const struct ov8858_fps_setting fps_options[MAX_FPS_OPTIONS_SUPPORTED];
};

/*
 * ov8858 device structure
 * */
struct ov8858_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	struct mutex input_lock; /* serialize sensor's ioctl */
	int fmt_idx;
	int streaming;
	int vt_pix_clk_freq_mhz;
	int fps_index;
	u16 sensor_id;			/* Sensor id from registers */
	u16 i2c_id;			/* Sensor id from i2c_device_id */
	int exposure;
	int gain;
	u16 digital_gain;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	u8 *otp_data;
	/* Prevent the framerate from being lowered in low light scenes. */
	int limit_exposure_flag;
	bool hflip;
	bool vflip;

	const struct ov8858_reg *regs;
	struct ov8858_vcm *vcm_driver;
	const struct ov8858_resolution *curr_res_table;
	int entries_curr_table;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *run_mode;
};

#define to_ov8858_sensor(x) container_of(x, struct ov8858_device, sd)
#define OV8858_MAX_WRITE_BUF_SIZE	32

struct ov8858_write_buffer {
	u16 addr;
	u8 data[OV8858_MAX_WRITE_BUF_SIZE];
};

struct ov8858_write_ctrl {
	int index;
	struct ov8858_write_buffer buffer;
};

static const struct ov8858_reg ov8858_soft_standby[] = {
	{OV8858_8BIT, 0x0100, 0x00},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_streaming[] = {
	{OV8858_8BIT, 0x0100, 0x01},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_param_hold[] = {
	{OV8858_8BIT, OV8858_GROUP_ACCESS,
			OV8858_GROUP_ZERO | OV8858_GROUP_ACCESS_HOLD_START},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_param_update[] = {
	{OV8858_8BIT, OV8858_GROUP_ACCESS,
			OV8858_GROUP_ZERO | OV8858_GROUP_ACCESS_HOLD_END},
	{OV8858_8BIT, OV8858_GROUP_ACCESS,
			OV8858_GROUP_ZERO | OV8858_GROUP_ACCESS_DELAY_LAUNCH},
	{OV8858_TOK_TERM, 0, 0}
};

extern int dw9714_vcm_power_up(struct v4l2_subdev *sd);
extern int dw9714_vcm_power_down(struct v4l2_subdev *sd);
extern int dw9714_vcm_init(struct v4l2_subdev *sd);
extern int dw9714_t_focus_vcm(struct v4l2_subdev *sd, u16 val);
extern int dw9714_t_focus_abs(struct v4l2_subdev *sd, s32 value);
extern int dw9714_t_focus_rel(struct v4l2_subdev *sd, s32 value);
extern int dw9714_q_focus_status(struct v4l2_subdev *sd, s32 *value);
extern int dw9714_q_focus_abs(struct v4l2_subdev *sd, s32 *value);
extern int dw9714_t_vcm_slew(struct v4l2_subdev *sd, s32 value);
extern int dw9714_t_vcm_timing(struct v4l2_subdev *sd, s32 value);

extern int vcm_power_up(struct v4l2_subdev *sd);
extern int vcm_power_down(struct v4l2_subdev *sd);

static struct ov8858_vcm ov8858_vcms[] = {
	[OV8858_SUNNY] = {
		.power_up	    = dw9714_vcm_power_up,
		.power_down	    = dw9714_vcm_power_down,
		.init		    = dw9714_vcm_init,
		.t_focus_vcm	= dw9714_t_focus_vcm,
		.t_focus_abs	= dw9714_t_focus_abs,
		.t_focus_rel	= dw9714_t_focus_rel,
		.q_focus_status = dw9714_q_focus_status,
		.q_focus_abs	= dw9714_q_focus_abs,
		.t_vcm_slew	    = dw9714_t_vcm_slew,
		.t_vcm_timing   = dw9714_t_vcm_timing,
	},
	[OV8858_ID_DEFAULT] = {
		.power_up	   = vcm_power_up,
		.power_down	   = vcm_power_down,
	},
};


#define OV8858_RES_WIDTH_MAX	3280
#define OV8858_RES_HEIGHT_MAX	2464

static struct ov8858_reg ov8858_BasicSettings[] = {
	{OV8858_8BIT, 0x0103, 0x01}, /* software_reset */
	{OV8858_8BIT, 0x0100, 0x00}, /* software_standby */
	/* PLL settings */
	{OV8858_8BIT, 0x0300, 0x05}, /* pll1_pre_div = /4 */
	{OV8858_8BIT, 0x0302, 0xAF}, /* pll1_multiplier = 175 */
	{OV8858_8BIT, 0x0303, 0x00}, /* pll1_divm = /(1 + 0) */
	{OV8858_8BIT, 0x0304, 0x03}, /* pll1_div_mipi = /8 */
	{OV8858_8BIT, 0x030B, 0x02}, /* pll2_pre_div = /2 */
	{OV8858_8BIT, 0x030D, 0x4E}, /* pll2_r_divp = 78 */
	{OV8858_8BIT, 0x030E, 0x00}, /* pll2_r_divs = /1 */
	{OV8858_8BIT, 0x030F, 0x04}, /* pll2_r_divsp = /(1 + 4) */
	/* pll2_pre_div0 = /1, pll2_r_divdac = /(1 + 1) */
	{OV8858_8BIT, 0x0312, 0x01},
	{OV8858_8BIT, 0x031E, 0x0C}, /* pll1_no_lat = 1, mipi_bitsel_man = 0 */

	/* PAD OEN2, VSYNC out enable=0x80, disable=0x00 */
	{OV8858_8BIT, 0x3002, 0x80},
	/* PAD OUT2, VSYNC pulse direction low-to-high = 1 */
	{OV8858_8BIT, 0x3007, 0x01},
	/* PAD SEL2, VSYNC out value = 0 */
	/* {OV8858_8BIT, 0x300D, 0x00}, */
	/* PAD OUT2, VSYNC out select = 0 */
	/* {OV8858_8BIT, 0x3010, 0x00}, */

	/* Npump clock div = /2, Ppump clock div = /4 */
	{OV8858_8BIT, 0x3015, 0x01},
	/*
	 * mipi_lane_mode = 1+3, mipi_lvds_sel = 1 = MIPI enable,
	 * r_phy_pd_mipi_man = 0, lane_dis_option = 0
	 */
	{OV8858_8BIT, 0x3018, 0x72},
	/* Clock switch output = normal, pclk_div = /1 */
	{OV8858_8BIT, 0x3020, 0x93},
	/*
	 * lvds_mode_o = 0, clock lane disable when pd_mipi = 0,
	 * pd_mipi enable when rst_sync = 1
	 */
	{OV8858_8BIT, 0x3022, 0x01},
	{OV8858_8BIT, 0x3031, 0x0A}, /* mipi_bit_sel = 10 */
	{OV8858_8BIT, 0x3034, 0x00},
	/* sclk_div = /1, sclk_pre_div = /1, chip debug = 1 */
	{OV8858_8BIT, 0x3106, 0x01},

	{OV8858_8BIT, 0x3305, 0xF1},
	{OV8858_8BIT, 0x3307, 0x04},
	{OV8858_8BIT, 0x3308, 0x00},
	{OV8858_8BIT, 0x3309, 0x28},
	{OV8858_8BIT, 0x330A, 0x00},
	{OV8858_8BIT, 0x330B, 0x20},
	{OV8858_8BIT, 0x330C, 0x00},
	{OV8858_8BIT, 0x330D, 0x00},
	{OV8858_8BIT, 0x330E, 0x00},
	{OV8858_8BIT, 0x330F, 0x40},

	{OV8858_8BIT, 0x3500, 0x00}, /* long exposure = 0x9A20 */
	{OV8858_8BIT, 0x3501, 0x9A}, /* long exposure = 0x9A20 */
	{OV8858_8BIT, 0x3502, 0x20}, /* long exposure = 0x9A20 */
	/*
	 * Digital fraction gain delay option = Delay 1 frame,
	 * Gain change delay option = Delay 1 frame,
	 * Gain delay option = Delay 1 frame,
	 * Gain manual as sensor gain = Input gain as real gain format,
	 * Exposure delay option (must be 0 = Delay 1 frame,
	 * Exposure change delay option (must be 0) = Delay 1 frame
	 */
	{OV8858_8BIT, 0x3503, 0x80},
	{OV8858_8BIT, 0x3505, 0x80}, /* gain conversation option */
	/*
	 * [10:7] are integer gain, [6:0] are fraction gain. For example:
	 * 0x80 is 1x gain, 0x100 is 2x gain, 0x1C0 is 3.5x gain
	 */
	{OV8858_8BIT, 0x3508, 0x07}, /* long gain = 0x0200 */
	{OV8858_8BIT, 0x3509, 0x00}, /* long gain = 0x0200 */
	{OV8858_8BIT, 0x350C, 0x00}, /* short gain = 0x0080 */
	{OV8858_8BIT, 0x350D, 0x80}, /* short gain = 0x0080 */
	{OV8858_8BIT, 0x3510, 0x00}, /* short exposure = 0x000200 */
	{OV8858_8BIT, 0x3511, 0x02}, /* short exposure = 0x000200 */
	{OV8858_8BIT, 0x3512, 0x00}, /* short exposure = 0x000200 */

	{OV8858_8BIT, 0x3600, 0x00},
	{OV8858_8BIT, 0x3601, 0x00},
	{OV8858_8BIT, 0x3602, 0x00},
	{OV8858_8BIT, 0x3603, 0x00},
	{OV8858_8BIT, 0x3604, 0x22},
	{OV8858_8BIT, 0x3605, 0x20},
	{OV8858_8BIT, 0x3606, 0x00},
	{OV8858_8BIT, 0x3607, 0x20},
	{OV8858_8BIT, 0x3608, 0x11},
	{OV8858_8BIT, 0x3609, 0x28},
	{OV8858_8BIT, 0x360A, 0x00},
	{OV8858_8BIT, 0x360B, 0x05},
	{OV8858_8BIT, 0x360C, 0xD4},
	{OV8858_8BIT, 0x360D, 0x40},
	{OV8858_8BIT, 0x360E, 0x0C},
	{OV8858_8BIT, 0x360F, 0x20},
	{OV8858_8BIT, 0x3610, 0x07},
	{OV8858_8BIT, 0x3611, 0x20},
	{OV8858_8BIT, 0x3612, 0x88},
	{OV8858_8BIT, 0x3613, 0x80},
	{OV8858_8BIT, 0x3614, 0x58},
	{OV8858_8BIT, 0x3615, 0x00},
	{OV8858_8BIT, 0x3616, 0x4A},
	{OV8858_8BIT, 0x3617, 0x40},
	{OV8858_8BIT, 0x3618, 0x5a},
	{OV8858_8BIT, 0x3619, 0x70},
	{OV8858_8BIT, 0x361A, 0x99},
	{OV8858_8BIT, 0x361B, 0x0a},
	{OV8858_8BIT, 0x361C, 0x07},
	{OV8858_8BIT, 0x361D, 0x00},
	{OV8858_8BIT, 0x361E, 0x00},
	{OV8858_8BIT, 0x361F, 0x00},
	{OV8858_8BIT, 0x3633, 0x0f},
	{OV8858_8BIT, 0x3634, 0x0f},
	{OV8858_8BIT, 0x3635, 0x0f},
	{OV8858_8BIT, 0x3636, 0x12},
	{OV8858_8BIT, 0x3638, 0xFF},
	{OV8858_8BIT, 0x3645, 0x13},
	{OV8858_8BIT, 0x3646, 0x83},
	{OV8858_8BIT, 0x364A, 0x07},

	{OV8858_8BIT, 0x3700, 0x30},
	{OV8858_8BIT, 0x3701, 0x18},
	{OV8858_8BIT, 0x3702, 0x50},
	{OV8858_8BIT, 0x3703, 0x32},
	{OV8858_8BIT, 0x3704, 0x28},
	{OV8858_8BIT, 0x3705, 0x00},
	{OV8858_8BIT, 0x3706, 0x82},
	{OV8858_8BIT, 0x3707, 0x08},
	{OV8858_8BIT, 0x3708, 0x48},
	{OV8858_8BIT, 0x3709, 0x66},
	{OV8858_8BIT, 0x370A, 0x01},
	{OV8858_8BIT, 0x370B, 0x82},
	{OV8858_8BIT, 0x370C, 0x07},
	{OV8858_8BIT, 0x3712, 0x44},
	{OV8858_8BIT, 0x3714, 0x24},
	{OV8858_8BIT, 0x3718, 0x14},
	{OV8858_8BIT, 0x3719, 0x31},
	{OV8858_8BIT, 0x371E, 0x31},
	{OV8858_8BIT, 0x371F, 0x7F},
	{OV8858_8BIT, 0x3720, 0x0A},
	{OV8858_8BIT, 0x3721, 0x0A},
	{OV8858_8BIT, 0x3724, 0x0C},
	{OV8858_8BIT, 0x3725, 0x02},
	{OV8858_8BIT, 0x3726, 0x0C},
	{OV8858_8BIT, 0x3728, 0x0A},
	{OV8858_8BIT, 0x3729, 0x03},
	{OV8858_8BIT, 0x372A, 0x06},
	{OV8858_8BIT, 0x372B, 0xA6},
	{OV8858_8BIT, 0x372C, 0xA6},
	{OV8858_8BIT, 0x372D, 0xA6},
	{OV8858_8BIT, 0x372E, 0x0C},
	{OV8858_8BIT, 0x372F, 0x20},
	{OV8858_8BIT, 0x3730, 0x02},
	{OV8858_8BIT, 0x3731, 0x0C},
	{OV8858_8BIT, 0x3732, 0x28},
	{OV8858_8BIT, 0x3733, 0x10},
	{OV8858_8BIT, 0x3734, 0x40},
	{OV8858_8BIT, 0x3736, 0x30},
	{OV8858_8BIT, 0x373A, 0x0A},
	{OV8858_8BIT, 0x373B, 0x0B},
	{OV8858_8BIT, 0x373C, 0x14},
	{OV8858_8BIT, 0x373E, 0x06},
	{OV8858_8BIT, 0x3750, 0x0a},
	{OV8858_8BIT, 0x3751, 0x0e},
	{OV8858_8BIT, 0x3755, 0x10},
	{OV8858_8BIT, 0x3758, 0x00},
	{OV8858_8BIT, 0x3759, 0x4C},
	{OV8858_8BIT, 0x375A, 0x0C},
	{OV8858_8BIT, 0x375B, 0x26},
	{OV8858_8BIT, 0x375C, 0x20},
	{OV8858_8BIT, 0x375D, 0x04},
	{OV8858_8BIT, 0x375E, 0x00},
	{OV8858_8BIT, 0x375F, 0x28},
	{OV8858_8BIT, 0x3760, 0x00},
	{OV8858_8BIT, 0x3761, 0x00},
	{OV8858_8BIT, 0x3762, 0x00},
	{OV8858_8BIT, 0x3763, 0x18},
	{OV8858_8BIT, 0x3766, 0xFF},
	{OV8858_8BIT, 0x3768, 0xCC},
	{OV8858_8BIT, 0x3769, 0x44},
	{OV8858_8BIT, 0x376A, 0x44},
	{OV8858_8BIT, 0x376B, 0x00},
	{OV8858_8BIT, 0x376F, 0x01},
	{OV8858_8BIT, 0x3772, 0x46},
	{OV8858_8BIT, 0x3773, 0x04},
	{OV8858_8BIT, 0x3774, 0x2C},
	{OV8858_8BIT, 0x3775, 0x13},
	{OV8858_8BIT, 0x3776, 0x08},
	{OV8858_8BIT, 0x3777, 0x00},
	{OV8858_8BIT, 0x3778, 0x17},
	{OV8858_8BIT, 0x37A0, 0x88},
	{OV8858_8BIT, 0x37A1, 0x7A},
	{OV8858_8BIT, 0x37A2, 0x7A},
	{OV8858_8BIT, 0x37A3, 0x00},
	{OV8858_8BIT, 0x37A4, 0x00},
	{OV8858_8BIT, 0x37A5, 0x00},
	{OV8858_8BIT, 0x37A6, 0x00},
	{OV8858_8BIT, 0x37A7, 0x88},
	{OV8858_8BIT, 0x37A8, 0x98},
	{OV8858_8BIT, 0x37A9, 0x98},
	{OV8858_8BIT, 0x37AA, 0x88},
	{OV8858_8BIT, 0x37AB, 0x5C},
	{OV8858_8BIT, 0x37AC, 0x5C},
	{OV8858_8BIT, 0x37AD, 0x55},
	{OV8858_8BIT, 0x37AE, 0x19},
	{OV8858_8BIT, 0x37AF, 0x19},
	{OV8858_8BIT, 0x37B0, 0x00},
	{OV8858_8BIT, 0x37B1, 0x00},
	{OV8858_8BIT, 0x37B2, 0x00},
	{OV8858_8BIT, 0x37B3, 0x84},
	{OV8858_8BIT, 0x37B4, 0x84},
	{OV8858_8BIT, 0x37B5, 0x60},
	{OV8858_8BIT, 0x37B6, 0x00},
	{OV8858_8BIT, 0x37B7, 0x00},
	{OV8858_8BIT, 0x37B8, 0x00},
	{OV8858_8BIT, 0x37B9, 0xFF},

	{OV8858_8BIT, 0x3800, 0x00}, /* h_crop_start high */
	{OV8858_8BIT, 0x3801, 0x0C}, /* h_crop_start low */
	{OV8858_8BIT, 0x3802, 0x00}, /* v_crop_start high */
	{OV8858_8BIT, 0x3803, 0x0C}, /* v_crop_start low */
	{OV8858_8BIT, 0x3804, 0x0C}, /* h_crop_end high */
	{OV8858_8BIT, 0x3805, 0xD3}, /* h_crop_end low */
	{OV8858_8BIT, 0x3806, 0x09}, /* v_crop_end high */
	{OV8858_8BIT, 0x3807, 0xA3}, /* v_crop_end low */
	{OV8858_8BIT, 0x3808, 0x0C}, /* h_output_size high */
	{OV8858_8BIT, 0x3809, 0xD0}, /* h_output_size low */
	{OV8858_8BIT, 0x380A, 0x09}, /* v_output_size high */
	{OV8858_8BIT, 0x380B, 0xA0}, /* v_output_size low */
	{OV8858_8BIT, 0x380C, 0x07}, /* horizontal timing size high */
	{OV8858_8BIT, 0x380D, 0x94}, /* horizontal timing size low */
	{OV8858_8BIT, 0x380E, 0x0A}, /* vertical timing size high */
	{OV8858_8BIT, 0x380F, 0x0D}, /* vertical timing size low */
	{OV8858_8BIT, 0x3810, 0x00}, /* h_win offset high */
	{OV8858_8BIT, 0x3811, 0x04}, /* h_win offset low */
	/* {OV8858_8BIT, 0x3812, 0x00}, v_win offset high */
	{OV8858_8BIT, 0x3813, 0x02}, /* v_win offset low */
	{OV8858_8BIT, 0x3814, 0x01}, /* h_odd_inc */
	{OV8858_8BIT, 0x3815, 0x01}, /* h_even_inc */
	{OV8858_8BIT, 0x3820, 0x00}, /* format1 */
	{OV8858_8BIT, 0x3821, 0x46}, /* format2 */
	{OV8858_8BIT, 0x382A, 0x01}, /* v_odd_inc */
	{OV8858_8BIT, 0x382B, 0x01}, /* v_even_inc */

	{OV8858_8BIT, 0x3830, 0x06},
	{OV8858_8BIT, 0x3836, 0x01},
	{OV8858_8BIT, 0x3837, 0x18},
	{OV8858_8BIT, 0x3841, 0xFF}, /* AUTO_SIZE_CTRL */
	{OV8858_8BIT, 0x3846, 0x48},

	{OV8858_8BIT, 0x3D85, 0x16}, /* OTP_REG85 */
	{OV8858_8BIT, 0x3D8C, 0x73},
	{OV8858_8BIT, 0x3D8D, 0xDE},
	{OV8858_8BIT, 0x3F08, 0x10}, /* PSRAM control register */

	{OV8858_8BIT, 0x4000, 0xF1}, /* BLC CTRL00 = default */
	{OV8858_8BIT, 0x4001, 0x00}, /* BLC CTRL01 */
	{OV8858_8BIT, 0x4002, 0x27}, /* BLC offset = 0x27 */
	{OV8858_8BIT, 0x4005, 0x10}, /* BLC target = 0x0010 */
	{OV8858_8BIT, 0x4009, 0x81}, /* BLC CTRL09 */
	{OV8858_8BIT, 0x400B, 0x0C}, /* BLC CTRL0B = default */
	{OV8858_8BIT, 0x401B, 0x00}, /* Zero line R coeff. = 0x0000 */
	{OV8858_8BIT, 0x401D, 0x00}, /* Zero line T coeff. = 0x0000 */
	{OV8858_8BIT, 0x401F, 0x00}, /* BLC CTRL1F */
	{OV8858_8BIT, 0x4011, 0x20},
	{OV8858_8BIT, 0x4020, 0x00}, /* Anchor left start = 0x0004 */
	{OV8858_8BIT, 0x4021, 0x04}, /* Anchor left start = 0x0004 */
	{OV8858_8BIT, 0x4022, 0x0C}, /* Anchor left end = 0x0BC3 */
	{OV8858_8BIT, 0x4023, 0x60}, /* Anchor left end = 0x0BC3 */
	{OV8858_8BIT, 0x4024, 0x0f}, /* Anchor right start = 0x0C36 */
	{OV8858_8BIT, 0x4025, 0x36}, /* Anchor right start = 0x0C36 */
	{OV8858_8BIT, 0x4026, 0x0F}, /* Anchor right end = 0x0C37 */
	{OV8858_8BIT, 0x4027, 0x37}, /* Anchor right end = 0x0C37 */
	{OV8858_8BIT, 0x4028, 0x00}, /* Top zero line start = 0 */
	{OV8858_8BIT, 0x4029, 0x02}, /* Top zero line number = 2 */
	{OV8858_8BIT, 0x402A, 0x04}, /* Top black line start = 4 */
	{OV8858_8BIT, 0x402B, 0x08}, /* Top black line number = 8 */
	{OV8858_8BIT, 0x402C, 0x00}, /* Bottom zero start line = 2 */
	{OV8858_8BIT, 0x402D, 0x02}, /* Bottom zero line number = 2 */
	{OV8858_8BIT, 0x402E, 0x04}, /* Bottom black line start = 12 */
	{OV8858_8BIT, 0x402F, 0x08}, /* Bottom black line number = 2 */

	{OV8858_8BIT, 0x4034, 0x3F},
	{OV8858_8BIT, 0x403D, 0x04}, /* BLC CTRL3D */
	{OV8858_8BIT, 0x4300, 0xFF}, /* clip_max[11:4] = 0xFFF */
	{OV8858_8BIT, 0x4301, 0x00}, /* clip_min[11:4] = 0 */
	{OV8858_8BIT, 0x4302, 0x0F}, /* clip_min/max[3:0] */
	{OV8858_8BIT, 0x4307, 0x01},
	{OV8858_8BIT, 0x4316, 0x00}, /* CTRL16 = default */
	{OV8858_8BIT, 0x4503, 0x18},
	{OV8858_8BIT, 0x4600, 0x01},
	{OV8858_8BIT, 0x4601, 0x97},
	/* wkup_dly = Mark1 wakeup delay/2^10 = 0x25 */
	{OV8858_8BIT, 0x4808, 0x25},
	{OV8858_8BIT, 0x4816, 0x52}, /* Embedded data type*/
	{OV8858_8BIT, 0x481F, 0x32}, /* clk_prepare_min = 0x32 */
	{OV8858_8BIT, 0x4825, 0x3A}, /* lpx_p_min = 0x3A */
	{OV8858_8BIT, 0x4826, 0x40}, /* hs_prepare_min = 0x40 */
	{OV8858_8BIT, 0x4837, 0x15}, /* pclk_period = 0x14 */
	{OV8858_8BIT, 0x4850, 0x10}, /* LANE SEL01 */
	{OV8858_8BIT, 0x4851, 0x32}, /* LANE SEL02 */

	{OV8858_8BIT, 0x4B00, 0x2A},
	{OV8858_8BIT, 0x4B0D, 0x00},
	{OV8858_8BIT, 0x4D00, 0x04}, /* TPM_CTRL_REG */
	{OV8858_8BIT, 0x4D01, 0x18}, /* TPM_CTRL_REG */
	{OV8858_8BIT, 0x4D02, 0xC3}, /* TPM_CTRL_REG */
	{OV8858_8BIT, 0x4D03, 0xFF}, /* TPM_CTRL_REG */
	{OV8858_8BIT, 0x4D04, 0xFF}, /* TPM_CTRL_REG */
	{OV8858_8BIT, 0x4D05, 0xFF}, /* TPM_CTRL_REG */

	/*
	 * Lens correction (LENC) function enable = 0
	 * Slave sensor AWB Gain function enable = 1
	 * Slave sensor AWB Statistics function enable = 1
	 * Master sensor AWB Gain function enable = 1
	 * Master sensor AWB Statistics function enable = 1
	 * Black DPC function enable = 1
	 * White DPC function enable =1
	 */
	{OV8858_8BIT, 0x5000, 0x7E},
	{OV8858_8BIT, 0x5001, 0x01}, /* BLC function enable = 1 */
	/*
	 * Horizontal scale function enable = 0
	 * WBMATCH bypass mode = Select slave sensor's gain
	 * WBMATCH function enable = 0
	 * Master MWB gain support RGBC = 0
	 * OTP_DPC function enable = 1
	 * Manual mode of VarioPixel function enable = 0
	 * Manual enable of VarioPixel function enable = 0
	 * Use VSYNC to latch ISP modules's function enable signals = 0
	 */
	{OV8858_8BIT, 0x5002, 0x08},
	/*
	 * Bypass all ISP modules after BLC module = 0
	 * DPC_DBC buffer control enable = 1
	 * WBMATCH VSYNC selection = Select master sensor's VSYNC fall
	 * Select master AWB gain to embed line = AWB gain before manual mode
	 * Enable BLC's input flip_i signal = 0
	 */
	{OV8858_8BIT, 0x5003, 0x20},
	{OV8858_8BIT, 0x5041, 0x1D}, /* ISP CTRL41 - embedded data=on */
	{OV8858_8BIT, 0x5046, 0x12}, /* ISP CTRL46 = default */
	/*
	 * Tail enable = 1
	 * Saturate cross cluster enable = 1
	 * Remove cross cluster enable = 1
	 * Enable to remove connected defect pixels in same channel = 1
	 * Enable to remove connected defect pixels in different channel = 1
	 * Smooth enable, use average G for recovery = 1
	 * Black/white sensor mode enable = 0
	 * Manual mode enable = 0
	 */
	{OV8858_8BIT, 0x5780, 0x3e},
	{OV8858_8BIT, 0x5781, 0x0f},
	{OV8858_8BIT, 0x5782, 0x44},
	{OV8858_8BIT, 0x5783, 0x02},
	{OV8858_8BIT, 0x5784, 0x01}, /* DPC CTRL04 */
	{OV8858_8BIT, 0x5785, 0x00},
	{OV8858_8BIT, 0x5786, 0x00},
	{OV8858_8BIT, 0x5787, 0x04}, /* DPC CTRL07 */
	{OV8858_8BIT, 0x5788, 0x02}, /* DPC CTRL08 */
	{OV8858_8BIT, 0x5789, 0x0f},
	{OV8858_8BIT, 0x578A, 0xfd}, /* DPC CTRL0A */
	{OV8858_8BIT, 0x578B, 0xf5}, /* DPC CTRL0B */
	{OV8858_8BIT, 0x578C, 0xf5}, /* DPC CTRL0C */
	{OV8858_8BIT, 0x578D, 0x03},
	{OV8858_8BIT, 0x578E, 0x08}, /* DPC CTRL0E */
	{OV8858_8BIT, 0x578F, 0x0c}, /* DPC CTRL0F */
	{OV8858_8BIT, 0x5790, 0x08}, /* DPC CTRL10 */
	{OV8858_8BIT, 0x5791, 0x04},
	{OV8858_8BIT, 0x5792, 0x00},
	{OV8858_8BIT, 0x5793, 0x52},
	{OV8858_8BIT, 0x5794, 0xa3},
	{OV8858_8BIT, 0x5870, 0x18},
	{OV8858_8BIT, 0x5871, 0x0D},
	{OV8858_8BIT, 0x586E, 0x10},
	{OV8858_8BIT, 0x586F, 0x08},
	{OV8858_8BIT, 0x58F8, 0x3D},
	{OV8858_8BIT, 0x5901, 0x00}, /* VAP CTRL01 = default */

	{OV8858_8BIT, 0x58F4, 0x32},
	/* WINC CTRL08 = embedded data in 1st line*/
	{OV8858_8BIT, 0x5A08, 0x00},
	{OV8858_8BIT, 0x5B00, 0x02}, /* OTP CTRL00 */
	{OV8858_8BIT, 0x5B01, 0x10}, /* OTP CTRL01 */
	{OV8858_8BIT, 0x5B02, 0x03}, /* OTP CTRL02 */
	{OV8858_8BIT, 0x5B03, 0xCF}, /* OTP CTRL03 */
	{OV8858_8BIT, 0x5B05, 0x6C}, /* OTP CTRL05 = default */
	{OV8858_8BIT, 0x5E00, 0x00}, /* PRE CTRL00 = default */
	{OV8858_8BIT, 0x5E01, 0x41}, /* PRE_CTRL01 = default */

	/* {OV8858_8BIT, 0x3768, 0xCC}, */

	{OV8858_8BIT, 0x470B, 0x28},
	{OV8858_8BIT, 0x4202, 0x00},
	{OV8858_8BIT, 0x400D, 0x10},
	{OV8858_8BIT, 0x4040, 0x07},
	{OV8858_8BIT, 0x403E, 0x08},
	{OV8858_8BIT, 0x4041, 0xc6},
	{OV8858_8BIT, 0x400A, 0x01},
	{OV8858_8BIT, 0x0100, 0x01},
	{OV8858_8BIT, 0x0100, 0x00},

	{OV8858_8BIT, 0x3F0A, 0x00},

	{OV8858_8BIT, 0x4500, 0x58},

	{OV8858_8BIT, 0x382D, 0xFF},

	/* {OV8858_8BIT, 0x0100, 0x01}, */
	{OV8858_TOK_TERM, 0, 0}
};

/*****************************STILL********************************/

static const struct ov8858_reg ov8858_8M[] = {
	{OV8858_8BIT, 0x3501, 0x9A}, /* exposure M */
	{OV8858_8BIT, 0x3502, 0x20}, /* exposure L */
	{OV8858_8BIT, 0x3508, 0x02}, /* gain H */
	{OV8858_8BIT, 0x3800, 0x00}, /* h_crop_start high */
	{OV8858_8BIT, 0x3801, 0x0C}, /* h_crop_start low */
	{OV8858_8BIT, 0x3802, 0x00}, /* v_crop_start high */
	{OV8858_8BIT, 0x3803, 0x0C}, /* v_crop_start low */
	{OV8858_8BIT, 0x3804, 0x0C}, /* h_crop_end high */
	{OV8858_8BIT, 0x3805, 0xD3}, /* h_crop_end low */
	{OV8858_8BIT, 0x3806, 0x09}, /* v_crop_end high */
	{OV8858_8BIT, 0x3807, 0xA3}, /* v_crop_end low */
	{OV8858_8BIT, 0x3808, 0x0C}, /* h_output_size high 3280 x 2448 */
	{OV8858_8BIT, 0x3809, 0xD0}, /* h_output_size low */
	{OV8858_8BIT, 0x380A, 0x09}, /* v_output_size high */
	{OV8858_8BIT, 0x380B, 0xa0}, /* v_output_size low */
	{OV8858_8BIT, 0x380C, 0x07}, /* horizontal timing size high */
	{OV8858_8BIT, 0x380D, 0x94}, /* horizontal timing size low */
	{OV8858_8BIT, 0x380E, 0x0A}, /* vertical timing size high */
	{OV8858_8BIT, 0x380F, 0x0D}, /* vertical timing size low */
	{OV8858_8BIT, 0x3814, 0x01}, /* x odd inc */
	{OV8858_8BIT, 0x3821, 0x46}, /* mirror on, bin off */
	{OV8858_8BIT, 0x382A, 0x01}, /* y odd inc */
	{OV8858_8BIT, 0x3830, 0x06},
	{OV8858_8BIT, 0x3836, 0x01},
	{OV8858_8BIT, 0x3F0A, 0x00},
	{OV8858_8BIT, 0x4001, 0x00}, /* total 256 black column */
	{OV8858_8BIT, 0x4022, 0x0C}, /* Anchor left end = 0x0BC3 */
	{OV8858_8BIT, 0x4023, 0x60}, /* Anchor left end = 0x0BC3 */
	/* {OV8858_8BIT, 0x4024, 0x0F}, Anchor right start = 0x0C36 */
	{OV8858_8BIT, 0x4025, 0x36}, /* Anchor right start = 0x0C36 */
	/* {OV8858_8BIT, 0x4026, 0x0F}, Anchor right end = 0x0C37 */
	{OV8858_8BIT, 0x4027, 0x37}, /* Anchor right end = 0x0C37 */
	{OV8858_8BIT, 0x402B, 0x08}, /* top black line number */
	{OV8858_8BIT, 0x402F, 0x08},
	{OV8858_8BIT, 0x4500, 0x58},
	{OV8858_8BIT, 0x4600, 0x01},
	{OV8858_8BIT, 0x4601, 0x97},
	{OV8858_8BIT, 0x4837, 0x14}, /* pclk_period = 0x14 */
	{OV8858_8BIT, 0x382D, 0xFF},
	{OV8858_8BIT, 0x3841, 0xFF},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_6M[] = {
	{OV8858_8BIT, 0x3501, 0x74},
	{OV8858_8BIT, 0x3502, 0x80},
	{OV8858_8BIT, 0x3808, 0x0C}, /* h_output_size high 3280 x 1852 */
	{OV8858_8BIT, 0x3809, 0xD0}, /* h_output_size low */
	{OV8858_8BIT, 0x380A, 0x07}, /* v_output_size high */
	{OV8858_8BIT, 0x380B, 0x3C}, /* v_output_size low */
	{OV8858_8BIT, 0x380C, 0x07}, /* horizontal timing size high */
	{OV8858_8BIT, 0x380D, 0x94}, /* horizontal timing size low */
	{OV8858_8BIT, 0x380E, 0x0A}, /* vertical timing size high */
	{OV8858_8BIT, 0x380F, 0x0D}, /* vertical timing size low */
	{OV8858_8BIT, 0x3810, 0x00},
	{OV8858_8BIT, 0x3811, 0x04},
	{OV8858_8BIT, 0x3813, 0x02},
	{OV8858_8BIT, 0x3814, 0x01},
	{OV8858_8BIT, 0x3815, 0x01},
	{OV8858_8BIT, 0x3820, 0x00},
	{OV8858_8BIT, 0x3821, 0x46},
	{OV8858_8BIT, 0x382A, 0x01},
	{OV8858_8BIT, 0x382B, 0x01},
	{OV8858_8BIT, 0x3830, 0x06},
	{OV8858_8BIT, 0x3836, 0x01},
	{OV8858_8BIT, 0x3837, 0x18},
	{OV8858_8BIT, 0x3841, 0xFF},
	{OV8858_8BIT, 0x3846, 0x48},
	{OV8858_8BIT, 0x3f08, 0x10},
	{OV8858_8BIT, 0x4020, 0x00},
	{OV8858_8BIT, 0x4021, 0x04},
	{OV8858_8BIT, 0x4022, 0x0C}, /* Anchor left end = 0x0BC3 */
	{OV8858_8BIT, 0x4023, 0x60}, /* Anchor left end = 0x0BC3 */
	{OV8858_8BIT, 0x4024, 0x0F}, /* Anchor right start = 0x0C36 */
	{OV8858_8BIT, 0x4025, 0x36}, /* Anchor right start = 0x0C36 */
	{OV8858_8BIT, 0x4026, 0x0F}, /* Anchor right end = 0x0C37 */
	{OV8858_8BIT, 0x4027, 0x37}, /* Anchor right end = 0x0C37 */
	{OV8858_8BIT, 0x4028, 0x00},
	{OV8858_8BIT, 0x4029, 0x02},
	{OV8858_8BIT, 0x402A, 0x04},
	{OV8858_8BIT, 0x402B, 0x08},
	{OV8858_8BIT, 0x402C, 0x00},
	{OV8858_8BIT, 0x402D, 0x02},
	{OV8858_8BIT, 0x402E, 0x04},
	{OV8858_8BIT, 0x402F, 0x08},
	{OV8858_8BIT, 0x4503, 0x18},
	{OV8858_8BIT, 0x4600, 0x00},
	{OV8858_8BIT, 0x4601, 0xF1},
	{OV8858_8BIT, 0x4837, 0x13}, /* pclk_period = 0x14 or 0x16*/
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_3216x1816[] = {
	{OV8858_8BIT, 0x3501, 0x9A}, /* exposure M */
	{OV8858_8BIT, 0x3502, 0x20}, /* exposure L */
	{OV8858_8BIT, 0x3508, 0x02}, /* gain H */
	{OV8858_8BIT, 0x3800, 0x00}, /* h_crop_start high */
	{OV8858_8BIT, 0x3801, 0x0C}, /* h_crop_start low */
	{OV8858_8BIT, 0x3802, 0x00}, /* v_crop_start high */
	{OV8858_8BIT, 0x3803, 0x0C}, /* v_crop_start low */
	{OV8858_8BIT, 0x3804, 0x0C}, /* h_crop_end high */
	{OV8858_8BIT, 0x3805, 0xD3}, /* h_crop_end low */
	{OV8858_8BIT, 0x3806, 0x09}, /* v_crop_end high */
	{OV8858_8BIT, 0x3807, 0xA3}, /* v_crop_end low */
	{OV8858_8BIT, 0x3808, 0x0C}, /* h_output_size high 3216 x 1816 */
	{OV8858_8BIT, 0x3809, 0x90}, /* h_output_size low */
	{OV8858_8BIT, 0x380A, 0x07}, /* v_output_size high */
	{OV8858_8BIT, 0x380B, 0x18}, /* v_output_size low */
	{OV8858_8BIT, 0x380C, 0x07}, /* horizontal timing size high */
	{OV8858_8BIT, 0x380D, 0x94}, /* horizontal timing size low */
	{OV8858_8BIT, 0x380E, 0x0A}, /* vertical timing size high */
	{OV8858_8BIT, 0x380F, 0x0D}, /* vertical timing size low */
	{OV8858_8BIT, 0x3814, 0x01}, /* x odd inc */
	{OV8858_8BIT, 0x3821, 0x46}, /* mirror on, bin off */
	{OV8858_8BIT, 0x382A, 0x01}, /* y odd inc */
	{OV8858_8BIT, 0x3830, 0x06},
	{OV8858_8BIT, 0x3836, 0x01},
	{OV8858_8BIT, 0x3F0A, 0x00},
	{OV8858_8BIT, 0x4001, 0x00}, /*  total 256 black column */
	{OV8858_8BIT, 0x4022, 0x0C}, /* Anchor left end = 0x0BC3 */
	{OV8858_8BIT, 0x4023, 0x30}, /* Anchor left end = 0x0BC3 */
	/* {OV8858_8BIT, 0x4024, 0x0F}, Anchor right start = 0x0C36 */
	{OV8858_8BIT, 0x4025, 0x36}, /* Anchor right start = 0x0C36 */
	/* {OV8858_8BIT, 0x4026, 0x0F}, Anchor right end = 0x0C37  */
	{OV8858_8BIT, 0x4027, 0x37}, /* Anchor right end = 0x0C37 */
	{OV8858_8BIT, 0x402B, 0x08}, /*  top black line number */
	{OV8858_8BIT, 0x402F, 0x08},
	{OV8858_8BIT, 0x4500, 0x58},
	{OV8858_8BIT, 0x4600, 0x00}, /* BLC */
	{OV8858_8BIT, 0x4601, 0xc9},
	{OV8858_8BIT, 0x4837, 0x14}, /* pclk_period = 0x14 */
	{OV8858_8BIT, 0x382D, 0xFF},
	{OV8858_8BIT, 0x3841, 0xFF},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_1632_1224_30fps[] = {
	{OV8858_8BIT, 0x3501, 0x4d}, /* exposure M */
	{OV8858_8BIT, 0x3502, 0x40}, /* exposure L */
	{OV8858_8BIT, 0x3508, 0x04}, /* gain H */
	{OV8858_8BIT, 0x3808, 0x06}, /* x output size H */
	{OV8858_8BIT, 0x3809, 0x60}, /* x output size L */
	{OV8858_8BIT, 0x380a, 0x04}, /* y output size H */
	{OV8858_8BIT, 0x380b, 0xc8}, /* y output size L */
	{OV8858_8BIT, 0x380c, 0x07}, /* HTS H */
	{OV8858_8BIT, 0x380d, 0x94}, /* 88 ; HTS L */
	{OV8858_8BIT, 0x380e, 0x0a}, /* 09;04 ; VTS H */
	{OV8858_8BIT, 0x380f, 0x0d}, /* aa;dc ; VTS L */
	{OV8858_8BIT, 0x3814, 0x03}, /* x odd inc */
	{OV8858_8BIT, 0x3821, 0x67}, /* mirror on, bin on */
	{OV8858_8BIT, 0x382a, 0x03}, /* y odd inc */
	{OV8858_8BIT, 0x3830, 0x08},
	{OV8858_8BIT, 0x3836, 0x02},
	{OV8858_8BIT, 0x3f0a, 0x00},
	{OV8858_8BIT, 0x4001, 0x10}, /* total 128 black column */
	{OV8858_8BIT, 0x4022, 0x06}, /* Anchor left end H */
	{OV8858_8BIT, 0x4023, 0x00}, /* Anchor left end L */
	{OV8858_8BIT, 0x4025, 0x2a}, /* Anchor right start L */
	{OV8858_8BIT, 0x4027, 0x2b}, /* Anchor right end L */
	{OV8858_8BIT, 0x402b, 0x04}, /* top black line number */
	{OV8858_8BIT, 0x402f, 0x04},
	{OV8858_8BIT, 0x4500, 0x58},
	{OV8858_8BIT, 0x4600, 0x00},
	{OV8858_8BIT, 0x4601, 0xcb},
	{OV8858_8BIT, 0x382d, 0x7f},
	{OV8858_8BIT, 0x3841, 0xff},
	{OV8858_8BIT, 0x4837, 0x13},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_1080P_60[] = {
	{OV8858_8BIT, 0x3778, 0x17},
	{OV8858_8BIT, 0x3800, 0x02}, /* h_crop_start high */
	{OV8858_8BIT, 0x3801, 0x26}, /* h_crop_start low */
	{OV8858_8BIT, 0x3802, 0x02}, /* v_crop_start high */
	{OV8858_8BIT, 0x3803, 0x8C}, /* v_crop_start low */
	{OV8858_8BIT, 0x3804, 0x0A}, /* h_crop_end high */
	{OV8858_8BIT, 0x3805, 0x9D}, /* h_crop_end low */
	{OV8858_8BIT, 0x3806, 0x07}, /* v_crop_end high */
	{OV8858_8BIT, 0x3807, 0x0A}, /* v_crop_end low */
	{OV8858_8BIT, 0x3808, 0x07}, /* h_output_size high*/
	{OV8858_8BIT, 0x3809, 0x90}, /* h_output_size low */
	{OV8858_8BIT, 0x380A, 0x04}, /* v_output_size high */
	{OV8858_8BIT, 0x380B, 0x48}, /* v_output_size low */
	{OV8858_8BIT, 0x380C, 0x07}, /* horizontal timing size high */
	{OV8858_8BIT, 0x380D, 0x94}, /* horizontal timing size low */
	{OV8858_8BIT, 0x380E, 0x04}, /* vertical timing size high */
	{OV8858_8BIT, 0x380F, 0xEC}, /* vertical timing size low */
	{OV8858_8BIT, 0x4022, 0x07}, /* Anchor left end = 0x072D */
	{OV8858_8BIT, 0x4023, 0x20}, /* Anchor left end = 0x072D */
	{OV8858_8BIT, 0x4024, 0x0f}, /* Anchor right start = 0x079E */
	{OV8858_8BIT, 0x4025, 0x36}, /* Anchor right start = 0x079E */
	{OV8858_8BIT, 0x4026, 0x0f}, /* Anchor right end = 0x079F */
	{OV8858_8BIT, 0x4027, 0x37}, /* Anchor right end = 0x079F */
	{OV8858_8BIT, 0x4600, 0x00},
	{OV8858_8BIT, 0x4601, 0xef},
	{OV8858_8BIT, 0x4837, 0x16}, /* pclk_period = 0x16 */
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_1080P_30[] = {
	{OV8858_8BIT, 0x3500, 0x00},
	{OV8858_8BIT, 0x3501, 0x44},
	{OV8858_8BIT, 0x3502, 0xA0},
	{OV8858_8BIT, 0x3808, 0x07}, /* h_output_size high*/
	{OV8858_8BIT, 0x3809, 0x90}, /* h_output_size low */
	{OV8858_8BIT, 0x380A, 0x04}, /* v_output_size high */
	{OV8858_8BIT, 0x380B, 0x48}, /* v_output_size low */
	{OV8858_8BIT, 0x380C, 0x07}, /* horizontal timing size high */
	{OV8858_8BIT, 0x380D, 0x94}, /* horizontal timing size low */
	{OV8858_8BIT, 0x380E, 0x0A}, /* vertical timing size high */
	{OV8858_8BIT, 0x380F, 0x0D}, /* vertical timing size low */
	{OV8858_8BIT, 0x3810, 0x00},
	{OV8858_8BIT, 0x3811, 0x04},
	{OV8858_8BIT, 0x3813, 0x02},
	{OV8858_8BIT, 0x3814, 0x01},
	{OV8858_8BIT, 0x3815, 0x01},
	{OV8858_8BIT, 0x3820, 0x00},
	{OV8858_8BIT, 0x3821, 0x46},
	{OV8858_8BIT, 0x382A, 0x01},
	{OV8858_8BIT, 0x382B, 0x01},
	{OV8858_8BIT, 0x3830, 0x06},
	{OV8858_8BIT, 0x3836, 0x01},
	{OV8858_8BIT, 0x3837, 0x18},
	{OV8858_8BIT, 0x3841, 0xFF},
	{OV8858_8BIT, 0x3846, 0x48},
	{OV8858_8BIT, 0x3f08, 0x10},
	{OV8858_8BIT, 0x4020, 0x00},
	{OV8858_8BIT, 0x4021, 0x04},
	{OV8858_8BIT, 0x4022, 0x07}, /* Anchor left end = 0x072D */
	{OV8858_8BIT, 0x4023, 0x20}, /* Anchor left end = 0x072D */
	{OV8858_8BIT, 0x4024, 0x0F}, /* Anchor right start = 0x079E */
	{OV8858_8BIT, 0x4025, 0x36}, /* Anchor right start = 0x079E */
	{OV8858_8BIT, 0x4026, 0x0F}, /* Anchor right end = 0x079F */
	{OV8858_8BIT, 0x4027, 0x37}, /* Anchor right end = 0x079F */
	{OV8858_8BIT, 0x4028, 0x00},
	{OV8858_8BIT, 0x4029, 0x02},
	{OV8858_8BIT, 0x402A, 0x04},
	{OV8858_8BIT, 0x402B, 0x08},
	{OV8858_8BIT, 0x402C, 0x00},
	{OV8858_8BIT, 0x402D, 0x02},
	{OV8858_8BIT, 0x402E, 0x04},
	{OV8858_8BIT, 0x4503, 0x18},
	{OV8858_8BIT, 0x4600, 0x00},
	{OV8858_8BIT, 0x4601, 0xEF},
	{OV8858_8BIT, 0x4837, 0x16}, /* pclk_period = 0x16 */
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_1640_926_30[] = {
	{OV8858_8BIT, 0x3501,  0x4d}, /* exposure M */
	{OV8858_8BIT, 0x3502,  0x40}, /* exposure L */
	{OV8858_8BIT, 0x3508,  0x04}, /* gain H */
	{OV8858_8BIT, 0x3808,  0x06}, /* x output size H */
	{OV8858_8BIT, 0x3809,  0x68}, /* 60 ; x output size L */
	{OV8858_8BIT, 0x380a,  0x03}, /* 04 ; y output size H */
	{OV8858_8BIT, 0x380b,  0x9e}, /* D0;c8 ; y output size L */
	{OV8858_8BIT, 0x380c,  0x07}, /* HTS H */
	{OV8858_8BIT, 0x380d,  0x94}, /* 88 ; HTS L */
	{OV8858_8BIT, 0x380e,  0x0a}, /* 09;04 ; VTS H */
	{OV8858_8BIT, 0x380f,  0x0d}, /* aa;dc ; VTS L */
	{OV8858_8BIT, 0x3814,  0x03}, /* x odd inc */
	{OV8858_8BIT, 0x3821,  0x67}, /* mirror on, bin on */
	{OV8858_8BIT, 0x382a,  0x03}, /* y odd inc */
	{OV8858_8BIT, 0x3830,  0x08},
	{OV8858_8BIT, 0x3836,  0x02},
	{OV8858_8BIT, 0x3f0a,  0x00},
	{OV8858_8BIT, 0x4001,  0x10}, /* total 128 black column */
	{OV8858_8BIT, 0x4022,  0x06}, /* Anchor left end H */
	{OV8858_8BIT, 0x4023,  0x00}, /* Anchor left end L */
	{OV8858_8BIT, 0x4025,  0x2a}, /* Anchor right start L */
	{OV8858_8BIT, 0x4027,  0x2b}, /* Anchor right end L */
	{OV8858_8BIT, 0x402b,  0x04}, /* top black line number */
	{OV8858_8BIT, 0x402f,  0x04},
	{OV8858_8BIT, 0x4500,  0x58},
	{OV8858_8BIT, 0x4600,  0x00},
	{OV8858_8BIT, 0x4601,  0xcb},
	{OV8858_8BIT, 0x382d,  0x7f},
	{OV8858_8BIT, 0x3841,  0xff},
	{OV8858_8BIT, 0x4837,  0x13},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_1640_1232_30[] = {
	{OV8858_8BIT, 0x3501,  0x4d}, /* exposure M */
	{OV8858_8BIT, 0x3502,  0x40}, /* exposure L */
	{OV8858_8BIT, 0x3508,  0x04}, /* gain H */
	{OV8858_8BIT, 0x3808,  0x06}, /* x output size H */
	{OV8858_8BIT, 0x3809,  0x68}, /* 60 ; x output size L */
	{OV8858_8BIT, 0x380a,  0x04}, /* y output size H */
	{OV8858_8BIT, 0x380b,  0xD0}, /* c8 ; y output size L */
	{OV8858_8BIT, 0x380c,  0x07}, /* HTS H */
	{OV8858_8BIT, 0x380d,  0x94}, /* 88 ; HTS L */
	{OV8858_8BIT, 0x380e,  0x0a}, /* 09;04 ; VTS H */
	{OV8858_8BIT, 0x380f,  0x0d}, /* aa;dc ; VTS L */
	{OV8858_8BIT, 0x3814,  0x03}, /* x odd inc */
	{OV8858_8BIT, 0x3821,  0x67}, /* mirror on, bin on */
	{OV8858_8BIT, 0x382a,  0x03}, /* y odd inc */
	{OV8858_8BIT, 0x3830,  0x08},
	{OV8858_8BIT, 0x3836,  0x02},
	{OV8858_8BIT, 0x3f0a,  0x00},
	{OV8858_8BIT, 0x4001,  0x10}, /* total 128 black column */
	{OV8858_8BIT, 0x4022,  0x06}, /* Anchor left end H */
	{OV8858_8BIT, 0x4023,  0x00}, /* Anchor left end L */
	{OV8858_8BIT, 0x4025,  0x2a}, /* Anchor right start L */
	{OV8858_8BIT, 0x4027,  0x2b}, /* Anchor right end L */
	{OV8858_8BIT, 0x402b,  0x04}, /* top black line number */
	{OV8858_8BIT, 0x402f,  0x04},
	{OV8858_8BIT, 0x4500,  0x58},
	{OV8858_8BIT, 0x4600,  0x00},
	{OV8858_8BIT, 0x4601,  0xcb},
	{OV8858_8BIT, 0x382d,  0x7f},
	{OV8858_8BIT, 0x3841,  0xff},
	{OV8858_8BIT, 0x4837,  0x13},
	{OV8858_TOK_TERM, 0, 0}
};

static const struct ov8858_reg ov8858_1640_1096_30[] = {
	{OV8858_8BIT, 0x3501,  0x4d}, /* exposure M */
	{OV8858_8BIT, 0x3502,  0x40}, /* exposure L */
	{OV8858_8BIT, 0x3508,  0x04}, /* gain H */
	{OV8858_8BIT, 0x3808,  0x06}, /* x output size H */
	{OV8858_8BIT, 0x3809,  0x68}, /* 60 ; x output size L */
	{OV8858_8BIT, 0x380a,  0x04}, /* y output size H */
	{OV8858_8BIT, 0x380b,  0x48}, /* D0;c8 ; y output size L */
	{OV8858_8BIT, 0x380c,  0x07}, /* HTS H */
	{OV8858_8BIT, 0x380d,  0x94}, /* 88 ; HTS L */
	{OV8858_8BIT, 0x380e,  0x0a}, /* 09;04 ; VTS H */
	{OV8858_8BIT, 0x380f,  0x0d}, /* aa;dc ; VTS L */
	{OV8858_8BIT, 0x3814,  0x03}, /* x odd inc */
	{OV8858_8BIT, 0x3821,  0x67}, /* mirror on, bin on */
	{OV8858_8BIT, 0x382a,  0x03}, /* y odd inc */
	{OV8858_8BIT, 0x3830,  0x08},
	{OV8858_8BIT, 0x3836,  0x02},
	{OV8858_8BIT, 0x3f0a,  0x00},
	{OV8858_8BIT, 0x4001,  0x10}, /* total 128 black column */
	{OV8858_8BIT, 0x4022,  0x06}, /* Anchor left end H */
	{OV8858_8BIT, 0x4023,  0x00}, /* Anchor left end L */
	{OV8858_8BIT, 0x4025,  0x2a}, /* Anchor right start L */
	{OV8858_8BIT, 0x4027,  0x2b}, /* Anchor right end L */
	{OV8858_8BIT, 0x402b,  0x04}, /* top black line number */
	{OV8858_8BIT, 0x402f,  0x04},
	{OV8858_8BIT, 0x4500,  0x58},
	{OV8858_8BIT, 0x4600,  0x00},
	{OV8858_8BIT, 0x4601,  0xcb},
	{OV8858_8BIT, 0x382d,  0x7f},
	{OV8858_8BIT, 0x3841,  0xff},
	{OV8858_8BIT, 0x4837,  0x13},
	{OV8858_TOK_TERM, 0, 0}
};

static struct ov8858_resolution ov8858_res_preview[] = {
	 {
		.desc = "ov8858_1640_926_30",
		.width = 1640,
		.height = 926,
		.used = 0,
		.regs = ov8858_1640_926_30,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.skip_frames = 1,
		.fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	 },
	 {
		.desc = "ov8858_1640_1232_30",
		.width = 1640,
		.height = 1232,
		.used = 0,
		.regs = ov8858_1640_1232_30,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.skip_frames = 1,
		.fps_options =  {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_3216x1816_PREVIEW",
		.width = 3216,
		.height = 1816,
		.used = 0,
		.regs = ov8858_3216x1816,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 0,
		.fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_6M_PREVIEW",
		.width = 3280,
		.height = 1852,
		.used = 0,
		.regs = ov8858_6M,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 0,
		.fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_8M_PREVIEW",
		.width = 3280,
		.height = 2464,
		.used = 0,
		.regs = ov8858_8M,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 0,
		.fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
};

static struct ov8858_resolution ov8858_res_still[] = {
		{
		.desc = "ov8858_1640_1232_30",
		.width = 1640,
		.height = 1232,
		.used = 0,
		.regs = ov8858_1640_1232_30,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.skip_frames = 1,
		.fps_options =  {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_3216x1816_STILL",
		.width = 3216,
		.height = 1816,
		.used = 0,
		.regs = ov8858_3216x1816,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 1,
		.fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_8M_STILL",
		.width = 3280,
		.height = 2464,
		.used = 0,
		.regs = ov8858_8M,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 1,
		.fps_options = {
			{
				/* Pixel clock: 149.76MHZ */
				.fps = 10,
				.pixels_per_line = 3880,
				.lines_per_frame = 3859,
			},
			{}
		},
	},
};

static struct ov8858_resolution ov8858_res_video[] = {
	 {
		.desc = "ov8858_1640_926_30",
		.width = 1640,
		.height = 926,
		.used = 0,
		.regs = ov8858_1640_926_30,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.skip_frames = 1,
		.fps_options =  {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_1640_1232_30",
		.width = 1640,
		.height = 1232,
		.used = 0,
		.regs = ov8858_1640_1232_30,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.skip_frames = 1,
		.fps_options =  {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_1640_1096_30",
		.width = 1640,
		.height = 1096,
		.used = 0,
		.regs = ov8858_1640_1096_30,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.skip_frames = 1,
		.fps_options =  {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		 .desc = "ov8858_6M_VIDEO",
		 .width = 3280,
		 .height = 1852,
		 .used = 0,
		 .regs = ov8858_6M,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 1,
		 .fps_options =  {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
	{
		.desc = "ov8858_8M_VIDEO",
		.width = 3280,
		.height = 2464,
		.used = 0,
		.regs = ov8858_8M,
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.skip_frames = 1,
		.fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 3880,
				.lines_per_frame = 2573,
			},
			{}
		},
	},
};

#endif /* __OV8858_H__ */
