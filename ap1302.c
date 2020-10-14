// SPDX-License-Identifier: GPL-2.0-only
/*
 * ap1302.c - driver for AP1302 mezzanine
 *
 * Copyright (C) 2020, Witekio, Inc.
 *
 * This driver can only provide limited feature on AP1302.
 * Still need enhancement
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/media-entity.h>
#include <media/v4l2-device.h>

#define DRIVER_NAME "ap1302"

#define AP1302_FW_WINDOW_SIZE			0x2000
#define AP1302_FW_WINDOW_OFFSET			0x8000

#define AP1302_REG_16BIT(n)			((2 << 16) | (n))
#define AP1302_REG_32BIT(n)			((4 << 16) | (n))
#define AP1302_REG_SIZE(n)			((n) >> 16)
#define AP1302_REG_ADDR(n)			((n) & 0xffff)

/* Info Registers */
#define AP1302_CHIP_VERSION			AP1302_REG_16BIT(0x0000)
#define AP1302_CHIP_ID				0x0265
#define AP1302_CHIP_REV				AP1302_REG_16BIT(0x0050)

/* Control Registers */
#define AP1302_BUBBLE_OUT_FMT			AP1302_REG_16BIT(0x1164)
#define AP1302_BUBBLE_OUT_FMT_FT_YUV		(3U << 4)
#define AP1302_BUBBLE_OUT_FMT_FT_RGB		(4U << 4)
#define AP1302_BUBBLE_OUT_FMT_FT_YUV_JFIF	(5U << 4)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_888	(0U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_565	(1U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_555M	(2U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_555L	(3U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_422	(0U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_420	(1U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_400	(2U << 0)
#define AP1302_ATOMIC				AP1302_REG_16BIT(0x1184)
#define AP1302_ATOMIC_MODE			BIT(2)
#define AP1302_ATOMIC_FINISH			BIT(1)
#define AP1302_ATOMIC_RECORD			BIT(0)

/*
 * Context Registers (Preview, Snapshot, Video). The addresses correspond to
 * the preview context.
 */
#define AP1302_CTX_OFFSET			0x1000

#define AP1302_CTX_WIDTH			AP1302_REG_16BIT(0x2000)
#define AP1302_CTX_HEIGHT			AP1302_REG_16BIT(0x2002)
#define AP1302_CTX_ROI_X0			AP1302_REG_16BIT(0x2004)
#define AP1302_CTX_ROI_Y0			AP1302_REG_16BIT(0x2006)
#define AP1302_CTX_ROI_X1			AP1302_REG_16BIT(0x2008)
#define AP1302_CTX_ROI_Y1			AP1302_REG_16BIT(0x200a)
#define AP1302_CTX_OUT_FMT			AP1302_REG_16BIT(0x2012)
#define AP1302_CTX_OUT_FMT_IPIPE_BYPASS		BIT(13)
#define AP1302_CTX_OUT_FMT_SS			BIT(12)
#define AP1302_CTX_OUT_FMT_FAKE_EN		BIT(11)
#define AP1302_CTX_OUT_FMT_ST_EN		BIT(10)
#define AP1302_CTX_OUT_FMT_IIS_NONE		(0U << 8)
#define AP1302_CTX_OUT_FMT_IIS_POST_VIEW	(1U << 8)
#define AP1302_CTX_OUT_FMT_IIS_VIDEO		(2U << 8)
#define AP1302_CTX_OUT_FMT_IIS_BUBBLE		(3U << 8)
#define AP1302_CTX_OUT_FMT_FT_JPEG_422		(0U << 4)
#define AP1302_CTX_OUT_FMT_FT_JPEG_420		(1U << 4)
#define AP1302_CTX_OUT_FMT_FT_YUV		(3U << 4)
#define AP1302_CTX_OUT_FMT_FT_RGB		(4U << 4)
#define AP1302_CTX_OUT_FMT_FT_YUV_JFIF		(5U << 4)
#define AP1302_CTX_OUT_FMT_FT_RAW8		(8U << 4)
#define AP1302_CTX_OUT_FMT_FT_RAW10		(9U << 4)
#define AP1302_CTX_OUT_FMT_FT_RAW12		(10U << 4)
#define AP1302_CTX_OUT_FMT_FT_RAW16		(11U << 4)
#define AP1302_CTX_OUT_FMT_FT_DNG8		(12U << 4)
#define AP1302_CTX_OUT_FMT_FT_DNG10		(13U << 4)
#define AP1302_CTX_OUT_FMT_FT_DNG12		(14U << 4)
#define AP1302_CTX_OUT_FMT_FT_DNG16		(15U << 4)
#define AP1302_CTX_OUT_FMT_FST_JPEG_ROTATE	BIT(2)
#define AP1302_CTX_OUT_FMT_FST_JPEG_SCAN	(0U << 0)
#define AP1302_CTX_OUT_FMT_FST_JPEG_JFIF	(1U << 0)
#define AP1302_CTX_OUT_FMT_FST_JPEG_EXIF	(2U << 0)
#define AP1302_CTX_OUT_FMT_FST_RGB_888		(0U << 0)
#define AP1302_CTX_OUT_FMT_FST_RGB_565		(1U << 0)
#define AP1302_CTX_OUT_FMT_FST_RGB_555M		(2U << 0)
#define AP1302_CTX_OUT_FMT_FST_RGB_555L		(3U << 0)
#define AP1302_CTX_OUT_FMT_FST_YUV_422		(0U << 0)
#define AP1302_CTX_OUT_FMT_FST_YUV_420		(1U << 0)
#define AP1302_CTX_OUT_FMT_FST_YUV_400		(2U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_SENSOR	(0U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_CAPTURE	(1U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_CP		(2U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_BPC		(3U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_IHDR		(4U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_PP		(5U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_DENSH	(6U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_PM		(7U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_GC		(8U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_CURVE	(9U << 0)
#define AP1302_CTX_OUT_FMT_FST_RAW_CCONV	(10U << 0)
#define AP1302_CTX_S1_SENSOR_MODE		AP1302_REG_16BIT(0x202e)
#define AP1302_CTX_HINF_CTRL			AP1302_REG_16BIT(0x2030)
#define AP1302_CTX_HINF_CTRL_BT656_LE		BIT(15)
#define AP1302_CTX_HINF_CTRL_BT656_16BIT	BIT(14)
#define AP1302_CTX_HINF_CTRL_MUX_DELAY(n)	((n) << 8)
#define AP1302_CTX_HINF_CTRL_LV_POL		BIT(7)
#define AP1302_CTX_HINF_CTRL_FV_POL		BIT(6)
#define AP1302_CTX_HINF_CTRL_MIPI_CONT_CLK	BIT(5)
#define AP1302_CTX_HINF_CTRL_SPOOF		BIT(4)
#define AP1302_CTX_HINF_CTRL_MIPI_MODE		BIT(3)
#define AP1302_CTX_HINF_CTRL_MIPI_LANES(n)	((n) << 0)

/* System Registers */
#define AP1302_BOOTDATA_STAGE			AP1302_REG_16BIT(0x6002)
#define AP1302_SENSOR_SELECT			AP1302_REG_16BIT(0x600c)
#define AP1302_SENSOR_SELECT_TP_MODE(n)		((n) << 8)
#define AP1302_SENSOR_SELECT_PATTERN_ON		BIT(7)
#define AP1302_SENSOR_SELECT_MODE_3D_ON		BIT(6)
#define AP1302_SENSOR_SELECT_CLOCK		BIT(5)
#define AP1302_SENSOR_SELECT_SINF_MIPI		BIT(4)
#define AP1302_SENSOR_SELECT_YUV		BIT(2)
#define AP1302_SENSOR_SELECT_SENSOR_TP		(0U << 0)
#define AP1302_SENSOR_SELECT_SENSOR(n)		(((n) + 1) << 0)
#define AP1302_SYS_START			AP1302_REG_16BIT(0x601a)
#define AP1302_SYS_START_PLL_LOCK		BIT(15)
#define AP1302_SYS_START_LOAD_OTP		BIT(12)
#define AP1302_SYS_START_RESTART_ERROR		BIT(11)
#define AP1302_SYS_START_STALL_STATUS		BIT(9)
#define AP1302_SYS_START_STALL_EN		BIT(8)
#define AP1302_SYS_START_STALL_MODE_FRAME	(0U << 6)
#define AP1302_SYS_START_STALL_MODE_DISABLED	(1U << 6)
#define AP1302_SYS_START_STALL_MODE_POWER_DOWN	(2U << 6)
#define AP1302_SYS_START_GO			BIT(4)
#define AP1302_SYS_START_PATCH_FUN		BIT(1)
#define AP1302_SYS_START_PLL_INIT		BIT(0)

/* Misc Registers */
#define AP1302_SIP_CRC				AP1302_REG_16BIT(0xf052)

enum ap1302_context {
	AP1302_CTX_PREVIEW = 0,
	AP1302_CTX_SNAPSHOT = 1,
	AP1302_CTX_VIDEO = 2,
};

struct ap1302_format_info {
	unsigned int code;
	u16 out_fmt;
};

struct ap1302_resolution {
	unsigned int width;
	unsigned int height;
};

struct ap1302_sensor_info {
	const char *compatible;
	const char *name;
	const struct ap1302_resolution *resolutions;
	const char * const *supplies;
};

struct ap1302_sensor {
	const struct ap1302_sensor_info *info;
	unsigned int num_supplies;
	struct regulator_bulk_data *supplies;
};

struct ap1302_device {
	struct device *dev;
	struct i2c_client *client;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *standby_gpio;
	struct clk *clock;
	struct regmap *regmap16;
	struct regmap *regmap32;

	const struct firmware *fw;

	struct mutex lock;	/* Protects formats */

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct {
		struct v4l2_mbus_framefmt format;
		const struct ap1302_format_info *info;
	} formats[1];

	struct ap1302_sensor sensors[2];
};

static inline struct ap1302_device *to_ap1302(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ap1302_device, sd);
}

struct ap1302_firmware_header {
	u16 pll_init_size;
	u16 crc;
} __packed;

#define MAX_FW_LOAD_RETRIES 3

static const struct ap1302_format_info supported_video_formats[] = {
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.out_fmt = AP1302_CTX_OUT_FMT_FT_YUV_JFIF
			 | AP1302_CTX_OUT_FMT_FST_YUV_422,
	}, {
		.code = MEDIA_BUS_FMT_UYYVYY8_0_5X24,
		.out_fmt = AP1302_CTX_OUT_FMT_FT_YUV_JFIF
			 | AP1302_CTX_OUT_FMT_FST_YUV_420,
	},
};

/* -----------------------------------------------------------------------------
 * Sensor Info
 */

static const struct ap1302_sensor_info ap1302_sensor_info[] = {
	{
		.compatible = "onnn,ar0144",
		.name = "ar0114",
		.resolutions = (const struct ap1302_resolution[]) {
			{ 2560, 800 },
			{ },
		},
	}, {
		.compatible = "onnn,ar0330",
		.name = "ar0330",
		.resolutions = (const struct ap1302_resolution[]) {
			{ 2304, 1536 },
			{ },
		},
		.supplies = (const char * const[]) {
			"vddpll",
			"vaa",
			"vddio",
			NULL,
		},
	}, {
		.compatible = "onnn,ar1335",
		.name = "ar1335",
		.resolutions = (const struct ap1302_resolution[]) {
			{ 1920, 1080 },
			{ },
		},
	},
};

static const struct ap1302_sensor_info ap1302_sensor_info_none = {
	.compatible = "",
	.name = "none",
	.resolutions = (const struct ap1302_resolution[]) {
		{ },
	},
};

/* -----------------------------------------------------------------------------
 * Register Configuration
 */

static const struct regmap_config ap1302_reg16_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 2,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config ap1302_reg32_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

static int ap1302_read(struct ap1302_device *ap1302, u32 reg, u32 *val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u16 addr = AP1302_REG_ADDR(reg);
	int ret;

	switch (size) {
	case 2:
		ret = regmap_read(ap1302->regmap16, addr, val);
		break;
	case 4:
		ret = regmap_read(ap1302->regmap32, addr, val);
		break;
	default:
		return -EINVAL;
	}

	if (ret) {
		dev_err(ap1302->dev, "%s: register 0x%04x %s failed: %d\n",
			__func__, addr, "read", ret);
		return ret;
	}

	dev_dbg(ap1302->dev, "%s: R0x%04x = 0x%0*x\n", __func__,
		addr, size * 2, *val);

	return 0;
}

static int ap1302_write(struct ap1302_device *ap1302, u32 reg, u32 val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u16 addr = AP1302_REG_ADDR(reg);
	int ret;

	switch (size) {
	case 2:
		ret = regmap_write(ap1302->regmap16, addr, val);
		break;
	case 4:
		ret = regmap_write(ap1302->regmap32, addr, val);
		break;
	default:
		return -EINVAL;
	}

	if (ret) {
		dev_err(ap1302->dev, "%s: register 0x%04x %s failed: %d\n",
			__func__, addr, "write", ret);
		return ret;
	}

	return 0;
}

static int ap1302_write_ctx(struct ap1302_device *ap1302,
			    enum ap1302_context ctx, u32 reg, u32 val)
{
	/*
	 * The snapshot context is missing the S1_SENSOR_MODE register,
	 * shifting all the addresses for the registers that come after it.
	 */
	if (ctx == AP1302_CTX_SNAPSHOT) {
		if (AP1302_REG_ADDR(reg) >= AP1302_CTX_S1_SENSOR_MODE)
			reg -= 2;
	}

	reg += ctx * AP1302_CTX_OFFSET;

	return ap1302_write(ap1302, reg, val);
}

/* -----------------------------------------------------------------------------
 * Power Handling
 */

static int ap1302_power_on_sensors(struct ap1302_device *ap1302)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		ret = regulator_bulk_enable(sensor->num_supplies,
					    sensor->supplies);
		if (ret) {
			dev_err(ap1302->dev,
				"Failed to enable supplies for sensor %u\n", i);
			goto error;
		}
	}

	return 0;

error:
	for (; i > 0; --i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i - 1];

		regulator_bulk_disable(sensor->num_supplies, sensor->supplies);
	}

	return ret;
}

static void ap1302_power_off_sensors(struct ap1302_device *ap1302)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		regulator_bulk_disable(sensor->num_supplies, sensor->supplies);
	}
}

static int ap1302_power_on(struct ap1302_device *ap1302)
{
	int ret;

	/* 0. RESET was asserted when getting the GPIO. */

	/* 1. Assert STANDBY. */
	if (ap1302->standby_gpio) {
		gpiod_set_value(ap1302->standby_gpio, 1);
		usleep_range(200, 1000);
	}

	/* 2. Power up the regulators. To be implemented. */

	/* 3. De-assert STANDBY. */
	if (ap1302->standby_gpio) {
		gpiod_set_value(ap1302->standby_gpio, 0);
		usleep_range(200, 1000);
	}

	/* 4. Turn the clock on. */
	ret = clk_prepare_enable(ap1302->clock);
	if (ret < 0) {
		dev_err(ap1302->dev, "Failed to enable clock: %d\n", ret);
		return ret;
	}

	/* 5. De-assert RESET. */
	gpiod_set_value(ap1302->reset_gpio, 0);

	/*
	 * 6. Wait for the AP1302 to initialize. The datasheet doesn't specify
	 * how long this takes.
	 */
	usleep_range(10000, 11000);

	return 0;
}

static void ap1302_power_off(struct ap1302_device *ap1302)
{
	/* 1. Assert RESET. */
	gpiod_set_value(ap1302->reset_gpio, 1);

	/* 2. Turn the clock off. */
	clk_disable_unprepare(ap1302->clock);

	/* 3. Assert STANDBY. */
	if (ap1302->standby_gpio) {
		gpiod_set_value(ap1302->standby_gpio, 1);
		usleep_range(200, 1000);
	}

	/* 4. Power down the regulators. To be implemented. */

	/* 5. De-assert STANDBY. */
	if (ap1302->standby_gpio) {
		usleep_range(200, 1000);
		gpiod_set_value(ap1302->standby_gpio, 0);
	}
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

static int ap1302_configure(struct ap1302_device *ap1302)
{
	int ret;

	ret = ap1302_write(ap1302, AP1302_ATOMIC, AP1302_ATOMIC_RECORD);
	if (ret < 0)
		return ret;

	ret = ap1302_write_ctx(ap1302, AP1302_CTX_PREVIEW, AP1302_CTX_WIDTH,
			       ap1302->formats[0].format.width);
	if (ret < 0)
		return ret;

	ret = ap1302_write_ctx(ap1302, AP1302_CTX_PREVIEW, AP1302_CTX_HEIGHT,
			       ap1302->formats[0].format.height);
	if (ret < 0)
		return ret;

	ret = ap1302_write_ctx(ap1302, AP1302_CTX_PREVIEW, AP1302_CTX_OUT_FMT,
			       ap1302->formats[0].info->out_fmt);
	if (ret < 0)
		return ret;

	/*
	ret = ap1302_write(ap1302, AP1302_BUBBLE_OUT_FMT,
			   AP1302_BUBBLE_OUT_FMT_FT_YUV_JFIF |
			   AP1302_BUBBLE_OUT_FMT_FST_YUV_420);
	if (ret < 0)
		return ret;
	*/

	ap1302_write(ap1302, AP1302_ATOMIC,
		     0x0008 | AP1302_ATOMIC_FINISH | AP1302_ATOMIC_RECORD);

	/*
	ret = ap1302_write(ap1302, AP1302_CTX_HINF_CTRL,
			   AP1302_CTX_HINF_CTRL_MIPI_CONT_CLK |
			   AP1302_CTX_HINF_CTRL_SPOOF |
			   AP1302_CTX_HINF_CTRL_MIPI_MODE);
	if (ret < 0)
		return ret;
	*/

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static struct v4l2_mbus_framefmt * ap1302_get_pad_format(struct ap1302_device *ap1302,
						struct v4l2_subdev_pad_config *cfg,
						unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ap1302->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ap1302->formats[pad].format;
	default:
		return NULL;
	}
}

static int ap1302_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(supported_video_formats))
		return -EINVAL;

	code->code = supported_video_formats[code->index].code;
	return 0;
}

static int ap1302_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_resolution *resolutions =
		ap1302->sensors[0].info->resolutions;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
		if (supported_video_formats[i].code == fse->code)
			break;
	}

	if (i >= ARRAY_SIZE(supported_video_formats))
		return-EINVAL;

	for (i = 0; i <= fse->index; ++resolutions, ++i) {
		if (!resolutions->width)
			return -EINVAL;
	}

	fse->min_width = resolutions->width;
	fse->min_height = resolutions->height;
	fse->max_width = resolutions->width;
	fse->max_height = resolutions->height;

	return 0;
}

static int ap1302_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct v4l2_mbus_framefmt *format;

	format = ap1302_get_pad_format(ap1302, cfg, fmt->pad, fmt->which);

	mutex_lock(&ap1302->lock);
	fmt->format = *format;
	mutex_unlock(&ap1302->lock);

	return 0;
}

static void ap1302_res_roundup(struct ap1302_device *ap1302, u32 *width,
			       u32 *height)
{
	const struct ap1302_resolution *resolutions =
		ap1302->sensors[0].info->resolutions;
	unsigned int i;

	/* TODO: Search for best match instead of rounding */
	for (i = 0; resolutions[i].width ; i++) {
		if (resolutions[i].width >= *width &&
		    resolutions[i].height >= *height) {
			*width = resolutions[i].width;
			*height = resolutions[i].height;
			return;
		}
	}

	/* Use default in case of no match */
	*width = resolutions[0].width;
	*height = resolutions[0].height;
}

static int ap1302_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_format_info *info = NULL;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	format = ap1302_get_pad_format(ap1302, cfg, fmt->pad, fmt->which);

	/* Validate the media bus code, default to the first supported value. */
	for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
		if (supported_video_formats[i].code == fmt->format.code) {
			info = &supported_video_formats[i];
			break;
		}
	}

	if (!info)
		info = &supported_video_formats[0];

	/* Find suitable supported resolution. */
	ap1302_res_roundup(ap1302, &fmt->format.width, &fmt->format.height);

	mutex_lock(&ap1302->lock);

	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = info->code;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ap1302->formats[fmt->pad].info = info;

	mutex_unlock(&ap1302->lock);

	fmt->format = *format;

	return 0;
}

static int ap1302_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	u32 reg;
	int ret;

	mutex_lock(&ap1302->lock);

	if (enable) {
		ret = ap1302_configure(ap1302);
		if (ret < 0)
			goto done;

		reg = AP1302_SYS_START_PLL_LOCK
		    | AP1302_SYS_START_GO;
	} else {
		reg = AP1302_SYS_START_PLL_LOCK
		    | AP1302_SYS_START_STALL_EN
		    | AP1302_SYS_START_STALL_MODE_DISABLED;
	}

	ret = ap1302_write(ap1302, AP1302_SYS_START, reg);

done:
	mutex_unlock(&ap1302->lock);

	if (ret < 0)
		dev_err(ap1302->dev, "Failed to %s stream: %d\n",
			enable ? "start" : "stop", ret);

	return ret;
}

static const struct media_entity_operations ap1302_media_ops = {
	.link_validate = v4l2_subdev_link_validate
};

static const struct v4l2_subdev_pad_ops ap1302_pad_ops = {
	.enum_mbus_code = ap1302_enum_mbus_code,
	.enum_frame_size = ap1302_enum_frame_size,
	.get_fmt = ap1302_get_fmt,
	.set_fmt = ap1302_set_fmt,
};

static const struct v4l2_subdev_video_ops ap1302_video_ops = {
	.s_stream = ap1302_s_stream,
};

static const struct v4l2_subdev_ops ap1302_subdev_ops = {
	.pad = &ap1302_pad_ops,
	.video = &ap1302_video_ops,
};

/* -----------------------------------------------------------------------------
 * Boot & Firmware Handling
 */

static int ap1302_request_firmware(struct ap1302_device *ap1302)
{
	const struct ap1302_firmware_header *fw_hdr;
	unsigned int fw_size;
	char name[64];
	int ret;

	ret = snprintf(name, sizeof(name), "ap1302_%s_%s_fw.bin",
		       ap1302->sensors[0].info->name,
		       ap1302->sensors[1].info->name);
	if (ret >= sizeof(name)) {
		dev_err(ap1302->dev, "Firmware name too long\n");
		return -EINVAL;
	}

	ret = request_firmware(&ap1302->fw, name, ap1302->dev);
	if (ret) {
		dev_err(ap1302->dev, "Failed to request firmware: %d\n", ret);
		return ret;
	}

	/*
	 * The firmware binary contains a header defined by the
	 * ap1302_firmware_header structure. The firmware itself (also referred
	 * to as bootdata) follows the header. Perform sanity checks to ensure
	 * the firmware is valid.
	 */
	fw_hdr = (const struct ap1302_firmware_header *)ap1302->fw->data;
	fw_size = ap1302->fw->size - sizeof(*fw_hdr);

	if (fw_hdr->pll_init_size > fw_size) {
		dev_err(ap1302->dev,
			"Invalid firmware: PLL init size too large\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * ap1302_write_fw_window() - Write a piece of firmware to the AP1302
 * @win_pos: Firmware load window current position
 * @buf: Firmware data buffer
 * @len: Firmware data length
 *
 * The firmware is loaded through a window in the registers space. Writes are
 * sequential starting at address 0x8000, and must wrap around when reaching
 * 0x9fff. This function write the firmware data stored in @buf to the AP1302,
 * keeping track of the window position in the @win_pos argument.
 */
static int ap1302_write_fw_window(struct ap1302_device *ap1302, const u8 *buf,
				  u32 len, unsigned int *win_pos)
{
	while (len > 0) {
		unsigned int write_addr;
		unsigned int write_size;
		int ret;

		/*
		 * Write at most len bytes, from the current position to the
		 * end of the window.
		 */
		write_addr = *win_pos + AP1302_FW_WINDOW_OFFSET;
		write_size = min(len, AP1302_FW_WINDOW_SIZE - *win_pos);

		ret = regmap_raw_write(ap1302->regmap16, write_addr, buf,
				       write_size);
		if (ret)
			return ret;

		buf += write_size;
		len -= write_size;

		*win_pos += write_size;
		if (*win_pos >= AP1302_FW_WINDOW_SIZE)
			*win_pos = 0;
	}

	return 0;
}

static int ap1302_load_firmware(struct ap1302_device *ap1302)
{
	const struct ap1302_firmware_header *fw_hdr;
	unsigned int fw_size;
	const u8 *fw_data;
	unsigned int win_pos = 0;
	unsigned int crc;
	int ret;

	fw_hdr = (const struct ap1302_firmware_header *)ap1302->fw->data;
	fw_data = (u8 *)&fw_hdr[1];
	fw_size = ap1302->fw->size - sizeof(*fw_hdr);

	/* Clear the CRC register. */
	ret = ap1302_write(ap1302, AP1302_SIP_CRC, 0xffff);
	if (ret)
		return ret;

	/*
	 * Load the PLL initialization settings, set the bootdata stage to 2 to
	 * apply the basic_init_hp settings, and wait 1ms for the PLL to lock.
	 */
	ret = ap1302_write_fw_window(ap1302, fw_data, fw_hdr->pll_init_size,
				     &win_pos);
	if (ret)
		return ret;

	ret = ap1302_write(ap1302, AP1302_BOOTDATA_STAGE, 0x0002);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	/* Load the rest of the bootdata content and verify the CRC. */
	ret = ap1302_write_fw_window(ap1302, fw_data + fw_hdr->pll_init_size,
				     fw_size - fw_hdr->pll_init_size, &win_pos);
	if (ret)
		return ret;

	msleep(40);

	ret = ap1302_read(ap1302, AP1302_SIP_CRC, &crc);
	if (ret)
		return ret;

	if (crc != fw_hdr->crc) {
		dev_warn(ap1302->dev,
			 "CRC mismatch: expected 0x%04x, got 0x%04x\n",
			 fw_hdr->crc, crc);
		return -EAGAIN;
	}

	/*
	 * Write 0xffff to the bootdata_stage register to indicate to the
	 * AP1302 that the whole bootdata content has been loaded.
	 */
	ret = ap1302_write(ap1302, AP1302_BOOTDATA_STAGE, 0xffff);
	if (ret)
		return ret;

	return 0;
}

static int ap1302_detect_chip(struct ap1302_device *ap1302)
{
	unsigned int version;
	unsigned int revision;
	int ret;

	ret = ap1302_read(ap1302, AP1302_CHIP_VERSION, &version);
	if (ret)
		return ret;

	ret = ap1302_read(ap1302, AP1302_CHIP_REV, &revision);
	if (ret)
		return ret;

	if (version != AP1302_CHIP_ID) {
		dev_err(ap1302->dev,
			"Invalid chip version, expected 0x%04x, got 0x%04x\n",
			AP1302_CHIP_ID, version);
		return -EINVAL;
	}

	dev_info(ap1302->dev, "AP1302 revision %u.%u.%u detected\n",
		 (revision & 0xf000) >> 12, (revision & 0x0f00) >> 8,
		 revision & 0x00ff);

	return 0;
}

static int ap1302_hw_init(struct ap1302_device *ap1302)
{
	unsigned int retries;
	int ret;

	/* Request and validate the firmware. */
	ret = ap1302_request_firmware(ap1302);
	if (ret)
		return ret;

	/*
	 * Power the sensors first, as the firmware will access them once it
	 * gets loaded.
	 */
	ret = ap1302_power_on_sensors(ap1302);
	if (ret < 0)
		goto error_firmware;

	/*
	 * Load the firmware, retrying in case of CRC errors. The AP1302 is
	 * reset with a full power cycle between each attempt.
	 */
	for (retries = 0; retries < MAX_FW_LOAD_RETRIES; ++retries) {
		ret = ap1302_power_on(ap1302);
		if (ret < 0)
			goto error_power_sensors;

		ret = ap1302_detect_chip(ap1302);
		if (ret)
			goto error_power;

		ret = ap1302_load_firmware(ap1302);
		if (!ret)
			break;

		if (ret != -EAGAIN)
			goto error_power;

		ap1302_power_off(ap1302);
	}

	if (retries == MAX_FW_LOAD_RETRIES) {
		dev_err(ap1302->dev,
			"Firmware load retries exceeded, aborting\n");
		ret = -ETIMEDOUT;
		goto error_power;
	}

	return 0;

error_power:
	ap1302_power_off(ap1302);
error_power_sensors:
	ap1302_power_off_sensors(ap1302);
error_firmware:
	release_firmware(ap1302->fw);

	return ret;
}

static void ap1302_hw_cleanup(struct ap1302_device *ap1302)
{
	ap1302_power_off(ap1302);
	ap1302_power_off_sensors(ap1302);
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int ap1302_config_v4l2(struct ap1302_device *ap1302)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev *sd;
	int ret;

	sd = &ap1302->sd;
	sd->dev = ap1302->dev;
	v4l2_i2c_subdev_init(sd, ap1302->client, &ap1302_subdev_ops);

	strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));
	strlcat(sd->name, ".", sizeof(sd->name));
	strlcat(sd->name, dev_name(ap1302->dev), sizeof(sd->name));
	dev_dbg(ap1302->dev, "name %s\n", sd->name);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &ap1302_media_ops;

	ap1302->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, 1, &ap1302->pad);
	if (ret < 0) {
		dev_err(ap1302->dev, "media_entity_init failed %d\n", ret);
		return ret;
	}

	ap1302->formats[0].info = &supported_video_formats[0];

	format = &ap1302->formats[0].format;
	format->width = ap1302->sensors[0].info->resolutions[0].width;
	format->height = ap1302->sensors[0].info->resolutions[0].height;
	format->field = V4L2_FIELD_NONE;
	format->code = ap1302->formats[0].info->code;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(ap1302->dev, "v4l2_async_register_subdev failed %d\n", ret);
		goto err;
	}

	return 0;

err:
	media_entity_cleanup(&sd->entity);

	return ret;
}

static int ap1302_parse_of_sensor(struct ap1302_device *ap1302,
				  struct device_node *node)
{
	struct ap1302_sensor *sensor;
	const char *compat;
	unsigned int i;
	u32 reg;
	int ret;

	/*
	 * Retrieve the sensor index and model from the reg property and
	 * compatible properties.
	 */
	ret = of_property_read_u32(node, "reg", &reg);
	if (ret < 0) {
		dev_warn(ap1302->dev,
			 "'reg' property missing in sensor node\n");
		return -EINVAL;
	}

	if (reg >= ARRAY_SIZE(ap1302->sensors)) {
		dev_warn(ap1302->dev, "Out-of-bounds 'reg' value %u\n",
			 reg);
		return -EINVAL;
	}

	sensor = &ap1302->sensors[reg];

	ret = of_property_read_string(node, "compatible", &compat);
	if (ret < 0)
		return 0;

	for (i = 0; i < ARRAY_SIZE(ap1302_sensor_info); ++i) {
		const struct ap1302_sensor_info *info =
			&ap1302_sensor_info[i];

		if (!strcmp(info->compatible, compat)) {
			sensor->info = info;
			break;
		}
	}

	if (sensor->info == &ap1302_sensor_info_none) {
		dev_warn(ap1302->dev, "Unsupported sensor %s, ignoring\n",
			 compat);
		return -EINVAL;
	}

	/* Retrieve the power supplies for the sensor, if any. */
	if (sensor->info->supplies) {
		for (i = 0; sensor->info->supplies[i]; ++i)
			;

		sensor->num_supplies = i;
		sensor->supplies = devm_kcalloc(ap1302->dev,
						sensor->num_supplies,
						sizeof(*sensor->supplies),
						GFP_KERNEL);
		if (!sensor->supplies)
			return -ENOMEM;

		for (i = 0; i < sensor->num_supplies; ++i)
			sensor->supplies[i].supply = sensor->info->supplies[i];

		ret = devm_regulator_bulk_get(ap1302->dev, sensor->num_supplies,
					      sensor->supplies);
		if (ret < 0) {
			dev_err(ap1302->dev,
				"Failed to get supplies for sensor %u\n", reg);
			return ret;
		}
	}

	return 0;
}

static int ap1302_parse_of(struct ap1302_device *ap1302)
{
	struct device_node *sensors;
	struct device_node *node;
	unsigned int i;

	/* Clock */
	ap1302->clock = devm_clk_get(ap1302->dev, NULL);
	if (IS_ERR(ap1302->clock)) {
		dev_err(ap1302->dev, "Failed to get clock: %ld\n",
			PTR_ERR(ap1302->clock));
		return PTR_ERR(ap1302->clock);
	}

	/* GPIOs */
	ap1302->reset_gpio = devm_gpiod_get(ap1302->dev, "reset",
					    GPIOD_OUT_HIGH);
	if (IS_ERR(ap1302->reset_gpio)) {
		dev_err(ap1302->dev, "Can't get reset GPIO: %ld\n",
			PTR_ERR(ap1302->reset_gpio));
		return PTR_ERR(ap1302->reset_gpio);
	}

	ap1302->standby_gpio = devm_gpiod_get_optional(ap1302->dev, "standby",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ap1302->standby_gpio)) {
		dev_err(ap1302->dev, "Can't get standby GPIO: %ld\n",
			PTR_ERR(ap1302->standby_gpio));
		return PTR_ERR(ap1302->standby_gpio);
	}

	/* Sensors */
	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i)
		ap1302->sensors[i].info = &ap1302_sensor_info_none;

	sensors = of_get_child_by_name(ap1302->dev->of_node, "sensors");
	if (!sensors) {
		dev_err(ap1302->dev, "'sensors' child node not found\n");
		return -EINVAL;
	}

	for_each_child_of_node(sensors, node) {
		if (of_node_name_eq(node, "sensor"))
			ap1302_parse_of_sensor(ap1302, node);
	}

	of_node_put(sensors);

	return 0;
}

static int ap1302_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ap1302_device *ap1302;
	int ret;

	ap1302 = devm_kzalloc(&client->dev, sizeof(*ap1302), GFP_KERNEL);
	if (!ap1302)
		return -ENOMEM;

	ap1302->dev = &client->dev;
	ap1302->client = client;

	mutex_init(&ap1302->lock);

	ap1302->regmap16 = devm_regmap_init_i2c(client, &ap1302_reg16_config);
	if (IS_ERR(ap1302->regmap16)) {
		dev_err(ap1302->dev, "regmap16 init failed: %ld\n",
			PTR_ERR(ap1302->regmap16));
		ret = -ENODEV;
		goto error;
	}

	ap1302->regmap32 = devm_regmap_init_i2c(client, &ap1302_reg32_config);
	if (IS_ERR(ap1302->regmap32)) {
		dev_err(ap1302->dev, "regmap32 init failed: %ld\n",
			PTR_ERR(ap1302->regmap32));
		ret = -ENODEV;
		goto error;
	}

	ret = ap1302_parse_of(ap1302);
	if (ret < 0)
		goto error;

	ret = ap1302_hw_init(ap1302);
	if (ret)
		goto error;

	ret = ap1302_config_v4l2(ap1302);
	if (ret)
		goto error_hw_cleanup;

	return 0;

error_hw_cleanup:
	ap1302_hw_cleanup(ap1302);
error:
	mutex_destroy(&ap1302->lock);
	return ret;
}

static int ap1302_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ap1302_device *ap1302 = to_ap1302(sd);

	ap1302_hw_cleanup(ap1302);

	release_firmware(ap1302->fw);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	mutex_destroy(&ap1302->lock);

	return 0;
}

static const struct of_device_id ap1302_of_id_table[] = {
	{ .compatible = "onnn,ap1302" },
	{ }
};
MODULE_DEVICE_TABLE(of, ap1302_of_id_table);

static struct i2c_driver ap1302_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= ap1302_of_id_table,
	},
	.probe		= ap1302_probe,
	.remove		= ap1302_remove,
};

module_i2c_driver(ap1302_i2c_driver);

MODULE_AUTHOR("Florian Rebaudo <frebaudo@witekio.com>");
MODULE_DESCRIPTION("Driver for ap1302 mezzanine");
MODULE_LICENSE("GPL");
