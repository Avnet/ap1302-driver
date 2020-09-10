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
#include <linux/regmap.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>

#include <media/media-entity.h>
#include <media/v4l2-device.h>

#define DRIVER_NAME "AP1302"

#define AP1302_CHIP_ID			0x265
#define AP1302_REG16			2
#define AP1302_REG32			4
#define AP1302_FW_WINDOW_SIZE		0x2000
#define AP1302_FW_WINDOW_OFFSET		0x8000

#define REG_CHIP_VERSION		0x0000
#define REG_CHIP_REV			0x0050
#define REG_SIP_CRC			0xf052
#define REG_BOOTDATA_STAGE		0x6002

struct ap1302_device {
	struct device *dev;
	struct v4l2_subdev sd;
	const struct firmware *fw;
	struct media_pad pad;
	struct i2c_client *client;
	u32 cam_config;
	struct v4l2_mbus_framefmt formats[1];
	struct regmap *regmap16;
	struct regmap *regmap32;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *standby_gpio;
	struct clk *clock;
};

static inline struct ap1302_device *to_ap1302(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ap1302_device, sd);
}

struct ap1302_firmware_header {
	u16 pll_init_size;
	u16 crc;
} __packed;

/* There's no standard V4L2_CID_GREEN_BALANCE defined in the
 * linux kernel. Let's borrow V4L2_CID_CHROMA_GAIN on green
 * balance adjustment
 */
#define V4L2_CID_GREEN_BALANCE	V4L2_CID_CHROMA_GAIN

#define MAX_FW_LOAD_RETRIES 3

enum {
	MODE_AP1302_AR0144_DUAL,
	MODE_AP1302_AR1335_SINGLE,
	MODE_AP1302_MAX,
};

struct ap1302_video_format {
	unsigned int width;
	const char *pattern;
	unsigned int code;
	unsigned int bpl_factor;
	unsigned int bpp;
	u32 fourcc;
	u8 num_planes;
	u8 buffers;
	u8 hsub;
	u8 vsub;
	const char *description;
};

struct ap1302_resolution {
	unsigned int width;
	unsigned int height;
};

static const struct ap1302_resolution resolutions_ar0144_dual[] = {
	{
		.width  = 2560,
		.height = 800,
	},
};

static const struct ap1302_resolution resolutions_ar1335_single[] = {
	{
		.width  = 1920,
		.height = 1080,
	},
};

static const struct ap1302_video_format supported_video_formats[] = {
	{ 8, NULL, MEDIA_BUS_FMT_UYVY8_1X16,
	  2, 16, V4L2_PIX_FMT_YUYV, 1, 1, 2, 1, "4:2:2, packed, YUYV" },
	{ 8, NULL, MEDIA_BUS_FMT_UYVY8_1X16,
	  2, 16, V4L2_PIX_FMT_UYVY, 1, 1, 2, 1, "4:2:2, packed, UYVY" },
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

static int ap1302_read(struct ap1302_device *ap1302, u16 reg, u16 len, void *val)
{
	int ret;

	if (len == AP1302_REG16)
		ret = regmap_read(ap1302->regmap16, reg, val);
	else if (len == AP1302_REG32)
		ret = regmap_read(ap1302->regmap32, reg, val);
	else
		ret = -EINVAL;

	if (ret) {
		dev_dbg(ap1302->dev, "Read reg failed. reg=0x%04X\n", reg);
		return ret;
	}

	if (len == AP1302_REG16)
		dev_dbg(ap1302->dev, "read_reg[0x%04X] = 0x%04X\n", reg, *(u16 *)val);
	else
		dev_dbg(ap1302->dev, "read_reg[0x%04X] = 0x%08X\n", reg, *(u32 *)val);

	return ret;
}

static int ap1302_write(struct ap1302_device *ap1302, u16 reg, u16 len, u32 val)
{
	int ret;
	if (len == AP1302_REG16)
		ret = regmap_write(ap1302->regmap16, reg, val);
	else if (len == AP1302_REG32)
		ret = regmap_write(ap1302->regmap32, reg, val);
	else
		ret = -EINVAL;

	if (ret) {
		dev_dbg(ap1302->dev, "Write reg failed. reg=0x%04X\n", reg);
		return ret;
	}

	return ret;
}

/* -----------------------------------------------------------------------------
 * Power Handling
 */

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
		return &ap1302->formats[pad];
	default:
		return NULL;
	}
}

static int ap1302_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	int pad = fmt->pad;

	fmt->format = *ap1302_get_pad_format(ap1302, cfg, pad, fmt->which);
	return 0;
}

static void ap1302_res_roundup(u32 *width, u32 *height, u32 mode)
{
	int i;
	int size;
	const struct ap1302_resolution *table;

	if (mode == MODE_AP1302_AR0144_DUAL) {
		table = resolutions_ar0144_dual;
		size = ARRAY_SIZE(resolutions_ar0144_dual);
	}
	else {
		table = resolutions_ar1335_single;
		size = ARRAY_SIZE(resolutions_ar1335_single);
	}

	/* TODO: Search for best match instead of rounding */
	for (i = 0; i < size; i++) {
		if ((table[i].width >= *width) && (table[i].height >= *height)) {
			*width = table[i].width;
			*height = table[i].height;
			return;
		}
	}

	/* Use default in case of no match */
	*width = table[0].width;
	*height = table[0].height;
}

static int ap1302_try_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	int i;
	struct ap1302_device *ap1302 = to_ap1302(sd);

	for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
		if (supported_video_formats[i].code == fmt->code)
			break;
	}

	if (i >= ARRAY_SIZE(supported_video_formats)) {
		/* default to first format in case no match */
		i = 0;
		fmt->code = supported_video_formats[0].code;
	}

	/* Find suitable supported resolution */
	ap1302_res_roundup(&fmt->width, &fmt->height, ap1302->cam_config);

	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int ap1302_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);

	struct v4l2_mbus_framefmt *fmt_ptr;
	struct v4l2_mbus_framefmt resp_fmt;

	fmt_ptr = ap1302_get_pad_format(ap1302, cfg, fmt->pad, fmt->which);
	resp_fmt = fmt->format;

	dev_dbg(ap1302->dev, "Configure source pad %d\n", fmt->pad);
	ap1302_try_mbus_fmt(sd, &resp_fmt);

	dev_dbg(ap1302->dev, "width %d height %d\n", resp_fmt.width, resp_fmt.height);

	*fmt_ptr = resp_fmt;
	fmt->format = resp_fmt;

	return 0;
}

static const struct media_entity_operations ap1302_media_ops = {
	.link_validate = v4l2_subdev_link_validate
};

static const struct v4l2_subdev_pad_ops ap1302_pad_ops = {
	.get_fmt = ap1302_get_fmt,
	.set_fmt = ap1302_set_fmt,
};

static const struct v4l2_subdev_ops ap1302_subdev_ops = {
	.pad = &ap1302_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Firmware Handling
 */

static int ap1302_request_firmware(struct ap1302_device *ap1302)
{
	int ret;

	if (ap1302->cam_config == MODE_AP1302_AR0144_DUAL)
		ret = request_firmware(&ap1302->fw, "ar0144_dual_fw.bin", ap1302->dev);
	else if (ap1302->cam_config == MODE_AP1302_AR1335_SINGLE)
		ret = request_firmware(&ap1302->fw, "ar1335_single_fw.bin", ap1302->dev);
	else
		ret = -EINVAL;

	if (ret)
		dev_err(ap1302->dev, "ap1302_request_firmware failed. ret=%d\n", ret);

	return ret;
}

/* When loading firmware, host writes firmware data from address 0x8000.
   When the address reaches 0x9fff, the next address should return to 0x8000.
   This function handles this address window and load firmware data to AP1302.
   win_pos indicates the offset within this window. Firmware loading procedure
   may call this function several times. win_pos records the current position
   that has been written to.*/
static int ap1302_write_fw_window(struct ap1302_device *ap1302,
				  u16 *win_pos, const u8 *buf, u32 len)
{
	int ret;
	u32 pos;
	u32 sub_len;

	for (pos = 0; pos < len; pos += sub_len) {
		if (len - pos < AP1302_FW_WINDOW_SIZE - *win_pos)
			sub_len = len - pos;
		else
			sub_len = AP1302_FW_WINDOW_SIZE - *win_pos;

		ret = regmap_raw_write(ap1302->regmap16, *win_pos + AP1302_FW_WINDOW_OFFSET,
					buf + pos, sub_len);
		if (ret)
			return ret;

		*win_pos += sub_len;
		if (*win_pos >= AP1302_FW_WINDOW_SIZE)
			*win_pos = 0;
	}

	return 0;
}

static int ap1302_load_firmware(struct ap1302_device *ap1302)
{
	const struct ap1302_firmware_header *ap1302_fw;
	const u8 *fw_data;
	u16 reg_val = 0;
	u16 win_pos = 0;
	int ret;

	dev_info(ap1302->dev, "Start to load firmware.\n");
	if (!ap1302->fw) {
		dev_err(ap1302->dev, "firmware not requested.\n");
		return -EINVAL;
	}

	ap1302_fw = (const struct ap1302_firmware_header *) ap1302->fw->data;

	/* The fw binary contains a header of struct ap1302_firmware_header.
	   Following the header is the bootdata of AP1302.
	   The bootdata pointer can be referenced as &ap1302_fw[1]. */
	fw_data = (u8 *)&ap1302_fw[1];

	/* Clear crc register. */
	ret = ap1302_write(ap1302, REG_SIP_CRC, AP1302_REG16, 0xffff);
	if (ret)
		return ret;

	/* Load FW data for PLL init stage. */
	ret = ap1302_write_fw_window(ap1302, &win_pos, fw_data, ap1302_fw->pll_init_size);
	if (ret)
		return ret;

	/* Write 2 to bootdata_stage register to apply basic_init_hp
	   settings and enable PLL. */
	ret = ap1302_write(ap1302, REG_BOOTDATA_STAGE, AP1302_REG16, 0x0002);
	if (ret)
		return ret;

	/* Wait 1ms for PLL to lock. */
	msleep(20);

	/* Load the rest of bootdata content. */
	ret = ap1302_write_fw_window(ap1302, &win_pos, fw_data + ap1302_fw->pll_init_size,
			ap1302->fw->size - sizeof(*ap1302_fw) - ap1302_fw->pll_init_size);
	if (ret)
		return ret;
	msleep(40);

	/* Check crc. */
	ret = ap1302_read(ap1302, REG_SIP_CRC, AP1302_REG16, &reg_val);
	if (ret)
		return ret;

	if (reg_val != ap1302_fw->crc) {
		dev_err(ap1302->dev, "crc does not match. T:0x%04X F:0x%04X\n",
			ap1302_fw->crc, reg_val);
		return -EAGAIN;
	}

	/* Write 0xffff to bootdata_stage register to indicate AP1302 that
	   the whole bootdata content has been loaded. */
	ret = ap1302_write(ap1302, REG_BOOTDATA_STAGE, AP1302_REG16, 0xffff);
	if (ret)
		return ret;

	dev_info(ap1302->dev, "Load firmware successfully.\n");

	return 0;
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

static int ap1302_detect_chip(struct ap1302_device *ap1302)
{
	unsigned int reg_val = 0;
	int ret;

	ret = ap1302_read(ap1302, REG_CHIP_VERSION, AP1302_REG16, &reg_val);
	if (ret || (reg_val != AP1302_CHIP_ID)) {
		dev_err(ap1302->dev,
			"Chip version does not match. ret=%d ver=0x%04x\n", ret, reg_val);
		return ret;
	}
	dev_info(ap1302->dev, "AP1302 Chip ID is 0x%X\n", reg_val);

	ret = ap1302_read(ap1302, REG_CHIP_REV, AP1302_REG16, &reg_val);
	if (ret)
		return ret;
	dev_info(ap1302->dev, "AP1302 Chip Rev is 0x%X\n", reg_val);

	return 0;
}

static int ap1302_config_hw(struct ap1302_device *ap1302)
{
	unsigned int retries;
	int ret;

	ret = ap1302_request_firmware(ap1302);
	if (ret) {
		dev_err(ap1302->dev, "Cannot request ap1302 firmware.\n");
		return ret;
	}

	for (retries = 0; retries < MAX_FW_LOAD_RETRIES; ++retries) {
		ret = ap1302_power_on(ap1302);
		if (ret < 0)
			return ret;

		ret = ap1302_detect_chip(ap1302);
		if (ret)
			break;

		ret = ap1302_load_firmware(ap1302);
		if (ret != -EAGAIN)
			break;

		ap1302_power_off(ap1302);
	}

	if (ret == -EAGAIN) {
		dev_err(ap1302->dev, "Firmware load failed, aborting\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int ap1302_config_v4l2(struct ap1302_device *ap1302)
{
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

	if (ap1302->cam_config == MODE_AP1302_AR0144_DUAL) {
		ap1302->formats[0].width = resolutions_ar0144_dual[0].width;
		ap1302->formats[0].height = resolutions_ar0144_dual[0].height;
	}
	else {
		ap1302->formats[0].width = resolutions_ar1335_single[0].width;
		ap1302->formats[0].height = resolutions_ar1335_single[0].height;
	}

	ap1302->formats[0].field = V4L2_FIELD_NONE;
	ap1302->formats[0].code = MEDIA_BUS_FMT_UYVY8_1X16;
	ap1302->formats[0].colorspace = V4L2_COLORSPACE_SRGB;

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

static int ap1302_parse_of(struct ap1302_device *ap1302)
{
	int ret;
	struct device_node *node = ap1302->dev->of_node;

	ret = of_property_read_u32(node, "onnn,cam-config", &ap1302->cam_config);
	if (ret < 0) {
		dev_err(ap1302->dev, "Missing onnn,cam-config property\n");
		return ret;
	}

	if (ap1302->cam_config >= MODE_AP1302_MAX) {
		dev_err(ap1302->dev, "Invalid cam-config\n");
		return -EINVAL;
	}

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

	return 0;
}

static int ap1302_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ap1302_device *ap1302;
	int ret;

	dev_info(&client->dev, "Probe: %s\n", DRIVER_NAME);

	ap1302 = devm_kzalloc(&client->dev, sizeof(*ap1302), GFP_KERNEL);
	if (!ap1302)
		return -ENOMEM;

	ap1302->dev = &client->dev;
	ap1302->client = client;

	ap1302->regmap16 = devm_regmap_init_i2c(client, &ap1302_reg16_config);
	if (IS_ERR(ap1302->regmap16)) {
		dev_err(ap1302->dev, "regmap16 init failed: %ld\n",
			PTR_ERR(ap1302->regmap16));
		return -ENODEV;
	}

	ap1302->regmap32 = devm_regmap_init_i2c(client, &ap1302_reg32_config);
	if (IS_ERR(ap1302->regmap32)) {
		dev_err(ap1302->dev, "regmap32 init failed: %ld\n",
			PTR_ERR(ap1302->regmap32));
		return -ENODEV;
	}

	ret = ap1302_parse_of(ap1302);
	if (ret < 0)
		return ret;

	ret = ap1302_config_hw(ap1302);
	if (ret)
		return ret;

	ret = ap1302_config_v4l2(ap1302);
	if (ret)
		return ret;

	dev_info(ap1302->dev, "%s probe successfully done\n", DRIVER_NAME);
	return 0;
}

static int ap1302_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ap1302_device *ap1302 = to_ap1302(sd);

	ap1302_power_off(ap1302);

	release_firmware(ap1302->fw);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

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
