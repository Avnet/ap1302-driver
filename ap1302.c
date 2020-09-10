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

#define DRIVER_NAME "ap1302"

#define AP1302_CHIP_ID			0x265
#define AP1302_REG16			2
#define AP1302_REG32			4
#define AP1302_FW_WINDOW_SIZE		0x2000
#define AP1302_FW_WINDOW_OFFSET		0x8000

#define REG_CHIP_VERSION		0x0000
#define REG_CHIP_REV			0x0050
#define REG_SIP_CRC			0xf052
#define REG_BOOTDATA_STAGE		0x6002

struct ap1302_resolution {
	unsigned int width;
	unsigned int height;
};

struct ap1302_sensor_info {
	const char *compatible;
	const char *name;
	const struct ap1302_resolution *resolutions;
};

struct ap1302_sensor {
	const struct ap1302_sensor_info *info;
};

struct ap1302_device {
	struct device *dev;
	struct v4l2_subdev sd;
	const struct firmware *fw;
	struct media_pad pad;
	struct i2c_client *client;
	struct v4l2_mbus_framefmt formats[1];
	struct regmap *regmap16;
	struct regmap *regmap32;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *standby_gpio;
	struct clk *clock;

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

static const struct ap1302_video_format supported_video_formats[] = {
	{ 8, NULL, MEDIA_BUS_FMT_UYVY8_1X16,
	  2, 16, V4L2_PIX_FMT_YUYV, 1, 1, 2, 1, "4:2:2, packed, YUYV" },
	{ 8, NULL, MEDIA_BUS_FMT_UYVY8_1X16,
	  2, 16, V4L2_PIX_FMT_UYVY, 1, 1, 2, 1, "4:2:2, packed, UYVY" },
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

static int ap1302_read(struct ap1302_device *ap1302, u16 reg, u16 len,
		       unsigned int *val)
{
	int ret;

	if (len == AP1302_REG16)
		ret = regmap_read(ap1302->regmap16, reg, val);
	else if (len == AP1302_REG32)
		ret = regmap_read(ap1302->regmap32, reg, val);
	else
		ret = -EINVAL;

	if (ret) {
		dev_err(ap1302->dev, "Register 0x%04x read failed: %d\n",
			reg, ret);
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
		dev_err(ap1302->dev, "Register 0x%04x write failed: %d\n",
			reg, ret);
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
	ap1302_res_roundup(ap1302, &fmt->width, &fmt->height);

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
	ret = ap1302_write(ap1302, REG_SIP_CRC, AP1302_REG16, 0xffff);
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

	ret = ap1302_write(ap1302, REG_BOOTDATA_STAGE, AP1302_REG16, 0x0002);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	/* Load the rest of the bootdata content and verify the CRC. */
	ret = ap1302_write_fw_window(ap1302, fw_data + fw_hdr->pll_init_size,
				     fw_size - fw_hdr->pll_init_size, &win_pos);
	if (ret)
		return ret;

	msleep(40);

	ret = ap1302_read(ap1302, REG_SIP_CRC, AP1302_REG16, &crc);
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
	ret = ap1302_write(ap1302, REG_BOOTDATA_STAGE, AP1302_REG16, 0xffff);
	if (ret)
		return ret;

	return 0;
}

static int ap1302_detect_chip(struct ap1302_device *ap1302)
{
	unsigned int version;
	unsigned int revision;
	int ret;

	ret = ap1302_read(ap1302, REG_CHIP_VERSION, AP1302_REG16, &version);
	if (ret)
		return ret;

	ret = ap1302_read(ap1302, REG_CHIP_REV, AP1302_REG16, &revision);
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

static int ap1302_config_hw(struct ap1302_device *ap1302)
{
	unsigned int retries;
	int ret;

	ret = ap1302_request_firmware(ap1302);
	if (ret)
		goto done;

	for (retries = 0; retries < MAX_FW_LOAD_RETRIES; ++retries) {
		ret = ap1302_power_on(ap1302);
		if (ret < 0)
			goto done;

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

done:
	if (ret < 0)
		release_firmware(ap1302->fw);

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

	ap1302->formats[0].width = ap1302->sensors[0].info->resolutions[0].width;
	ap1302->formats[0].height = ap1302->sensors[0].info->resolutions[0].height;
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

static int ap1302_parse_of_sensor(struct ap1302_device *ap1302,
				  struct device_node *node)
{
	struct ap1302_sensor *sensor;
	const char *compat;
	unsigned int i;
	u32 reg;
	int ret;

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
