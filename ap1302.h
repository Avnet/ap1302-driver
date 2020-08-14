#ifndef __AP1302_H
#define __AP1302_H

#define AP1302_CHIP_ID		0x265
#define AP1302_REG16		2
#define AP1302_REG32		4
#define AP1302_FW_WINDOW_SIZE	0x2000
#define AP1302_FW_WINDOW_OFFSET	0x8000

#define REG_CHIP_VERSION	0x0000
#define REG_CHIP_REV		0x0050
#define REG_SIP_CRC		0xF052
#define REG_BOOTDATA_STAGE	0x6002

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
};

struct ap1302_firmware_header {
	u16 pll_init_size;
	u16 crc;
} __packed;

#endif
