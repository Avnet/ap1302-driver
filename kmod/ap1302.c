// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AP1302 external camera ISP from ON Semiconductor
 *
 * Copyright (C) 2021, Witekio, Inc.
 * Copyright (C) 2021, Xilinx, Inc.
 * Copyright (C) 2021, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
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
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define DRIVER_NAME "ap1302"

#define AP1302_FW_WINDOW_SIZE			0x2000
#define AP1302_FW_WINDOW_OFFSET			0x8000

#define AP1302_MIN_WIDTH			24U
#define AP1302_MIN_HEIGHT			16U
#define AP1302_MAX_WIDTH			4224U
#define AP1302_MAX_HEIGHT			4092U

#define AP1302_REG_16BIT(n)			((2 << 24) | (n))
#define AP1302_REG_32BIT(n)			((4 << 24) | (n))
#define AP1302_REG_SIZE(n)			((n) >> 24)
#define AP1302_REG_ADDR(n)			((n) & 0x0000ffff)
#define AP1302_REG_PAGE(n)			((n) & 0x00ff0000)
#define AP1302_REG_PAGE_MASK			0x00ff0000

/* Info Registers */
#define AP1302_CHIP_VERSION			AP1302_REG_16BIT(0x0000)
#define AP1302_CHIP_ID				0x0265
#define AP1302_FRAME_CNT			AP1302_REG_16BIT(0x0002)
#define AP1302_ERROR				AP1302_REG_16BIT(0x0006)
#define AP1302_ERR_FILE				AP1302_REG_32BIT(0x0008)
#define AP1302_ERR_LINE				AP1302_REG_16BIT(0x000c)
#define AP1302_SIPM_ERR_0			AP1302_REG_16BIT(0x0014)
#define AP1302_SIPM_ERR_1			AP1302_REG_16BIT(0x0016)
#define AP1302_CHIP_REV				AP1302_REG_16BIT(0x0050)
#define AP1302_AF_POS				AP1302_REG_16BIT(0x01B0)
#define AP1302_CON_BUF(n)			AP1302_REG_16BIT(0x0a2c + (n))
#define AP1302_CON_BUF_SIZE			512

/* Control Registers */
#define AP1302_ORIENTATION			AP1302_REG_16BIT(0x100C)
#define AP1302_ORIENTATION_HFLIP	(1U << 0)
#define AP1302_ORIENTATION_VFLIP	(1U << 1)
#define AP1302_DZ_TGT_FCT			AP1302_REG_16BIT(0x1010)
#define AP1302_SFX_MODE				AP1302_REG_16BIT(0x1016)
#define AP1302_SFX_MODE_SFX_MASK		0x00ff
#define AP1302_SFX_MODE_SFX_NORMAL		(0U << 0)
#define AP1302_SFX_MODE_SFX_ALIEN		(1U << 0)
#define AP1302_SFX_MODE_SFX_ANTIQUE		(2U << 0)
#define AP1302_SFX_MODE_SFX_BW			(3U << 0)
#define AP1302_SFX_MODE_SFX_EMBOSS		(4U << 0)
#define AP1302_SFX_MODE_SFX_EMBOSS_COLORED	(5U << 0)
#define AP1302_SFX_MODE_SFX_GRAYSCALE		(6U << 0)
#define AP1302_SFX_MODE_SFX_NEGATIVE		(7U << 0)
#define AP1302_SFX_MODE_SFX_BLUISH		(8U << 0)
#define AP1302_SFX_MODE_SFX_GREENISH		(9U << 0)
#define AP1302_SFX_MODE_SFX_REDISH		(10U << 0)
#define AP1302_SFX_MODE_SFX_POSTERIZE1		(11U << 0)
#define AP1302_SFX_MODE_SFX_POSTERIZE2		(12U << 0)
#define AP1302_SFX_MODE_SFX_SEPIA1		(13U << 0)
#define AP1302_SFX_MODE_SFX_SEPIA2		(14U << 0)
#define AP1302_SFX_MODE_SFX_SKETCH		(15U << 0)
#define AP1302_SFX_MODE_SFX_SOLARIZE		(16U << 0)
#define AP1302_SFX_MODE_SFX_FOGGY		(17U << 0)
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
 * Preview Context Registers (preview_*). AP1302 supports 3 "contexts"
 * (Preview, Snapshot, Video). These can be programmed for different size,
 * format, FPS, etc. There is no functional difference between the contexts,
 * so the only potential benefit of using them is reduced number of register
 * writes when switching output modes (if your concern is atomicity, see
 * "atomic" register).
 * So there's virtually no benefit in using contexts for this driver and it
 * would significantly increase complexity. Let's use preview context only.
 */
#define AP1302_PREVIEW_WIDTH			AP1302_REG_16BIT(0x2000)
#define AP1302_PREVIEW_HEIGHT			AP1302_REG_16BIT(0x2002)
#define AP1302_PREVIEW_ROI_X0			AP1302_REG_16BIT(0x2004)
#define AP1302_PREVIEW_ROI_Y0			AP1302_REG_16BIT(0x2006)
#define AP1302_PREVIEW_ROI_X1			AP1302_REG_16BIT(0x2008)
#define AP1302_PREVIEW_ROI_Y1			AP1302_REG_16BIT(0x200a)
#define AP1302_PREVIEW_OUT_FMT			AP1302_REG_16BIT(0x2012)
#define AP1302_PREVIEW_OUT_FMT_IPIPE_BYPASS	BIT(13)
#define AP1302_PREVIEW_OUT_FMT_SS		BIT(12)
#define AP1302_PREVIEW_OUT_FMT_FAKE_EN		BIT(11)
#define AP1302_PREVIEW_OUT_FMT_ST_EN		BIT(10)
#define AP1302_PREVIEW_OUT_FMT_IIS_NONE		(0U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_POST_VIEW	(1U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_VIDEO	(2U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_BUBBLE	(3U << 8)
#define AP1302_PREVIEW_OUT_FMT_FT_JPEG_422	(0U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_JPEG_420	(1U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_YUV		(3U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RGB		(4U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF	(5U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW8		(8U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW10		(9U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW12		(10U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW16		(11U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG8		(12U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG10		(13U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG12		(14U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG16		(15U << 4)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_ROTATE	BIT(2)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_SCAN	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_JFIF	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_EXIF	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_888	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_565	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_555M	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_555L	(3U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_422	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_420	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_400	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_SENSOR	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CAPTURE	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CP	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_BPC	(3U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_IHDR	(4U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_PP	(5U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_DENSH	(6U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_PM	(7U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_GC	(8U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CURVE	(9U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CCONV	(10U << 0)
#define AP1302_PREVIEW_S1_SENSOR_MODE		AP1302_REG_16BIT(0x202e)
#define AP1302_PREVIEW_HINF_CTRL		AP1302_REG_16BIT(0x2030)
#define AP1302_PREVIEW_HINF_CTRL_BT656_LE	BIT(15)
#define AP1302_PREVIEW_HINF_CTRL_BT656_16BIT	BIT(14)
#define AP1302_PREVIEW_HINF_CTRL_MUX_DELAY(n)	((n) << 8)
#define AP1302_PREVIEW_HINF_CTRL_LV_POL		BIT(7)
#define AP1302_PREVIEW_HINF_CTRL_FV_POL		BIT(6)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_CONT_CLK	BIT(5)
#define AP1302_PREVIEW_HINF_CTRL_SPOOF		BIT(4)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_MODE	BIT(3)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_LANES(n)	((n) << 0)

/* IQ Registers */
#define AP1302_AE_CTRL				AP1302_REG_16BIT(0x5002)
#define AP1302_AE_CTRL_STATS_SEL		BIT(11)
#define AP1302_AE_CTRL_IMM			BIT(10)
#define AP1302_AE_CTRL_ROUND_ISO		BIT(9)
#define AP1302_AE_CTRL_UROI_FACE		BIT(7)
#define AP1302_AE_CTRL_UROI_LOCK		BIT(6)
#define AP1302_AE_CTRL_UROI_BOUND		BIT(5)
#define AP1302_AE_CTRL_IMM1			BIT(4)
#define AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN	(0U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_EXP_TIME	(1U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_GAIN		(2U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_ISO		(3U << 0)
#define AP1302_AE_CTRL_AUTO_BV_EXP_TIME		(9U << 0)
#define AP1302_AE_CTRL_AUTO_BV_GAIN		(10U << 0)
#define AP1302_AE_CTRL_AUTO_BV_ISO		(11U << 0)
#define AP1302_AE_CTRL_FULL_AUTO		(12U << 0)
#define AP1302_AE_CTRL_MODE_MASK		0x000f
#define AP1302_AE_MANUAL_GAIN			AP1302_REG_16BIT(0x5006)
#define AP1302_AE_MANUAL_GAIN_MASK		0xFFFF
#define AP1302_AE_BV_OFF			AP1302_REG_16BIT(0x5014)
#define AP1302_AE_MET				AP1302_REG_16BIT(0x503E)
#define AP1302_AE_MET_MASK			0x0003
#define AP1302_AF_CTRL				AP1302_REG_16BIT(0x5058)
#define AP1302_AF_CTRL_MODE_MASK		0x0003
#define AP1302_AF_CTRL_MODE_OFF			(0U << 0)
#define AP1302_AF_CTRL_MODE_STANDBY		(1U << 0)
#define AP1302_AF_CTRL_MODE_AUTO		(2U << 0)
#define AP1302_AF_CTRL_MODE_MANUAL		(3U << 0)
#define AP1302_AF_CTRL_TRIGGER			BIT(3)
#define AP1302_AWB_CTRL				AP1302_REG_16BIT(0x5100)
#define AP1302_AWB_CTRL_RECALC			BIT(13)
#define AP1302_AWB_CTRL_POSTGAIN		BIT(12)
#define AP1302_AWB_CTRL_UNGAIN			BIT(11)
#define AP1302_AWB_CTRL_CLIP			BIT(10)
#define AP1302_AWB_CTRL_SKY			BIT(9)
#define AP1302_AWB_CTRL_FLASH			BIT(8)
#define AP1302_AWB_CTRL_FACE_OFF		(0U << 6)
#define AP1302_AWB_CTRL_FACE_IGNORE		(1U << 6)
#define AP1302_AWB_CTRL_FACE_CONSTRAINED	(2U << 6)
#define AP1302_AWB_CTRL_FACE_ONLY		(3U << 6)
#define AP1302_AWB_CTRL_IMM			BIT(5)
#define AP1302_AWB_CTRL_IMM1			BIT(4)
#define AP1302_AWB_CTRL_MODE_OFF		(0U << 0)
#define AP1302_AWB_CTRL_MODE_HORIZON		(1U << 0)
#define AP1302_AWB_CTRL_MODE_A			(2U << 0)
#define AP1302_AWB_CTRL_MODE_CWF		(3U << 0)
#define AP1302_AWB_CTRL_MODE_D50		(4U << 0)
#define AP1302_AWB_CTRL_MODE_D65		(5U << 0)
#define AP1302_AWB_CTRL_MODE_D75		(6U << 0)
#define AP1302_AWB_CTRL_MODE_MANUAL		(7U << 0)
#define AP1302_AWB_CTRL_MODE_MEASURE		(8U << 0)
#define AP1302_AWB_CTRL_MODE_AUTO		(15U << 0)
#define AP1302_AWB_CTRL_MODE_MASK		0x000f
#define AP1302_FLICK_CTRL			AP1302_REG_16BIT(0x5440)
#define AP1302_FLICK_CTRL_FREQ(n)		((n) << 8)
#define AP1302_FLICK_CTRL_ETC_IHDR_UP		BIT(6)
#define AP1302_FLICK_CTRL_ETC_DIS		BIT(5)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_MAX_ET	BIT(4)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET	BIT(3)
#define AP1302_FLICK_CTRL_FRC_EN		BIT(2)
#define AP1302_FLICK_CTRL_MODE_MASK		(3U << 0)
#define AP1302_FLICK_CTRL_MODE_DISABLED		(0U << 0)
#define AP1302_FLICK_CTRL_MODE_MANUAL		(1U << 0)
#define AP1302_FLICK_CTRL_MODE_AUTO		(2U << 0)
#define AP1302_SCENE_CTRL			AP1302_REG_16BIT(0x5454)
#define AP1302_SCENE_CTRL_MODE_MASK		0x00FF
#define AP1302_SCENE_CTRL_MODE_NORMAL		(0U << 0)
#define AP1302_SCENE_CTRL_MODE_PORTRAIT		(1U << 0)
#define AP1302_SCENE_CTRL_MODE_LANDSCAPE	(2U << 0)
#define AP1302_SCENE_CTRL_MODE_SPORT		(3U << 0)
#define AP1302_SCENE_CTRL_MODE_CLOSE_UP		(4U << 0)
#define AP1302_SCENE_CTRL_MODE_NIGHT		(5U << 0)
#define AP1302_SCENE_CTRL_MODE_TWILIGHT		(6U << 0)
#define AP1302_SCENE_CTRL_MODE_BACKLIGHT	(7U << 0)
#define AP1302_SCENE_CTRL_MODE_HIGH_SENSITIVE	(8U << 0)
#define AP1302_SCENE_CTRL_MODE_NIGHT_PORTRAIT	(9U << 0)
#define AP1302_SCENE_CTRL_MODE_BEACH		(10U << 0)
#define AP1302_SCENE_CTRL_MODE_DOCUMENT		(11U << 0)
#define AP1302_SCENE_CTRL_MODE_PARTY		(12U << 0)
#define AP1302_SCENE_CTRL_MODE_FIREWORKS	(13U << 0)
#define AP1302_SCENE_CTRL_MODE_SUNSET		(14U << 0)
#define AP1302_SCENE_CTRL_MODE_AUTO		(0xffU << 0)

/* System Registers */
#define AP1302_BOOTDATA_STAGE			AP1302_REG_16BIT(0x6002)
#define AP1302_WARNING(n)			AP1302_REG_16BIT(0x6004 + (n) * 2)
#define AP1302_SENSOR_SELECT			AP1302_REG_16BIT(0x600c)
#define AP1302_SENSOR_SELECT_TP_MODE(n)		((n) << 8)
#define AP1302_SENSOR_SELECT_PATTERN_ON		BIT(7)
#define AP1302_SENSOR_SELECT_MODE_3D_ON		BIT(6)
#define AP1302_SENSOR_SELECT_CLOCK_SEC		BIT(5)
#define AP1302_SENSOR_SELECT_CLOCK_PRIM		BIT(4)
#define AP1302_SENSOR_SELECT_RESET_SEC		BIT(3)
#define AP1302_SENSOR_SELECT_RESET_PRIM		BIT(2)
#define AP1302_SENSOR_SELECT_SENSOR_TP		(0U << 0)
#define AP1302_SENSOR_SELECT_SENSOR_PRIM	(1U << 0)
#define AP1302_SENSOR_SELECT_SENSOR_SEC		(2U << 0)
#define AP1302_SYS_START			AP1302_REG_16BIT(0x601a)
#define AP1302_SYS_START_PLL_LOCK		BIT(15)
#define AP1302_SYS_START_LOAD_OTP		BIT(12)
#define AP1302_SYS_START_RESTART_ERROR		BIT(11)
#define AP1302_SYS_START_STALL_STATUS		BIT(9)
#define AP1302_SYS_START_STALL_EN		BIT(8)
#define AP1302_SYS_START_STALL_MODE_FRAME		(0U << 6)
#define AP1302_SYS_START_STALL_MODE_DISABLED		(1U << 6)
#define AP1302_SYS_START_STALL_MODE_STANDBY		(2U << 6)
#define AP1302_SYS_START_STALL_MODE_STANDBY_SENSOR_OFF	(3U << 6)
#define AP1302_SYS_START_GO			BIT(4)
#define AP1302_SYS_START_PATCH_FUN		BIT(1)
#define AP1302_SYS_START_PLL_INIT		BIT(0)
#define AP1302_SYSTEM_FREQ_IN			AP1302_REG_32BIT(0x6024)
#define AP1302_SYSTEM_FREQ_IN_MHZ(x)		((x)<<16)
#define AP1302_HINF_MIPI_FREQ_TGT		AP1302_REG_32BIT(0x6034)
#define AP1302_HINF_MIPI_FREQ_TGT_MHZ(x)	((x)<<16)

#define AP1302_SENSOR_SIP(n)			AP1302_REG_16BIT(AP1302_PRIMARY_SENSOR_SIP + (n) * 2)

#define AP1302_PRIMARY_SENSOR_SIP		AP1302_REG_16BIT(0x604A)
#define AP1302_SECONDARY_SENSOR_SIP		AP1302_REG_32BIT(0x604C)

#define AP1302_SENSOR_SIP_ID(x)			((x&0x7F)<<1)
#define AP1302_SENSOR_SIP_AW			(1<<8)
#define AP1302_SENSOR_SIP_DW			(1<<9)
#define AP1302_SENSOR_SIP_SIPM			(1<<10)
#define AP1302_SENSOR_SIP_LANES(x)		((x&0xF)<<12)

#define AP1302_DMA_SRC				AP1302_REG_32BIT(0x60a0)
#define AP1302_DMA_DST				AP1302_REG_32BIT(0x60a4)
#define AP1302_DMA_SIP_SIPM(n)			((n) << 26)
#define AP1302_DMA_SIP_DATA_16_BIT		BIT(25)
#define AP1302_DMA_SIP_ADDR_16_BIT		BIT(24)
#define AP1302_DMA_SIP_ID(n)			((n) << 17)
#define AP1302_DMA_SIP_REG(n)			((n) << 0)
#define AP1302_DMA_SIZE				AP1302_REG_32BIT(0x60a8)
#define AP1302_DMA_CTRL				AP1302_REG_16BIT(0x60ac)
#define AP1302_DMA_CTRL_SCH_NORMAL		(0 << 12)
#define AP1302_DMA_CTRL_SCH_NEXT		(1 << 12)
#define AP1302_DMA_CTRL_SCH_NOW			(2 << 12)
#define AP1302_DMA_CTRL_DST_REG			(0 << 8)
#define AP1302_DMA_CTRL_DST_SRAM		(1 << 8)
#define AP1302_DMA_CTRL_DST_SPI			(2 << 8)
#define AP1302_DMA_CTRL_DST_SIP			(3 << 8)
#define AP1302_DMA_CTRL_SRC_REG			(0 << 4)
#define AP1302_DMA_CTRL_SRC_SRAM		(1 << 4)
#define AP1302_DMA_CTRL_SRC_SPI			(2 << 4)
#define AP1302_DMA_CTRL_SRC_SIP			(3 << 4)
#define AP1302_DMA_CTRL_MODE_32_BIT		BIT(3)
#define AP1302_DMA_CTRL_MODE_MASK		(7 << 0)
#define AP1302_DMA_CTRL_MODE_IDLE		(0 << 0)
#define AP1302_DMA_CTRL_MODE_SET		(1 << 0)
#define AP1302_DMA_CTRL_MODE_COPY		(2 << 0)
#define AP1302_DMA_CTRL_MODE_MAP		(3 << 0)
#define AP1302_DMA_CTRL_MODE_UNPACK		(4 << 0)
#define AP1302_DMA_CTRL_MODE_OTP_READ		(5 << 0)
#define AP1302_DMA_CTRL_MODE_SIP_PROBE		(6 << 0)

#define AP1302_BOOTDATA_CHECKSUM		AP1302_REG_16BIT(0x6134)

#define AP1302_BRIGHTNESS			AP1302_REG_16BIT(0x7000)
#define AP1302_CONTRAST			AP1302_REG_16BIT(0x7002)
#define AP1302_SATURATION			AP1302_REG_16BIT(0x7006)
#define AP1302_GAMMA				AP1302_REG_16BIT(0x700A)

/* Misc Registers */
#define AP1302_REG_ADV_START			0xe000
#define AP1302_ADVANCED_BASE			AP1302_REG_32BIT(0xf038)
#define AP1302_SIP_CRC				AP1302_REG_16BIT(0xf052)

/* Advanced System Registers */
#define AP1302_ADV_IRQ_SYS_INTE			AP1302_REG_32BIT(0x00230000)
#define AP1302_ADV_IRQ_SYS_INTE_TEST_COUNT	BIT(25)
#define AP1302_ADV_IRQ_SYS_INTE_HINF_1		BIT(24)
#define AP1302_ADV_IRQ_SYS_INTE_HINF_0		BIT(23)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_B_MIPI_L	(7U << 20)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_B_MIPI	BIT(19)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_A_MIPI_L	(15U << 14)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_A_MIPI	BIT(13)
#define AP1302_ADV_IRQ_SYS_INTE_SINF		BIT(12)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_S		BIT(11)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_B		BIT(10)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_A		BIT(9)
#define AP1302_ADV_IRQ_SYS_INTE_IP		BIT(8)
#define AP1302_ADV_IRQ_SYS_INTE_TIMER		BIT(7)
#define AP1302_ADV_IRQ_SYS_INTE_SIPM		(3U << 6)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_ADR_RANGE	BIT(5)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_DIRECT_WRITE	BIT(4)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_FIFO_WRITE	BIT(3)
#define AP1302_ADV_IRQ_SYS_INTE_SPI		BIT(2)
#define AP1302_ADV_IRQ_SYS_INTE_GPIO_CNT	BIT(1)
#define AP1302_ADV_IRQ_SYS_INTE_GPIO_PIN	BIT(0)

/* Advanced Slave MIPI Registers */
#define AP1302_ADV_SINF_MIPI_INTERNAL_p_LANE_n_STAT(p, n) \
	AP1302_REG_32BIT(0x00420008 + (p) * 0x50000 + (n) * 0x20)
#define AP1302_LANE_ERR_LP_VAL(n)		(((n) >> 30) & 3)
#define AP1302_LANE_ERR_STATE(n)		(((n) >> 24) & 0xf)
#define AP1302_LANE_ERR				BIT(18)
#define AP1302_LANE_ABORT			BIT(17)
#define AP1302_LANE_LP_VAL(n)			(((n) >> 6) & 3)
#define AP1302_LANE_STATE(n)			((n) & 0xf)
#define AP1302_LANE_STATE_STOP_S		0x0
#define AP1302_LANE_STATE_HS_REQ_S		0x1
#define AP1302_LANE_STATE_LP_REQ_S		0x2
#define AP1302_LANE_STATE_HS_S			0x3
#define AP1302_LANE_STATE_LP_S			0x4
#define AP1302_LANE_STATE_ESC_REQ_S		0x5
#define AP1302_LANE_STATE_TURN_REQ_S		0x6
#define AP1302_LANE_STATE_ESC_S			0x7
#define AP1302_LANE_STATE_ESC_0			0x8
#define AP1302_LANE_STATE_ESC_1			0x9
#define AP1302_LANE_STATE_TURN_S		0xa
#define AP1302_LANE_STATE_TURN_MARK		0xb
#define AP1302_LANE_STATE_ERROR_S		0xc

#define AP1302_ADV_CAPTURE_A_FV_CNT		AP1302_REG_32BIT(0x00490040)
#define AP1302_ADV_HINF_MIPI_T3		AP1302_REG_32BIT(0x840014)
#define AP1302_TCLK_POST_MASK			0xFF
#define AP1302_TCLK_POST_SHIFT			0x0
#define AP1302_TCLK_PRE_MASK			0xFF00
#define AP1302_TCLK_PRE_SHIFT			0x8


// TODO This should go in v4l2-controls.h after V4L2_CID_USER_CCS_BASE
/* The base for the AP1302 driver controls.
 * We reserve 32 controls for this driver. */
#ifndef V4L2_CID_USER_CCS_BASE
#define V4L2_CID_USER_CCS_BASE			(V4L2_CID_USER_BASE + 0x10f0)
#endif
#define V4L2_CID_USER_AP1302_BASE		(V4L2_CID_USER_CCS_BASE + 128)

#define V4L2_CID_AP1302_STEREO_ORDER				(V4L2_CID_USER_AP1302_BASE + 0)

struct ap1302_device;

enum {
	AP1302_PAD_SINK_0,
	AP1302_PAD_SINK_1,
	AP1302_PAD_SOURCE_VC0,
	AP1302_PAD_SOURCE_VC1,
	AP1302_PAD_SOURCE_VC2,
	AP1302_PAD_SOURCE_VC3,
	AP1302_PAD_MAX,
};

struct ap1302_format_info {
	unsigned int code;
	u16 out_fmt;
};

struct ap1302_format {
	struct v4l2_mbus_framefmt format;
	const struct ap1302_format_info *info;
};

struct ap1302_size {
	unsigned int width;
	unsigned int height;
};

struct ap1302_sensor_supply {
	const char *name;
	unsigned int post_delay_us;
};

struct ap1302_sensor_info {
	const char *model;
	struct ap1302_size resolution;
	u32 format;
};

struct ap1302_sensor {
	struct ap1302_device *ap1302;
	unsigned int index;

	struct device_node *of_node;
	struct device *dev;
	unsigned int num_supplies;
	struct regulator_bulk_data *supplies;
	struct ap1302_sensor_supply *supplies_data;

	struct v4l2_subdev sd;
	struct media_pad pad;

	bool link_enabled;
	u32 sip; // XXX May need to break into SIPM, I2C Addr, ...
};

static inline struct ap1302_sensor *to_ap1302_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ap1302_sensor, sd);
}

struct ap1302_device {
	struct device *dev;
	struct i2c_client *client;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *standby_gpio;
	struct regulator *vcc_supply;
	struct clk *clock;
	struct regmap *regmap16;
	struct regmap *regmap32;
	u32 reg_page;

	const struct firmware *fw;

	struct v4l2_fwnode_endpoint bus_cfg;

	struct mutex lock;	/* Protects formats */

	struct v4l2_subdev sd;
	struct media_pad pads[AP1302_PAD_MAX];
	struct ap1302_format formats[AP1302_PAD_MAX];
	unsigned int width_factor;
	bool streaming;

	struct v4l2_ctrl_handler ctrls;

	struct ap1302_sensor_info sensor_info;
	struct ap1302_sensor sensors[2];

	struct {
		struct dentry *dir;
		struct mutex lock;
		u32 sipm_addr;
	} debugfs;

	bool stall_standby;
	bool use_vcid; // virtual channel
	bool stereo_order;
	bool primary_clk_rst_only;
};

static inline struct ap1302_device *to_ap1302(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ap1302_device, sd);
}

#define FW_MAGIC 0x33315041

struct ap1302_firmware_header {
	u32 magic;
	u32 version;
	char desc[256];
	u16 pll_init_size;
	u16 crc;
} __packed;

/**
 * Allows specifying a firmware file when loading the module
 */
static char *fw_name_param=NULL;
module_param_named(fw, fw_name_param, charp, 0444);

#define MAX_FW_LOAD_RETRIES 3

static const struct ap1302_format_info supported_video_formats[] = {
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF
			 | AP1302_PREVIEW_OUT_FMT_FST_YUV_422,
	}, {
		.code = MEDIA_BUS_FMT_UYYVYY8_0_5X24,
		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF
			 | AP1302_PREVIEW_OUT_FMT_FST_YUV_420,
	},
#if defined(MEDIA_BUS_FMT_VYYUYY8_1X24) // MEDIA_BUS_FMT_VYYUYY8_1X24 for Xilinx only ?
	{
		.code = MEDIA_BUS_FMT_VYYUYY8_1X24,
		.out_fmt = AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF
			 | AP1302_PREVIEW_OUT_FMT_FST_YUV_420,
	},
#endif
};

static const struct ap1302_sensor_info ap1302_sensor_info_tpg = {
	.model = "tpg",
	.resolution = { 1920, 1080 },
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

static int __ap1302_write(struct ap1302_device *ap1302, u32 reg, u32 val)
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

static int ap1302_write(struct ap1302_device *ap1302, u32 reg, u32 val,
			int *err)
{
	u32 page = AP1302_REG_PAGE(reg);
	int ret;

	if (err && *err)
		return *err;

	if (page) {
		if (ap1302->reg_page != page) {
			ret = __ap1302_write(ap1302, AP1302_ADVANCED_BASE,
					     page);
			if (ret < 0)
				goto done;

			ap1302->reg_page = page;
		}

		reg &= ~AP1302_REG_PAGE_MASK;
		reg += AP1302_REG_ADV_START;
	}

	ret = __ap1302_write(ap1302, reg, val);

done:
	if (err && ret)
		*err = ret;

	return ret;
}

static int __ap1302_read(struct ap1302_device *ap1302, u32 reg, u32 *val)
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

static int ap1302_read(struct ap1302_device *ap1302, u32 reg, u32 *val)
{
	u32 page = AP1302_REG_PAGE(reg);
	int ret;

	if (page) {
		if (ap1302->reg_page != page) {
			ret = __ap1302_write(ap1302, AP1302_ADVANCED_BASE,
					     page);
			if (ret < 0)
				return ret;

			ap1302->reg_page = page;
		}

		reg &= ~AP1302_REG_PAGE_MASK;
		reg += AP1302_REG_ADV_START;
	}

	return __ap1302_read(ap1302, reg, val);
}

/* Setup for regmap poll */
static int __ap1302_poll_param(struct ap1302_device *ap1302, u32 reg,
		struct regmap **regmap,u16 *addr)
{
	u32 page = AP1302_REG_PAGE(reg);
	int ret;

	if (page) {
		if (ap1302->reg_page != page) {
			ret = __ap1302_write(ap1302, AP1302_ADVANCED_BASE,
					     page);
			if (ret < 0)
				return ret;

			ap1302->reg_page = page;
		}

		reg &= ~AP1302_REG_PAGE_MASK;
		reg += AP1302_REG_ADV_START;
	}

	*addr = AP1302_REG_ADDR(reg);

	switch (AP1302_REG_SIZE(reg)) {
	case 2:
		*regmap=ap1302->regmap16;
		break;
	case 4:
		*regmap=ap1302->regmap32;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(ap1302->dev, "%s: R0x%08x -> 0x%04x\n", __func__,
			reg,*addr);

	return 0;
}

#define ap1302_poll_timeout(ap1302,reg,val,cond,sleep_us,timeout_us) \
({ \
	struct regmap *__regmap; \
	u16 addr; \
	int __retpoll; \
	__retpoll = __ap1302_poll_param(ap1302,reg,&__regmap,&addr); \
	if (!__retpoll) \
		__retpoll = regmap_read_poll_timeout(__regmap, addr, val, cond, sleep_us, timeout_us); \
	__retpoll; \
})

/* -----------------------------------------------------------------------------
 * Sensor Registers Access
 *
 * Read and write sensor registers through the AP1302 DMA interface.
 */

static int ap1302_dma_wait_idle(struct ap1302_device *ap1302)
{
	unsigned int i;
	u32 ctrl;
	int ret;

	for (i = 50; i > 0; i--) {
		ret = ap1302_read(ap1302, AP1302_DMA_CTRL, &ctrl);
		if (ret < 0)
			return ret;

		if ((ctrl & AP1302_DMA_CTRL_MODE_MASK) ==
		    AP1302_DMA_CTRL_MODE_IDLE)
			break;

		usleep_range(1000, 1500);
	}

	if (!i) {
		dev_err(ap1302->dev, "DMA timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ap1302_sipm_read(struct ap1302_device *ap1302, unsigned int port,
			    u32 reg, u32 *val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u32 src;
	int ret;

	if (size > 2)
		return -EINVAL;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	// TODO Fix i2c address base on sensor

	ap1302_write(ap1302, AP1302_DMA_SIZE, size, &ret);
	src = AP1302_DMA_SIP_SIPM(port)
	    | (size == 2 ? AP1302_DMA_SIP_DATA_16_BIT : 0)
	    | AP1302_DMA_SIP_ADDR_16_BIT
	    | AP1302_DMA_SIP_ID((ap1302->sensors[0].sip>>1) & 0x3F)
	    | AP1302_DMA_SIP_REG(AP1302_REG_ADDR(reg));
	ap1302_write(ap1302, AP1302_DMA_SRC, src, &ret);

	/*
	 * Use the AP1302_DMA_DST register as both the destination address, and
	 * the scratch pad to store the read value.
	 */
	ap1302_write(ap1302, AP1302_DMA_DST, AP1302_REG_ADDR(AP1302_DMA_DST),
		     &ret);

	ap1302_write(ap1302, AP1302_DMA_CTRL,
		     AP1302_DMA_CTRL_SCH_NORMAL |
		     AP1302_DMA_CTRL_DST_REG |
		     AP1302_DMA_CTRL_SRC_SIP |
		     AP1302_DMA_CTRL_MODE_COPY, &ret);
	if (ret < 0)
		return ret;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_DMA_DST, val);
	if (ret < 0)
		return ret;

	/*
	 * The value is stored in big-endian at the DMA_DST address. The regmap
	 * uses big-endian, so 8-bit values are stored in bits 31:24 and 16-bit
	 * values in bits 23:16.
	 */
	*val >>= 32 - size * 8;

	return 0;
}

static int ap1302_sipm_write(struct ap1302_device *ap1302, unsigned int port,
			     u32 reg, u32 val)
{
	unsigned int size = AP1302_REG_SIZE(reg);
	u32 dst;
	int ret;

	if (size > 2)
		return -EINVAL;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	ap1302_write(ap1302, AP1302_DMA_SIZE, size, &ret);

	/*
	 * Use the AP1302_DMA_SRC register as both the source address, and the
	 * scratch pad to store the write value.
	 *
	 * As the AP1302 uses big endian, to store the value at address DMA_SRC
	 * it must be written in the high order bits of the registers. However,
	 * 8-bit values seem to be incorrectly handled by the AP1302, which
	 * expects them to be stored at DMA_SRC + 1 instead of DMA_SRC. The
	 * value is thus unconditionally shifted by 16 bits, unlike for DMA
	 * reads.
	 */
	ap1302_write(ap1302, AP1302_DMA_SRC,
		     (val << 16) | AP1302_REG_ADDR(AP1302_DMA_SRC), &ret);
	if (ret < 0)
		return ret;

	dst = AP1302_DMA_SIP_SIPM(port)
	    | (size == 2 ? AP1302_DMA_SIP_DATA_16_BIT : 0)
	    | AP1302_DMA_SIP_ADDR_16_BIT
	    | AP1302_DMA_SIP_ID((ap1302->sensors[0].sip>>1) & 0x3F)
	    | AP1302_DMA_SIP_REG(AP1302_REG_ADDR(reg));
	ap1302_write(ap1302, AP1302_DMA_DST, dst, &ret);

	ap1302_write(ap1302, AP1302_DMA_CTRL,
		     AP1302_DMA_CTRL_SCH_NORMAL |
		     AP1302_DMA_CTRL_DST_SIP |
		     AP1302_DMA_CTRL_SRC_REG |
		     AP1302_DMA_CTRL_MODE_COPY, &ret);
	if (ret < 0)
		return ret;

	ret = ap1302_dma_wait_idle(ap1302);
	if (ret < 0)
		return ret;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Debugfs
 */

static int ap1302_sipm_addr_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;

	mutex_lock(&ap1302->debugfs.lock);
	*val = ap1302->debugfs.sipm_addr;
	mutex_unlock(&ap1302->debugfs.lock);

	return 0;
}

static int ap1302_sipm_addr_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;

	if (val & ~0x8700ffff)
		return -EINVAL;

	switch ((val >> 24) & 7) {
	case 1:
	case 2:
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&ap1302->debugfs.lock);
	ap1302->debugfs.sipm_addr = val;
	mutex_unlock(&ap1302->debugfs.lock);

	return 0;
}

static int ap1302_sipm_data_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;
	u32 value;
	u32 addr;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	addr = ap1302->debugfs.sipm_addr;
	if (!addr) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = ap1302_sipm_read(ap1302, addr >> 30, addr & ~BIT(31),
			       &value);
	if (!ret)
		*val = value;

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_sipm_data_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;
	u32 addr;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	addr = ap1302->debugfs.sipm_addr;
	if (!addr) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = ap1302_sipm_write(ap1302, addr >> 30, addr & ~BIT(31),
				val);

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_post_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;
	u32 value;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &value);
	if (!ret)
		*val = value & AP1302_TCLK_POST_MASK;

	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_post_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;
	u32 reg, reg_val;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);
	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &reg);
	if (ret < 0)
		goto unlock;

	reg_val = reg & ~(AP1302_TCLK_POST_MASK);
	reg_val = reg_val | val;
	ret = ap1302_write(ap1302, AP1302_ADV_HINF_MIPI_T3, reg_val, NULL);

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_pre_get(void *arg, u64 *val)
{
	struct ap1302_device *ap1302 = arg;
	u32 value;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);

	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &value);
	if (!ret)
		*val = (value & AP1302_TCLK_PRE_MASK) >> AP1302_TCLK_PRE_SHIFT;

	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

static int ap1302_mipi_tclk_pre_set(void *arg, u64 val)
{
	struct ap1302_device *ap1302 = arg;
	u32 reg, reg_val;
	int ret;

	mutex_lock(&ap1302->debugfs.lock);
	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &reg);
	if (ret < 0)
		goto unlock;

	reg_val = reg & ~(AP1302_TCLK_PRE_MASK);
	reg_val = reg_val | val << AP1302_TCLK_PRE_SHIFT;
	ret = ap1302_write(ap1302, AP1302_ADV_HINF_MIPI_T3, reg_val, NULL);

unlock:
	mutex_unlock(&ap1302->debugfs.lock);

	return ret;
}

/*
 * The sipm_addr and sipm_data attributes expose access to the sensor I2C bus.
 *
 * To read or write a register, sipm_addr has to first be written with the
 * register address. The address is a 32-bit integer formatted as follows.
 *
 * I000 0SSS 0000 0000 RRRR RRRR RRRR RRRR
 *
 * I: SIPM index (0 or 1)
 * S: Size (1: 8-bit, 2: 16-bit)
 * R: Register address (16-bit)
 *
 * The sipm_data attribute can then be read to read the register value, or
 * written to write it.
 */

DEFINE_DEBUGFS_ATTRIBUTE(ap1302_sipm_addr_fops, ap1302_sipm_addr_get,
			 ap1302_sipm_addr_set, "0x%08llx\n");
DEFINE_DEBUGFS_ATTRIBUTE(ap1302_sipm_data_fops, ap1302_sipm_data_get,
			 ap1302_sipm_data_set, "0x%08llx\n");

/* The debugfs is to read and write mipi clk parameters tclk_post values */
DEFINE_DEBUGFS_ATTRIBUTE(ap1302_mipi_tclk_post_fops, ap1302_mipi_tclk_post_get,
			 ap1302_mipi_tclk_post_set, "0x%08llx\n");

/* The debugfs is to read and write mipi clk parameters and tclk_pre values */
DEFINE_DEBUGFS_ATTRIBUTE(ap1302_mipi_tclk_pre_fops, ap1302_mipi_tclk_pre_get,
			 ap1302_mipi_tclk_pre_set, "0x%08llx\n");

static void ap1302_debugfs_init(struct ap1302_device *ap1302)
{
	struct dentry *dir;
	char name[16];

	mutex_init(&ap1302->debugfs.lock);

	snprintf(name, sizeof(name), "ap1302.%s", dev_name(ap1302->dev));

	dir = debugfs_create_dir(name, NULL);
	if (IS_ERR(dir))
		return;

	ap1302->debugfs.dir = dir;

	debugfs_create_file_unsafe("sipm_addr", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_sipm_addr_fops);
	debugfs_create_file_unsafe("sipm_data", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_sipm_data_fops);
	debugfs_create_file_unsafe("mipi_tclk_post", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_mipi_tclk_post_fops);
	debugfs_create_file_unsafe("mipi_tclk_pre", 0600, ap1302->debugfs.dir,
				   ap1302, &ap1302_mipi_tclk_pre_fops);
}

static void ap1302_debugfs_cleanup(struct ap1302_device *ap1302)
{
	debugfs_remove_recursive(ap1302->debugfs.dir);
	mutex_destroy(&ap1302->debugfs.lock);
}

/* -----------------------------------------------------------------------------
 * Power Handling
 */

static int ap1302_power_on_sensors(struct ap1302_device *ap1302)
{
	struct ap1302_sensor *sensor;
	unsigned int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		sensor = &ap1302->sensors[i];
		ret = 0;

		for (j = 0; j < sensor->num_supplies; ++j) {
			unsigned int delay;

			/*
			 * We can't use regulator_bulk_enable() as it would
			 * enable all supplies in parallel, breaking the sensor
			 * power sequencing constraints.
			 */
			ret = regulator_enable(sensor->supplies[j].consumer);
			if (ret < 0) {
				dev_err(ap1302->dev,
					"Failed to enable supply %u for sensor %u\n",
					j, i);
				goto error;
			}

			delay = sensor->supplies_data[j].post_delay_us;
			usleep_range(delay, delay + 100);
		}
	}

	return 0;

error:
	for (; j > 0; --j)
		regulator_disable(sensor->supplies[j - 1].consumer);

	for (; i > 0; --i) {
		sensor = &ap1302->sensors[i - 1];
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

	/* 2. Power up the regulators. */
	if (ap1302->vcc_supply)
	{
		ret = regulator_enable(ap1302->vcc_supply);
		if (ret < 0) {
			dev_err(ap1302->dev, "Failed to enable vcc supply: %d\n", ret);
			return ret;
		}
		usleep_range(200, 1000);
	}

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
	/* Allow clock to setup */
	usleep_range(200, 1000);

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

	/* Allow Reset to occur */
	usleep_range(200, 1000);

	/* 2. Turn the clock off. */
	clk_disable_unprepare(ap1302->clock);

	/* 3. Assert STANDBY. */
	if (ap1302->standby_gpio) {
		gpiod_set_value(ap1302->standby_gpio, 1);
		usleep_range(200, 1000);
	}

	/* 4. Power down the regulators. */
	if (ap1302->vcc_supply)
	{
		regulator_disable(ap1302->vcc_supply);
		usleep_range(200, 1000);
	}

	/* 5. De-assert STANDBY. */
	if (ap1302->standby_gpio) {
		usleep_range(200, 1000);
		gpiod_set_value(ap1302->standby_gpio, 0);
	}
}

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 */

static int ap1302_dump_console(struct ap1302_device *ap1302)
{
	u8 *buffer;
	u8 *endp;
	u8 *p;
	int ret;

	buffer = kmalloc(AP1302_CON_BUF_SIZE + 1, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	// XXX use CON_WP, CON_RP see AP1302-Rev2.ini

	// 256 16-bit register : 512 bytes
	ret = regmap_raw_read(ap1302->regmap16, AP1302_CON_BUF(0), buffer,
			      AP1302_CON_BUF_SIZE/2);
	if (ret < 0) {
		dev_err(ap1302->dev, "Failed to read console buffer: %d\n",
			ret);
		goto done;
	}

	buffer[AP1302_CON_BUF_SIZE] = '\0';

	for (p = buffer; p < buffer + AP1302_CON_BUF_SIZE && *p; p = endp + 1) {
		endp = strchrnul(p, '\n');
		*endp = '\0';

		dev_info(ap1302->dev,"log > %s\n",p);
	}

	ret = 0;

done:
	kfree(buffer);
	return ret;
}

static u16 ap1302_width_factor(struct ap1302_device *ap1302)
{
	/*
	 * Width factor depends on the number of camera links
	 */

	u16 width_factor=0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		width_factor+= ap1302->sensors[i].link_enabled;
	}

	if (width_factor==0 || ap1302->use_vcid) {
		width_factor = 1;
	}
	dev_dbg(ap1302->dev,"width_factor %d\n", width_factor);
	return width_factor;
}

static int ap1302_configure(struct ap1302_device *ap1302)
{
	const struct ap1302_format *format = &ap1302->formats[AP1302_PAD_SOURCE_VC0];
	unsigned int data_lanes = ap1302->bus_cfg.bus.mipi_csi2.num_data_lanes;
	int ret = 0;
	u16 width_factor = ap1302_width_factor(ap1302);
	u8 sensor_link = ((!!ap1302->sensors[1].link_enabled)<<1) |
			!!ap1302->sensors[0].link_enabled;


	u32 value = AP1302_PREVIEW_HINF_CTRL_SPOOF |
			AP1302_PREVIEW_HINF_CTRL_MIPI_LANES(data_lanes);

	if (ap1302->use_vcid)
		value |= AP1302_PREVIEW_HINF_CTRL_MIPI_MODE;

	ap1302_write(ap1302, AP1302_PREVIEW_HINF_CTRL, value, &ret);

	value =  AP1302_SENSOR_SELECT_TP_MODE(4) ;

	switch(sensor_link)
	{
		case 0:
			value |= AP1302_SENSOR_SELECT_PATTERN_ON |
				AP1302_SENSOR_SELECT_SENSOR_TP;
			break;
		case 1:
			value |= AP1302_SENSOR_SELECT_SENSOR_PRIM |
			AP1302_SENSOR_SELECT_RESET_PRIM |
			AP1302_SENSOR_SELECT_CLOCK_PRIM;
			break;
		case 2:
			value |= AP1302_SENSOR_SELECT_SENSOR_SEC;
			if (ap1302->primary_clk_rst_only) {
				value |= AP1302_SENSOR_SELECT_RESET_PRIM |
					AP1302_SENSOR_SELECT_CLOCK_PRIM;
			}
			else {
				value |= AP1302_SENSOR_SELECT_RESET_SEC |
					AP1302_SENSOR_SELECT_CLOCK_SEC;
			}
			break;
		case 3:
			if (ap1302->use_vcid)
				value |= AP1302_SENSOR_SELECT_SENSOR_PRIM;
			else
				value |= ap1302->stereo_order ?
					AP1302_SENSOR_SELECT_SENSOR_SEC :
					AP1302_SENSOR_SELECT_SENSOR_PRIM ;

			value |= AP1302_SENSOR_SELECT_RESET_PRIM |
				AP1302_SENSOR_SELECT_CLOCK_PRIM |
				AP1302_SENSOR_SELECT_MODE_3D_ON ;

			if (!ap1302->primary_clk_rst_only)  {
				value |= AP1302_SENSOR_SELECT_RESET_SEC |
					AP1302_SENSOR_SELECT_CLOCK_SEC;
			}
	}

	dev_dbg(ap1302->dev,"Sensor Select 0x%x\n",value);
	ap1302_write(ap1302, AP1302_SENSOR_SELECT, value, &ret);

	ap1302_write(ap1302, AP1302_PREVIEW_WIDTH,
		     format->format.width / width_factor, &ret);
	ap1302_write(ap1302, AP1302_PREVIEW_HEIGHT,
		     format->format.height, &ret);
	ap1302_write(ap1302, AP1302_PREVIEW_OUT_FMT,
		     format->info->out_fmt, &ret);
	if (ret < 0)
		return ret;

	__v4l2_ctrl_handler_setup(&ap1302->ctrls);

	return 0;
}

static int ap1302_stall(struct ap1302_device *ap1302, bool stall)
{
	u32 value;
	int ret;

	ret = ap1302_read(ap1302, AP1302_SYS_START, &value);
	if (ret < 0)
		return ret;

	if ( !! (value & AP1302_SYS_START_STALL_STATUS) == stall ) {
		dev_warn(ap1302->dev,
			 "Stall status already as requested : %s\n",stall?"stalled":"running");
		return 0;
	}

	if (stall) {
		if (ap1302->stall_standby)
			dev_info(ap1302->dev,"Standby, sensor shutdown stall mode\n");

		ap1302_write(ap1302, AP1302_SYS_START,
				 AP1302_SYS_START_STALL_EN |
				 (ap1302->stall_standby?
					AP1302_SYS_START_STALL_MODE_STANDBY_SENSOR_OFF:
					AP1302_SYS_START_STALL_MODE_DISABLED)
				, &ret);
		if (ret < 0)
			return ret;

		/*
		 * Wait for Stall Status
		 */
		ret = ap1302_poll_timeout(ap1302,AP1302_SYS_START,value,
				value & AP1302_SYS_START_STALL_STATUS,
				10000,5000000);
		if (ret) {
			dev_err(ap1302->dev,"Stall Failed: %d\n",ret);
			return ret;
		}

		ap1302->streaming = false;
	} else {
		ap1302->streaming = true;

		ap1302_write(ap1302, AP1302_SYS_START,
				    AP1302_SYS_START_STALL_EN |
				    AP1302_SYS_START_STALL_MODE_DISABLED, &ret);
		if (ret < 0)
			return ret;

		/*
		 * Wait for Stall Status
		 */
		ret = ap1302_poll_timeout(ap1302,AP1302_SYS_START,value,
				!(value & AP1302_SYS_START_STALL_STATUS),
				10000,5000000);
		if (ret) {
			dev_err(ap1302->dev,"Stall Failed: %d\n",ret);
			return ret;
		}
	}
	return 0;
}

static int ap1302_set_mipi_t3_clk(struct ap1302_device *ap1302)
{
	unsigned int mipi_t3, t_clk_post, t_clk_pre;
	int ret;

	/* Set the Tclk_post and Tclk_pre values */
	ret = ap1302_read(ap1302, AP1302_ADV_HINF_MIPI_T3, &mipi_t3);
	if (ret)
		return ret;

	/* Read Tclk post default setting and increment by 2 */
	t_clk_post = ((mipi_t3 & AP1302_TCLK_POST_MASK)
					>> AP1302_TCLK_POST_SHIFT) + 0x5;
	/* Read Tclk pre default setting and increment by 1 */
	t_clk_pre = ((mipi_t3 & AP1302_TCLK_PRE_MASK)
					>> AP1302_TCLK_PRE_SHIFT) + 0x1;

	mipi_t3 = ((mipi_t3 & ~(AP1302_TCLK_POST_MASK))
					& ~(AP1302_TCLK_PRE_MASK));
	mipi_t3 = (mipi_t3 | (t_clk_pre << AP1302_TCLK_PRE_SHIFT)
					| t_clk_post);

	/* Write MIPI_T3 register with updated Tclk_post and Tclk_pre values */
	return ap1302_write(ap1302, AP1302_ADV_HINF_MIPI_T3, mipi_t3, NULL);
}

/* -----------------------------------------------------------------------------
 * V4L2 Controls
 */

static u16 ap1302_wb_values[] = {
	AP1302_AWB_CTRL_MODE_OFF,	/* V4L2_WHITE_BALANCE_MANUAL */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_AUTO */
	AP1302_AWB_CTRL_MODE_A,		/* V4L2_WHITE_BALANCE_INCANDESCENT */
	AP1302_AWB_CTRL_MODE_D50,	/* V4L2_WHITE_BALANCE_FLUORESCENT */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_FLUORESCENT_H */
	AP1302_AWB_CTRL_MODE_HORIZON,	/* V4L2_WHITE_BALANCE_HORIZON */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_DAYLIGHT */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_FLASH */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_CLOUDY */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_SHADE */
};

static int ap1302_set_wb_mode(struct ap1302_device *ap1302, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AWB_CTRL, &val);
	if (ret)
		return ret;

	val &= ~AP1302_AWB_CTRL_MODE_MASK;
	val |= ap1302_wb_values[mode];

	if (mode == V4L2_WHITE_BALANCE_FLASH)
		val |= AP1302_AWB_CTRL_FLASH;
	else
		val &= ~AP1302_AWB_CTRL_FLASH;

	return ap1302_write(ap1302, AP1302_AWB_CTRL, val, NULL);
}

static int ap1302_get_wb_mode(struct ap1302_device *ap1302, s32 *mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AWB_CTRL, &val);
	if (ret)
		return ret;

	val&=AP1302_AWB_CTRL_MODE_MASK;
	*mode = 0; // Default to V4L2_WHITE_BALANCE_MANUAL

	for(ret=0;ret<ARRAY_SIZE(ap1302_wb_values);ret++) {
		if (val == ap1302_wb_values[ret]) {
			*mode = ret;
			break;
		}
	}

	return 0;
}

static int ap1302_set_exposure(struct ap1302_device *ap1302, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AE_CTRL, &val);
	if (ret)
		return ret;

	val &= ~AP1302_AE_CTRL_MODE_MASK;
	val |= mode;

	return ap1302_write(ap1302, AP1302_AE_CTRL, val, NULL);
}

static int ap1302_get_exposure(struct ap1302_device *ap1302, s32 *mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AE_CTRL, &val);
	if (ret)
		return ret;
	*mode = val&AP1302_AE_CTRL_MODE_MASK;
	return 0;
}

static int ap1302_set_exp_met(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_AE_MET, val, NULL);
}

static int ap1302_get_exp_met(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;
	ret = ap1302_read(ap1302, AP1302_AE_MET, &val);
	if (ret)
		return ret;

	*value = val & AP1302_AE_MET_MASK;
	return 0;
}

static int ap1302_set_gain(struct ap1302_device *ap1302, s32 val)
{
	// Format is U8.8
	return ap1302_write(ap1302, AP1302_AE_MANUAL_GAIN, val, NULL);
}

static int ap1302_get_gain(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;
	ret = ap1302_read(ap1302, AP1302_AE_MANUAL_GAIN, &val);
	if (ret)
		return ret;

	*value = val & AP1302_AE_MANUAL_GAIN_MASK;
	return 0;
}

static int ap1302_set_hflip(struct ap1302_device *ap1302, s32 flip)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	val &= ~AP1302_ORIENTATION_HFLIP;
	val |= flip?AP1302_ORIENTATION_HFLIP:0;

	return ap1302_write(ap1302, AP1302_ORIENTATION, val, NULL);
}

static int ap1302_get_hflip(struct ap1302_device *ap1302, s32 *flip)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	*flip = !!(val&AP1302_ORIENTATION_HFLIP);
	return 0;
}

static int ap1302_set_vflip(struct ap1302_device *ap1302, s32 flip)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	val &= ~AP1302_ORIENTATION_VFLIP;
	val |= flip?AP1302_ORIENTATION_VFLIP:0;

	return ap1302_write(ap1302, AP1302_ORIENTATION, val, NULL);
}

static int ap1302_get_vflip(struct ap1302_device *ap1302, s32 *flip)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	*flip = !!(val&AP1302_ORIENTATION_VFLIP);
	return 0;
}

static int ap1302_set_contrast(struct ap1302_device *ap1302, s32 val)
{
	// Format is s3.12
	return ap1302_write(ap1302, AP1302_CONTRAST, val, NULL);
}

static int ap1302_get_contrast(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;
	ret = ap1302_read(ap1302, AP1302_CONTRAST, &val);
	if (ret)
		return ret;

	*value=val;
	return 0;
}

static int ap1302_set_brightness(struct ap1302_device *ap1302, s32 val)
{
	// Format is s3.12
	return ap1302_write(ap1302, AP1302_BRIGHTNESS, val, NULL);
}

static int ap1302_get_brightness(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;
	ret = ap1302_read(ap1302, AP1302_BRIGHTNESS, &val);
	if (ret)
		return ret;

	*value=val;
	return 0;
}

static int ap1302_set_saturation(struct ap1302_device *ap1302, s32 val)
{
	// Format is s3.12
	return ap1302_write(ap1302, AP1302_SATURATION, val, NULL);
}

static int ap1302_get_saturation(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;
	ret = ap1302_read(ap1302, AP1302_SATURATION, &val);
	if (ret)
		return ret;

	*value=val;
	return 0;
}

static int ap1302_set_gamma(struct ap1302_device *ap1302, s32 val)
{
	// Format is s3.12
	return ap1302_write(ap1302, AP1302_GAMMA, val, NULL);
}

static int ap1302_get_gamma(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;
	ret = ap1302_read(ap1302, AP1302_GAMMA, &val);
	if (ret)
		return ret;

	*value=val;
	return 0;
}

static int ap1302_set_zoom(struct ap1302_device *ap1302, s32 val)
{
	// Format s7.8
	return ap1302_write(ap1302, AP1302_DZ_TGT_FCT, val, NULL);
}

static int ap1302_get_zoom(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;
	ret = ap1302_read(ap1302, AP1302_DZ_TGT_FCT, &val);
	if (ret)
		return ret;

	*value=val;
	return 0;
}

static u16 ap1302_sfx_values[] = {
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_NONE */
	AP1302_SFX_MODE_SFX_BW,		/* V4L2_COLORFX_BW */
	AP1302_SFX_MODE_SFX_SEPIA1,	/* V4L2_COLORFX_SEPIA */
	AP1302_SFX_MODE_SFX_NEGATIVE,	/* V4L2_COLORFX_NEGATIVE */
	AP1302_SFX_MODE_SFX_EMBOSS,	/* V4L2_COLORFX_EMBOSS */
	AP1302_SFX_MODE_SFX_SKETCH,	/* V4L2_COLORFX_SKETCH */
	AP1302_SFX_MODE_SFX_BLUISH,	/* V4L2_COLORFX_SKY_BLUE */
	AP1302_SFX_MODE_SFX_GREENISH,	/* V4L2_COLORFX_GRASS_GREEN */
	AP1302_SFX_MODE_SFX_REDISH,	/* V4L2_COLORFX_SKIN_WHITEN */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_VIVID */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_AQUA */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_ART_FREEZE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SILHOUETTE */
	AP1302_SFX_MODE_SFX_SOLARIZE,	/* V4L2_COLORFX_SOLARIZATION */
	AP1302_SFX_MODE_SFX_ANTIQUE,	/* V4L2_COLORFX_ANTIQUE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SET_CBCR */
};

static int ap1302_set_special_effect(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SFX_MODE, ap1302_sfx_values[val]&AP1302_SFX_MODE_SFX_MASK,
			    NULL);
}

static int ap1302_get_special_effect(struct ap1302_device *ap1302, s32 *mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_SFX_MODE, &val);
	if (ret)
		return ret;

	val&=AP1302_SFX_MODE_SFX_MASK;

	*mode = V4L2_COLORFX_NONE; // Default

	for(ret=0;ret<ARRAY_SIZE(ap1302_sfx_values);ret++) {
		if (val == ap1302_sfx_values[ret]) {
			*mode = ret;
			break;
		}
	}

	return 0;
}

static u16 ap1302_scene_mode_values[] = {
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_NONE */
	AP1302_SCENE_CTRL_MODE_BACKLIGHT,	/* V4L2_SCENE_MODE_BACKLIGHT */
	AP1302_SCENE_CTRL_MODE_BEACH,		/* V4L2_SCENE_MODE_BEACH_SNOW */
	AP1302_SCENE_CTRL_MODE_TWILIGHT,	/* V4L2_SCENE_MODE_CANDLE_LIGHT */
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_DAWN_DUSK */
	AP1302_SCENE_CTRL_MODE_NORMAL,		/* V4L2_SCENE_MODE_FALL_COLORS */
	AP1302_SCENE_CTRL_MODE_FIREWORKS,	/* V4L2_SCENE_MODE_FIREWORKS */
	AP1302_SCENE_CTRL_MODE_LANDSCAPE,	/* V4L2_SCENE_MODE_LANDSCAPE */
	AP1302_SCENE_CTRL_MODE_NIGHT,		/* V4L2_SCENE_MODE_NIGHT */
	AP1302_SCENE_CTRL_MODE_PARTY,		/* V4L2_SCENE_MODE_PARTY_INDOOR */
	AP1302_SCENE_CTRL_MODE_PORTRAIT,	/* V4L2_SCENE_MODE_PORTRAIT */
	AP1302_SCENE_CTRL_MODE_SPORT,		/* V4L2_SCENE_MODE_SPORTS */
	AP1302_SCENE_CTRL_MODE_SUNSET,		/* V4L2_SCENE_MODE_SUNSET */
	AP1302_SCENE_CTRL_MODE_DOCUMENT,	/* V4L2_SCENE_MODE_TEXT */
};

static int ap1302_set_scene_mode(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_SCENE_CTRL,
			    ap1302_scene_mode_values[val]&AP1302_SCENE_CTRL_MODE_MASK, NULL);
}

static int ap1302_get_scene_mode(struct ap1302_device *ap1302, s32 *mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_SCENE_CTRL, &val);
	if (ret)
		return ret;

	val&=AP1302_SCENE_CTRL_MODE_MASK;

	*mode = V4L2_SCENE_MODE_NONE; // Default

	for(ret=0;ret<ARRAY_SIZE(ap1302_scene_mode_values);ret++) {
		if (val == ap1302_scene_mode_values[ret]) {
			*mode = ret;
			break;
		}
	}

	return 0;
}

static const u16 ap1302_flicker_values[] = {
	AP1302_FLICK_CTRL_MODE_DISABLED,
	AP1302_FLICK_CTRL_FREQ(50) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_FREQ(60) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_MODE_AUTO,
};

static int ap1302_set_flicker_freq(struct ap1302_device *ap1302, s32 val)
{
	return ap1302_write(ap1302, AP1302_FLICK_CTRL,
			    ap1302_flicker_values[val], NULL);
}

static int ap1302_get_flicker_freq(struct ap1302_device *ap1302, s32 *value)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_FLICK_CTRL, &val);
	if (ret)
		return ret;

	*value = V4L2_CID_POWER_LINE_FREQUENCY_AUTO; // Default


	if ((val & AP1302_FLICK_CTRL_MODE_MASK) ==
			AP1302_FLICK_CTRL_MODE_DISABLED) {
		*value = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
	}
	else if((val & AP1302_FLICK_CTRL_MODE_MASK) ==
			AP1302_FLICK_CTRL_MODE_MANUAL) {
		if((val>>8) == 50)
			*value = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
		if((val>>8) == 60)
			*value = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;

	}
	else if((val & AP1302_FLICK_CTRL_MODE_MASK) ==
			AP1302_FLICK_CTRL_MODE_AUTO) {
		*value = V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
	}

	return 0;
}

static int ap1302_set_auto_focus(struct ap1302_device *ap1302, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AF_CTRL, &val);
	if (ret)
		return ret;

	val &= ~(AP1302_AF_CTRL_MODE_MASK|AP1302_AF_CTRL_TRIGGER);
	if (mode)
		val |= AP1302_AF_CTRL_MODE_AUTO|AP1302_AF_CTRL_TRIGGER;

	return ap1302_write(ap1302, AP1302_AF_CTRL, val, NULL);
}

static int ap1302_get_auto_focus(struct ap1302_device *ap1302, s32 *mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(ap1302, AP1302_AF_CTRL, &val);
	if (ret)
		return ret;

	*mode = (val&AP1302_AF_CTRL_MODE_MASK)==AP1302_AF_CTRL_MODE_AUTO?1:0;
	return 0;
}

static int ap1302_set_stereo_order(struct ap1302_device *ap1302, s32 path)
{
	ap1302->stereo_order = !!path;
	return 0;
}

static int ap1302_get_stereo_order(struct ap1302_device *ap1302, s32 *path)
{
	*path = ap1302->stereo_order;
	return 0;
}

static int ap1302_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ap1302_device *ap1302 =
		container_of(ctrl->handler, struct ap1302_device, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return ap1302_set_wb_mode(ap1302, ctrl->val);

	case V4L2_CID_EXPOSURE:
		return ap1302_set_exposure(ap1302, ctrl->val);

	case V4L2_CID_EXPOSURE_METERING:
		return ap1302_set_exp_met(ap1302, ctrl->val);

	case V4L2_CID_GAIN:
		return ap1302_set_gain(ap1302, ctrl->val);

	case V4L2_CID_HFLIP:
		return ap1302_set_hflip(ap1302, ctrl->val);

	case V4L2_CID_VFLIP:
		return ap1302_set_vflip(ap1302, ctrl->val);

	case V4L2_CID_GAMMA:
		return ap1302_set_gamma(ap1302, ctrl->val);

	case V4L2_CID_CONTRAST:
		return ap1302_set_contrast(ap1302, ctrl->val);

	case V4L2_CID_BRIGHTNESS:
		return ap1302_set_brightness(ap1302, ctrl->val);

	case V4L2_CID_SATURATION:
		return ap1302_set_saturation(ap1302, ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return ap1302_set_zoom(ap1302, ctrl->val);

	case V4L2_CID_COLORFX:
		return ap1302_set_special_effect(ap1302, ctrl->val);

	case V4L2_CID_SCENE_MODE:
		return ap1302_set_scene_mode(ap1302, ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return ap1302_set_flicker_freq(ap1302, ctrl->val);

	case V4L2_CID_FOCUS_AUTO:
		return ap1302_set_auto_focus(ap1302, ctrl->val);

	case V4L2_CID_AP1302_STEREO_ORDER:
		return ap1302_set_stereo_order(ap1302, ctrl->val);

	default:
		dev_dbg(ap1302->dev, "Unknown control 0x%x\n",ctrl->id);
		return -EINVAL;
	}
}

static int ap1302_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ap1302_device *ap1302 =
		container_of(ctrl->handler, struct ap1302_device, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return ap1302_get_wb_mode(ap1302, &ctrl->val);

	case V4L2_CID_EXPOSURE:
		return ap1302_get_exposure(ap1302, &ctrl->val);

	case V4L2_CID_EXPOSURE_METERING:
		return ap1302_get_exp_met(ap1302, &ctrl->val);

	case V4L2_CID_GAIN:
		return ap1302_get_gain(ap1302, &ctrl->val);

	case V4L2_CID_HFLIP:
		return ap1302_get_hflip(ap1302, &ctrl->val);

	case V4L2_CID_VFLIP:
		return ap1302_get_vflip(ap1302, &ctrl->val);

	case V4L2_CID_GAMMA:
		return ap1302_get_gamma(ap1302, &ctrl->val);

	case V4L2_CID_CONTRAST:
		return ap1302_get_contrast(ap1302, &ctrl->val);

	case V4L2_CID_BRIGHTNESS:
		return ap1302_get_brightness(ap1302, &ctrl->val);

	case V4L2_CID_SATURATION:
		return ap1302_get_saturation(ap1302, &ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return ap1302_get_zoom(ap1302, &ctrl->val);

	case V4L2_CID_COLORFX:
		return ap1302_get_special_effect(ap1302, &ctrl->val);

	case V4L2_CID_SCENE_MODE:
		return ap1302_get_scene_mode(ap1302, &ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return ap1302_get_flicker_freq(ap1302, &ctrl->val);

	case V4L2_CID_FOCUS_AUTO:
		return ap1302_get_auto_focus(ap1302, &ctrl->val);

	case V4L2_CID_AP1302_STEREO_ORDER:
		return ap1302_get_stereo_order(ap1302, &ctrl->val);

	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops ap1302_ctrl_ops = {
	.s_ctrl = ap1302_s_ctrl,
};

static const struct v4l2_ctrl_config ap1302_ctrls[] = {
	{
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.min = 0,
		.max = 9,
		.def = 1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.name = "Gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x0, // 0: sRGB
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_CONTRAST,
		.name = "Contrast",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_BRIGHTNESS,
		.name = "Brightness",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SATURATION,
		.name = "Saturation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x1000,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xC,
		.step = 1,
		.def = 0xC,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_METERING,
		.name = "Exposure Metering",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0x3,
		.step = 1,
		.def = 0x1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_HFLIP,
		.name = "HFlip",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_VFLIP,
		.name = "VFlip",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_ZOOM_ABSOLUTE,
		.min = 0x0,
		.max = 0x1000,
		.step = 1,
		.def = 0x0100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.min = 0,
		.max = 15,
		.def = 0,
		.menu_skip_mask = BIT(15) | BIT(12) | BIT(11) | BIT(10) | BIT(9),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SCENE_MODE,
		.min = 0,
		.max = 13,
		.def = 0,
		.menu_skip_mask = BIT(5) | BIT(4),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.min = 0,
		.max = 3,
		.def = 3,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_FOCUS_AUTO,
		.name = "Auto Focus",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_AP1302_STEREO_ORDER,
		.name = "Stereo Order",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
};

static int ap1302_ctrls_init(struct ap1302_device *ap1302)
{
	unsigned int i;
	int ret;

	ret = v4l2_ctrl_handler_init(&ap1302->ctrls, ARRAY_SIZE(ap1302_ctrls));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ap1302_ctrls); i++)
	{
		struct v4l2_ctrl * ctrl = v4l2_ctrl_new_custom(&ap1302->ctrls,
								&ap1302_ctrls[i], NULL);
		ret = ap1302_g_ctrl(ctrl);
		if (!ret && ctrl->default_value != ctrl->val) {
			// Updating default value based on firmware values
			dev_info(ap1302->dev,"Ctrl '%s' default value updated from %lld to %d\n",
					ctrl->name, ctrl->default_value, ctrl->val);
			ctrl->default_value = ctrl->val;
			ctrl->cur.val = ctrl->val;
		}
	}

	if (ap1302->ctrls.error) {
		ret = ap1302->ctrls.error;
		v4l2_ctrl_handler_free(&ap1302->ctrls);
		return ret;
	}

	/* Use same lock for controls as for everything else. */
	ap1302->ctrls.lock = &ap1302->lock;
	ap1302->sd.ctrl_handler = &ap1302->ctrls;

	return 0;
}

static void ap1302_ctrls_cleanup(struct ap1302_device *ap1302)
{
	v4l2_ctrl_handler_free(&ap1302->ctrls);
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static struct v4l2_mbus_framefmt *
ap1302_get_pad_format(struct ap1302_device *ap1302,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ap1302->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ap1302->formats[pad].format;
	default:
		return NULL;
	}
}

static int ap1302_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state)
{
	u32 which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_sensor_info *info = &ap1302->sensor_info;
	unsigned int pad;

	for (pad = 0; pad < ARRAY_SIZE(ap1302->formats); ++pad) {
		struct v4l2_mbus_framefmt *format =
			ap1302_get_pad_format(ap1302, sd_state, pad, which);

		format->width = info->resolution.width;
		format->height = info->resolution.height;

		/*
		 * The source pad combines images side by side in multi-sensor
		 * setup.
		 */
		if (pad >= AP1302_PAD_SOURCE_VC0) {
			format->width *= ap1302_width_factor(ap1302);
			format->code = ap1302->formats[pad].info->code;
		} else {
			format->code = info->format;
		}

		format->field = V4L2_FIELD_NONE;
		format->colorspace = V4L2_COLORSPACE_SRGB;
	}

	return 0;
}

static int ap1302_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);

	if (code->pad < AP1302_PAD_SOURCE_VC0) {
		/*
		 * On the sink pads, only the format produced by the sensor is
		 * supported.
		 */
		if (code->index)
			return -EINVAL;

		code->code = ap1302->sensor_info.format;
	} else {
		/* On the source pad, multiple formats are supported. */
		if (code->index >= ARRAY_SIZE(supported_video_formats))
			return -EINVAL;

		code->code = supported_video_formats[code->index].code;
	}

	return 0;
}

static int ap1302_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	unsigned int i;

	if (fse->index)
		return -EINVAL;

	if (fse->pad < AP1302_PAD_SOURCE_VC0) {
		/*
		 * On the sink pads, only the size produced by the sensor is
		 * supported.
		 */
		if (fse->code != ap1302->sensor_info.format)
			return -EINVAL;

		fse->min_width = ap1302->sensor_info.resolution.width;
		fse->min_height = ap1302->sensor_info.resolution.height;
		fse->max_width = ap1302->sensor_info.resolution.width;
		fse->max_height = ap1302->sensor_info.resolution.height;
	} else {
		/*
		 * On the source pad, the AP1302 can freely scale within the
		 * scaler's limits.
		 */
		for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
			if (supported_video_formats[i].code == fse->code)
				break;
		}

		if (i >= ARRAY_SIZE(supported_video_formats))
			return -EINVAL;

		fse->min_width = AP1302_MIN_WIDTH * ap1302_width_factor(ap1302);
		fse->min_height = AP1302_MIN_HEIGHT;
		fse->max_width = AP1302_MAX_WIDTH;
		fse->max_height = AP1302_MAX_HEIGHT;
	}

	return 0;
}

static int ap1302_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct v4l2_mbus_framefmt *format;

	format = ap1302_get_pad_format(ap1302, sd_state, fmt->pad, fmt->which);

	mutex_lock(&ap1302->lock);
	fmt->format = *format;
	mutex_unlock(&ap1302->lock);

	return 0;
}

static int ap1302_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_format_info *info = NULL;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;
	u16 width_factor;

	/* Formats on the sink pads can't be changed. */
	if (fmt->pad < AP1302_PAD_SOURCE_VC0)
		return ap1302_get_fmt(sd, sd_state, fmt);

	format = ap1302_get_pad_format(ap1302, sd_state, fmt->pad, fmt->which);

	/* Validate the media bus code, default to the first supported value. */
	for (i = 0; i < ARRAY_SIZE(supported_video_formats); i++) {
		if (supported_video_formats[i].code == fmt->format.code) {
			info = &supported_video_formats[i];
			break;
		}
	}

	if (!info)
		info = &supported_video_formats[0];


	width_factor = ap1302_width_factor(ap1302);
	/*
	 * Clamp the size. The width must be a multiple of 4 (or 8 in the
	 * dual-sensor case) and the height a multiple of 2.
	 */
	fmt->format.width = clamp(ALIGN_DOWN(fmt->format.width,
					     4 * width_factor),
				  AP1302_MIN_WIDTH * width_factor,
				  AP1302_MAX_WIDTH);
	fmt->format.height = clamp(ALIGN_DOWN(fmt->format.height, 2),
				   AP1302_MIN_HEIGHT, AP1302_MAX_HEIGHT);

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

static int ap1302_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	const struct ap1302_size *resolution = &ap1302->sensor_info.resolution;

	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = resolution->width * ap1302_width_factor(ap1302);
		sel->r.height = resolution->height;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ap1302_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	int ret;

	mutex_lock(&ap1302->lock);

	if (enable == ap1302->streaming)
		goto done;

	if (enable) {
		ret = ap1302_configure(ap1302);
		if (ret < 0)
			goto done;

		ret = ap1302_stall(ap1302, false);
	} else {
		ret = ap1302_stall(ap1302, true);
	}

done:
	mutex_unlock(&ap1302->lock);

	if (ret < 0)
		dev_err(ap1302->dev, "Failed to %s stream: %d\n",
			enable ? "start" : "stop", ret);

	return ret;
}

static const char * const ap1302_warnings[] = {
	"HINF_BANDWIDTH",
	"FLICKER_DETECTION",
	"FACED_NE",
	"SMILED_NE",
	"HINF_OVERRUN",
	NULL,
	"FRAME_TOO_SMALL",
	"MISSING_PHASES",
	"SPOOF_UNDERRUN",
	"JPEG_NOLAST",
	"NO_IN_FREQ_SPEC",
	"SINF0",
	"SINF1",
	"CAPTURE0",
	"CAPTURE1",
	"ISR_UNHANDLED",
	"INTERLEAVE_SPOOF",
	"INTERLEAVE_BUF",
	"COORD_OUT_OF_RANGE",
	"ICP_CLOCKING",
	"SENSOR_CLOCKING",
	"SENSOR_NO_IHDR",
	"DIVIDE_BY_ZERO",
	"INT0_UNDERRUN",
	"INT1_UNDERRUN",
	"SCRATCHPAD_TOO_BIG",
	"OTP_RECORD_READ",
	"NO_LSC_IN_OTP",
	"GPIO_INT_LOST",
	"NO_PDAF_DATA",
	"FAR_PDAF_ACCESS_SKIP",
	"PDAF_ERROR",
	"ATM_TVI_BOUNDS",
	"SIPM_0_RTY",
	"SIPM_1_TRY",
	"SIPM_0_NO_ACK",
	"SIPM_1_NO_ACK",
	"SMILE_DIS",
	"DVS_DIS",
	"TEST_DIS",
	"SENSOR_LV2LV",
	"SENSOR_FV2FV",
	"FRAME_LOST",
};

static const char * const ap1302_lane_states[] = {
	"stop_s",
	"hs_req_s",
	"lp_req_s",
	"hs_s",
	"lp_s",
	"esc_req_s",
	"turn_req_s",
	"esc_s",
	"esc_0",
	"esc_1",
	"turn_s",
	"turn_mark",
	"error_s",
};

static int ap1302_log_status(struct v4l2_subdev *sd)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	u16 frame_count_icp;
	u16 frame_count_brac;
	u16 frame_count_hinf;
	u32 warning[4];
	u32 error[3];
	unsigned int i;
	u32 value;
	int ret;

	/* Dump the console buffer. */
	ap1302_dump_console(ap1302);


	/* Print errors. */
	ret = ap1302_read(ap1302, AP1302_ERROR, &error[0]);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_ERR_FILE, &error[1]);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_ERR_LINE, &error[2]);
	if (ret < 0)
		return ret;

	dev_info(ap1302->dev, "ERROR: 0x%04x (file 0x%08x:%u)\n",
		 error[0], error[1], error[2]);

	ret = ap1302_read(ap1302, AP1302_SIPM_ERR_0, &error[0]);
	if (ret < 0)
		return ret;

	ret = ap1302_read(ap1302, AP1302_SIPM_ERR_1, &error[1]);
	if (ret < 0)
		return ret;

	dev_info(ap1302->dev, "SIPM_ERR [0] 0x%04x [1] 0x%04x\n",
		 error[0], error[1]);

	/* Print warnings. */
	for (i = 0; i < ARRAY_SIZE(warning); ++i) {
		ret = ap1302_read(ap1302, AP1302_WARNING(i), &warning[i]);
		if (ret < 0)
			return ret;
	}

	dev_info(ap1302->dev,
		 "WARNING [0] 0x%04x [1] 0x%04x [2] 0x%04x [3] 0x%04x\n",
		 warning[0], warning[1], warning[2], warning[3]);

	for (i = 0; i < ARRAY_SIZE(ap1302_warnings); ++i) {
		if ((warning[i / 16] & BIT(i % 16)) &&
		    ap1302_warnings[i])
			dev_info(ap1302->dev, "- WARN_%s\n",
				 ap1302_warnings[i]);
	}

	/* Print the frame counter. */
	ret = ap1302_read(ap1302, AP1302_FRAME_CNT, &value);
	if (ret < 0)
		return ret;

	frame_count_hinf = value >> 8;
	frame_count_brac = value & 0xff;

	ret = ap1302_read(ap1302, AP1302_ADV_CAPTURE_A_FV_CNT, &value);
	if (ret < 0)
		return ret;

	frame_count_icp = value & 0xffff;

	dev_info(ap1302->dev, "Frame counters: ICP %u, HINF %u, BRAC %u\n",
		 frame_count_icp, frame_count_hinf, frame_count_brac);

	/* Provide Autofocus Info */
	ret = ap1302_read(ap1302, AP1302_AF_CTRL, &value);
	if (ret < 0)
		return ret;
	if ( (value & AP1302_AF_CTRL_MODE_MASK) == AP1302_AF_CTRL_MODE_AUTO) {
		ret = ap1302_read(ap1302, AP1302_AF_POS, &value);
		if (ret < 0)
			return ret;
		dev_info(ap1302->dev, "AF Pos: %d\n",(s16)value);
	}

	return 0;
}

static int ap1302_subdev_registered(struct v4l2_subdev *sd)
{
	struct ap1302_device *ap1302 = to_ap1302(sd);
	unsigned int i;
	const char *status;
	int ret;
	u32 flag;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		dev_dbg(ap1302->dev, "registering sensor %u\n", i);

		ret = v4l2_device_register_subdev(sd->v4l2_dev, &sensor->sd);
		if (ret)
			return ret;

		flag=0;
		status = of_get_property(sensor->of_node, "status", &ret);
		if (status == NULL) {
			// No sensor node, unconnected, disabled
			flag = MEDIA_LNK_FL_IMMUTABLE;
		}
		else if (ret > 0) {
			if (!strcmp(status, "okay") || !strcmp(status, "ok"))
				flag = MEDIA_LNK_FL_ENABLED;
			else if (!strcmp(status, "immutable"))
				flag = MEDIA_LNK_FL_ENABLED |
					MEDIA_LNK_FL_IMMUTABLE;
			// else disabled
		}

		ret = media_create_pad_link(&sensor->sd.entity, 0,
					    &sd->entity, i,
					    flag);
		if (ret)
			return ret;

		/* Notify MIPI subdev entity */
		ret = media_entity_call(&sd->entity, link_setup,
					&sd->entity.pads[i],
					&sensor->sd.entity.pads[0],
					flag);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;
	}

	return 0;
}

/* mipi csi2 subdev media entity operations */
static int ap1302_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);

	struct ap1302_device *ap1302 = to_ap1302(sd);
	dev_dbg(ap1302->dev, "%s <-> %s \n",local->entity->name, remote->entity->name);
	dev_dbg(ap1302->dev, "ap1302_link_setup flags %lx indx %d : %d\n",local->flags,local->index,flags);

	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case AP1302_PAD_SOURCE_VC0:
		case AP1302_PAD_SOURCE_VC2:
		case AP1302_PAD_SOURCE_VC3:
			// VC0 is the default
			// VC2 and VC3 would need support for VC selection
			break;
		case AP1302_PAD_SOURCE_VC1:
			ap1302->use_vcid = flags & MEDIA_LNK_FL_ENABLED? 1 : 0;
			break;
		default:
			return -EINVAL;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		switch (local->index) {
		case AP1302_PAD_SINK_0:
		case AP1302_PAD_SINK_1:
			ap1302->sensors[local->index].link_enabled = !!(flags & MEDIA_LNK_FL_ENABLED);
			dev_dbg(ap1302->dev,"Sensor %d Link %s\n",local->index,
					flags & MEDIA_LNK_FL_ENABLED? "Enabled"
							: "Disabled");

			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static const struct media_entity_operations ap1302_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.link_setup = ap1302_link_setup
};

static const struct v4l2_subdev_pad_ops ap1302_pad_ops = {
	.init_cfg = ap1302_init_cfg,
	.enum_mbus_code = ap1302_enum_mbus_code,
	.enum_frame_size = ap1302_enum_frame_size,
	.get_fmt = ap1302_get_fmt,
	.set_fmt = ap1302_set_fmt,
	.get_selection = ap1302_get_selection,
	.set_selection = ap1302_get_selection,
};

static const struct v4l2_subdev_video_ops ap1302_video_ops = {
	.s_stream = ap1302_s_stream,
};

static const struct v4l2_subdev_core_ops ap1302_core_ops = {
	.log_status = ap1302_log_status,
};

static const struct v4l2_subdev_ops ap1302_subdev_ops = {
	.core = &ap1302_core_ops,
	.video = &ap1302_video_ops,
	.pad = &ap1302_pad_ops,
};

static const struct v4l2_subdev_internal_ops ap1302_subdev_internal_ops = {
	.registered = ap1302_subdev_registered,
};

/* -----------------------------------------------------------------------------
 * Sensor
 */

static int ap1302_sensor_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *sd_state,
					struct v4l2_subdev_mbus_code_enum *code)
{
	struct ap1302_sensor *sensor = to_ap1302_sensor(sd);
	const struct ap1302_sensor_info *info = &sensor->ap1302->sensor_info;

	if (code->index)
		return -EINVAL;

	code->code = info->format;
	return 0;
}

static int ap1302_sensor_enum_frame_size(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_frame_size_enum *fse)
{
	struct ap1302_sensor *sensor = to_ap1302_sensor(sd);
	const struct ap1302_sensor_info *info = &sensor->ap1302->sensor_info;

	if (fse->index)
		return -EINVAL;

	if (fse->code != info->format)
		return -EINVAL;

	fse->min_width = info->resolution.width;
	fse->min_height = info->resolution.height;
	fse->max_width = info->resolution.width;
	fse->max_height = info->resolution.height;

	return 0;
}

static int ap1302_sensor_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ap1302_sensor *sensor = to_ap1302_sensor(sd);
	const struct ap1302_sensor_info *info = &sensor->ap1302->sensor_info;

	memset(&fmt->format, 0, sizeof(fmt->format));

	fmt->format.width = info->resolution.width;
	fmt->format.height = info->resolution.height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.code = info->format;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static const struct v4l2_subdev_pad_ops ap1302_sensor_pad_ops = {
	.enum_mbus_code = ap1302_sensor_enum_mbus_code,
	.enum_frame_size = ap1302_sensor_enum_frame_size,
	.get_fmt = ap1302_sensor_get_fmt,
	.set_fmt = ap1302_sensor_get_fmt,
};

static const struct v4l2_subdev_ops ap1302_sensor_subdev_ops = {
	.pad = &ap1302_sensor_pad_ops,
};

static int ap1302_sensor_parse_of(struct ap1302_device *ap1302,
				  struct device_node *node)
{
	struct ap1302_sensor *sensor;
	u32 reg;
	int ret;
	int num_supplies;
	struct property *prop;
	const char *supply;
	u32 delay;
	const __be32 *p;
	int count;

	/* Retrieve the sensor index from the reg property. */
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
	if (sensor->ap1302) {
		dev_warn(ap1302->dev, "Duplicate entry for sensor %u\n", reg);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "sip",
					&sensor->sip);
	if (ret) {
		dev_warn(ap1302->dev, "Sensor %d: error reading sip property %d\n", reg, ret);
	}

	num_supplies = of_property_count_strings(node, "regulator-supplies");
	if (num_supplies < 0) {
		dev_warn(ap1302->dev,
			"sensor %d: no regulator defined - skipping\n",
			reg);
		num_supplies = 0;

	}

	if (num_supplies)
	{
		sensor->num_supplies = num_supplies;

		sensor->supplies_data = devm_kzalloc(ap1302->dev, num_supplies *
					sizeof(*sensor->supplies_data), GFP_KERNEL);
		if (!sensor->supplies_data)
			return -ENOMEM;

		count = 0;
		of_property_for_each_string(node, "regulator-supplies", prop, supply)
			sensor->supplies_data[count++].name = supply;

		count = 0;
		of_property_for_each_u32(node, "regulator-delays", prop, p, delay)
			sensor->supplies_data[count++].post_delay_us = delay;
	}

	sensor->of_node = of_node_get(node);

	return 0;
}

static void ap1302_sensor_dev_release(struct device *dev)
{
	of_node_put(dev->of_node);
	kfree(dev);
}

static int ap1302_sensor_init(struct ap1302_sensor *sensor, unsigned int index)
{
	struct ap1302_device *ap1302 = sensor->ap1302;
	struct v4l2_subdev *sd = &sensor->sd;
	unsigned int i;
	int ret;

	sensor->index = index;

	/*
	 * Register a device for the sensor, to support usage of the regulator
	 * API.
	 */
	sensor->dev = kzalloc(sizeof(*sensor->dev), GFP_KERNEL);
	if (!sensor->dev)
		return -ENOMEM;

	sensor->dev->parent = ap1302->dev;
	sensor->dev->of_node = of_node_get(sensor->of_node);
	sensor->dev->release = &ap1302_sensor_dev_release;
	dev_set_name(sensor->dev, "%s-%s.%u", dev_name(ap1302->dev),
		     ap1302->sensor_info.model, index);

	ret = device_register(sensor->dev);
	if (ret < 0) {
		dev_err(ap1302->dev,
			"Failed to register device for sensor %u\n", index);
		goto error;
	}

	/* Retrieve the power supplies for the sensor, if any. */
	if (sensor->num_supplies) {
		const struct ap1302_sensor_supply *supplies =
				sensor->supplies_data;

		sensor->supplies = devm_kcalloc(ap1302->dev, sensor->num_supplies,
						sizeof(*sensor->supplies),
						GFP_KERNEL);
		if (!sensor->supplies) {
			ret = -ENOMEM;
			goto error;
		}

		for (i = 0; i < sensor->num_supplies; ++i)
			sensor->supplies[i].supply = supplies[i].name;

		ret = regulator_bulk_get(sensor->dev, sensor->num_supplies,
					 sensor->supplies);
		if (ret < 0) {
			dev_err(ap1302->dev,
				"Failed to get supplies for sensor %u\n", index);
			goto error;
		}
	}

	sd->dev = sensor->dev;
	v4l2_subdev_init(sd, &ap1302_sensor_subdev_ops);

	snprintf(sd->name, sizeof(sd->name), "%s.%s.%u",
			dev_name(ap1302->dev),ap1302->sensor_info.model, index);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0) {
		dev_err(ap1302->dev,
			"failed to initialize media entity for sensor %u: %d\n",
			index, ret);
		goto error;
	}

	return 0;

error:
	put_device(sensor->dev);
	return ret;
}

static void ap1302_sensor_cleanup(struct ap1302_sensor *sensor)
{
	media_entity_cleanup(&sensor->sd.entity);

	if (sensor->num_supplies)
		regulator_bulk_free(sensor->num_supplies, sensor->supplies);

	device_unregister(sensor->dev);
	of_node_put(sensor->of_node);
}

/* --------------------------------------------------------------------------
 * sysfs attributes
 */
static ssize_t ap1302_store_stall_standby(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct ap1302_device *ap1302 = to_ap1302(sd);

	if (strtobool(buf, &ap1302->stall_standby) < 0)
		return -EINVAL;

	return size;
}

static ssize_t ap1302_show_stall_standby(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct ap1302_device *ap1302 = to_ap1302(sd);

	return sysfs_emit(buf, "%d\n", ap1302->stall_standby);
}

static DEVICE_ATTR(stall_standby, 0644, ap1302_show_stall_standby, ap1302_store_stall_standby);

static ssize_t ap1302_show_lane_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct ap1302_device *ap1302 = to_ap1302(sd);
	static const char * const lp_states[] = {
		"00", "10", "01", "11",
	};
	unsigned int index;
	unsigned int counts[4][ARRAY_SIZE(ap1302_lane_states)];
	unsigned int samples = 0;
	unsigned int lane;
	unsigned int i;
	u32 first[4] = { 0, };
	u32 last[4] = { 0, };
	int ret;
	u32 buf_len=0;

	/* Sample the lane state. */
	for (index = 0; index < ARRAY_SIZE(ap1302->sensors); ++index) {
		struct ap1302_sensor *sensor = &ap1302->sensors[index];

		if (!sensor->ap1302)
			continue;

		samples = 0;
		memset(first, 0, sizeof(first));
		memset(last, 0, sizeof(last));
		memset(counts, 0, sizeof(counts));

		for (i = 0; i < 1000; ++i) {
			u32 values[4];

			/*
			 * Read the state of all lanes and skip read errors and invalid
			 * values.
			 */
			for (lane = 0; lane < 4; ++lane) {
				ret = ap1302_read(sensor->ap1302,
						  AP1302_ADV_SINF_MIPI_INTERNAL_p_LANE_n_STAT(index, lane),
						  &values[lane]);
				if (ret < 0)
					break;

				if (AP1302_LANE_STATE(values[lane]) >=
				    ARRAY_SIZE(ap1302_lane_states)) {
					ret = -EINVAL;
					break;
				}
			}

			if (ret < 0)
				continue;

			/* Accumulate the samples and save the first and last states. */
			for (lane = 0; lane < 4; ++lane)
				counts[lane][AP1302_LANE_STATE(values[lane])]++;

			if (!samples)
				memcpy(first, values, sizeof(first));
			memcpy(last, values, sizeof(last));

			samples++;
		}

		if (!samples)
			return buf_len;

		/*
		 * Print the LP state from the first sample, the error state from the
		 * last sample, and the states accumulators for each lane.
		 */
		for (lane = 0; lane < 4; ++lane) {
			u32 state = last[lane];
			char error_msg[25] = "";

			if (state & (AP1302_LANE_ERR | AP1302_LANE_ABORT)) {
				unsigned int err = AP1302_LANE_ERR_STATE(state);
				const char *err_state = NULL;

				err_state = err < ARRAY_SIZE(ap1302_lane_states)
					  ? ap1302_lane_states[err] : "INVALID";

				scnprintf(error_msg, sizeof(error_msg), "ERR (%s%s) %s LP%s",
					 state & AP1302_LANE_ERR ? "E" : "",
					 state & AP1302_LANE_ABORT ? "A" : "",
					 err_state,
					 lp_states[AP1302_LANE_ERR_LP_VAL(state)]);
			}
			buf_len += scnprintf(buf+buf_len,PAGE_SIZE-buf_len,
					"SINF%u L%u state: LP%s %s\n",
					 index, lane,
					 lp_states[AP1302_LANE_LP_VAL(first[lane])],
					 error_msg);

			for (i = 0; i < ARRAY_SIZE(ap1302_lane_states); ++i) {
				if (counts[lane][i])
					buf_len += scnprintf(buf+buf_len,
						PAGE_SIZE-buf_len," %s:%u",
						ap1302_lane_states[i],
						counts[lane][i]);
			}
			buf_len += scnprintf(buf+buf_len,PAGE_SIZE-buf_len,"\n");
		}
	}

	return buf_len;
}

static DEVICE_ATTR(lane_status, 0444, ap1302_show_lane_status, NULL);

/* -----------------------------------------------------------------------------
 * Boot & Firmware Handling
 */

static int ap1302_request_firmware(struct ap1302_device *ap1302)
{
	const struct ap1302_firmware_header *fw_hdr;
	unsigned int fw_size;
	char name[64];
	int ret;

	ret = snprintf(name, sizeof(name), "ap1302_%s_fw.bin",
			ap1302->sensor_info.model);
	if (ret >= sizeof(name)) {
		dev_err(ap1302->dev, "Firmware name too long\n");
		return -EINVAL;
	}

	if (fw_name_param!=NULL) {
		ret = snprintf(name, sizeof(name), "%s",fw_name_param);
		if (ret >= sizeof(name)) {
			dev_err(ap1302->dev, "Firmware name too long\n");
			return -EINVAL;
		}
	}

	dev_info(ap1302->dev, "Requesting firmware %s\n", name);

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

	if (fw_hdr->magic != FW_MAGIC) {
		dev_warn(ap1302->dev,"Invalid firmware header detected; Assuming Raw Binary\n");
	}else{
		dev_info(ap1302->dev,"Firmware header version : %d\n",fw_hdr->version);
		dev_info(ap1302->dev,"Firmware description : %s\n",fw_hdr->desc);

		fw_size = ap1302->fw->size - sizeof(*fw_hdr);

		if (fw_hdr->pll_init_size > fw_size) {
			dev_err(ap1302->dev,
				"Invalid firmware: PLL init size too large\n");
			return -EINVAL;
		}
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
	unsigned long clock_freq,clock_fp_mhz;
	const u8 *fw_data;
	unsigned int win_pos = 0,value;
	int i;
	int ret;

	fw_hdr = (const struct ap1302_firmware_header *)ap1302->fw->data;

        if (fw_hdr->magic == FW_MAGIC) {
		fw_data = (u8 *)&fw_hdr[1];
		fw_size = ap1302->fw->size - sizeof(*fw_hdr);
        }else{
		dev_warn(ap1302->dev,"Loading raw binary firmware\n");
		fw_data = ap1302->fw->data;
		fw_size = ap1302->fw->size;
	}

	// Fixed Point Calculation
#define HZ_TO_S15_16_MHZ(hz) \
	(s32)div_s64( ((s64)hz)<<16, 1000000)

	clock_freq = clk_get_rate(ap1302->clock);
	clock_fp_mhz = HZ_TO_S15_16_MHZ(clock_freq);
	dev_info(ap1302->dev,"AP1302 oscillator clock %ld hz (FP 0x%08lx)\n",clock_freq,clock_fp_mhz);

	ret = ap1302_write(ap1302, AP1302_SYSTEM_FREQ_IN,
			clock_fp_mhz, NULL);
	if (ret)
		return ret;

	// Using the first link frequency
	clock_freq = (u32)*ap1302->bus_cfg.link_frequencies;
	clock_fp_mhz = HZ_TO_S15_16_MHZ(clock_freq);
	dev_info(ap1302->dev,"AP1302 MIPI frequency %ld hz (FP 0x%08lx)\n",clock_freq,clock_fp_mhz);

	ret = ap1302_write(ap1302, AP1302_HINF_MIPI_FREQ_TGT,
			clock_fp_mhz, NULL);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {

		ret = ap1302_read(ap1302, AP1302_SENSOR_SIP(i), &value);
		if (ret < 0)
			return ret;
		dev_dbg(ap1302->dev,"Sensor %d SIP %04X\n",i,value);

		if(ap1302->sensors[i].sip) {
			ret = ap1302_write(ap1302, AP1302_SENSOR_SIP(i),
					ap1302->sensors[i].sip, NULL);
			if (ret)
				return ret;

			dev_dbg(ap1302->dev,"Sensor %d SIP update %04X\n",i,ap1302->sensors[i].sip);
		}

		ret = ap1302_read(ap1302, AP1302_SENSOR_SIP(i), &value);
		if (ret < 0)
			return ret;
		dev_dbg(ap1302->dev,"Sensor %d Read SIP %04X\n",i,value);
	}

	/* Load bootdata, pll_init_size not needed for firmware 429 and later */
	ret = ap1302_write_fw_window(ap1302, fw_data,
				     fw_size, &win_pos);
	if (ret)
		return ret;

	msleep(40);

	/*
	 * Write 0xffff to the bootdata_stage register to indicate to the
	 * AP1302 that the whole bootdata content has been loaded.
	 */
	ret = ap1302_write(ap1302, AP1302_BOOTDATA_STAGE, 0xffff, NULL);
	if (ret)
		return ret;

	msleep(10);

	/*
	 * Wait for AP1302_BOOTDATA_STAGE to become 0xFFFF
	 */
	ret = ap1302_poll_timeout(ap1302, AP1302_BOOTDATA_STAGE,
			value,value==0xFFFF, 10000, 5000000);
	if (ret < 0)
	{
		dev_err(ap1302->dev,
			 "AP1302_BOOTDATA_STAGE not 0xFFFF : %04X (POLL %d)\n",value,ret);
		// Dump
		ap1302_log_status(&ap1302->sd);
#if defined(DEBUG)
	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {

		ap1302_read(ap1302, AP1302_SENSOR_SIP(i), &value);
		dev_dbg(ap1302->dev,"Sensor %d FIRMWARE LOADED SIP %04X\n",i,value);

	}
#endif
		return ret;
	}

	/* Print errors. */
	ret = ap1302_read(ap1302, AP1302_ERROR, &value);
	if (ret < 0)
		return ret;

	if(value)
	{
		dev_warn(ap1302->dev,
			"Error Reg : %04X\n",value);
		return -EAGAIN;
	}

#if defined(DEBUG)
	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {

		ret = ap1302_read(ap1302, AP1302_SENSOR_SIP(i), &value);
		if (ret < 0)
			return ret;
		dev_dbg(ap1302->dev,"Sensor %d FIRMWARE LOADED SIP %04X\n",i,value);

		if(ap1302->sensors[i].sip) {
			ret = ap1302_write(ap1302, AP1302_SENSOR_SIP(i),
					ap1302->sensors[i].sip, NULL);
			if (ret)
				return ret;

			dev_dbg(ap1302->dev,"Sensor %d SIP update %04X\n",i,ap1302->sensors[i].sip);
		}
	}
#endif

	/* The AP1302 starts outputting frames right after boot, stop it. */
	ret = ap1302_stall(ap1302, true);
	if (ret)
		return ret;

	/* Adjust MIPI TCLK timings */
	return ap1302_set_mipi_t3_clk(ap1302);
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

		// Dump
		ap1302_log_status(&ap1302->sd);

		ap1302_power_off(ap1302);
	}

	if (retries == MAX_FW_LOAD_RETRIES) {
		dev_err(ap1302->dev,
			"Firmware load retries exceeded, aborting\n");
		ret = -ETIMEDOUT;
		goto error_power_sensors;
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
	struct v4l2_subdev *sd;
	unsigned int i;
	int ret;

	sd = &ap1302->sd;
	sd->dev = ap1302->dev;
	v4l2_i2c_subdev_init(sd, ap1302->client, &ap1302_subdev_ops);

	strscpy(sd->name, DRIVER_NAME, sizeof(sd->name));
	strlcat(sd->name, ".", sizeof(sd->name));
	strlcat(sd->name, dev_name(ap1302->dev), sizeof(sd->name));
	dev_dbg(ap1302->dev, "name %s\n", sd->name);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->internal_ops = &ap1302_subdev_internal_ops;
	// MEDIA_ENT_F_CAM_SENSOR : MEDIA_ENT_F_PROC_VIDEO_ISP not defined yet
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &ap1302_media_ops;

	for (i = 0; i < ARRAY_SIZE(ap1302->pads); ++i)
		ap1302->pads[i].flags = i >= AP1302_PAD_SOURCE_VC0
				      ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(&sd->entity, ARRAY_SIZE(ap1302->pads),
				     ap1302->pads);
	if (ret < 0) {
		dev_err(ap1302->dev, "media_entity_init failed %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ap1302->formats); ++i)
		ap1302->formats[i].info = &supported_video_formats[0];

	ret = ap1302_init_cfg(sd, NULL);
	if (ret < 0)
		goto error_media;

	ret = ap1302_ctrls_init(ap1302);
	if (ret < 0)
		goto error_media;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(ap1302->dev, "v4l2_async_register_subdev failed %d\n", ret);
		goto error_ctrls;
	}

	return 0;

error_ctrls:
	ap1302_ctrls_cleanup(ap1302);
error_media:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static int ap1302_parse_of(struct ap1302_device *ap1302)
{
	struct device_node *sensors;
	struct device_node *node;
	struct fwnode_handle *ep;
	int ret;

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

	ap1302->vcc_supply = devm_regulator_get_optional(ap1302->dev, "vcc");
	if (IS_ERR(ap1302->vcc_supply)) {
		if (PTR_ERR(ap1302->vcc_supply) == -ENODEV)
		{
			dev_warn(ap1302->dev,"vcc supply not found\n");
			ap1302->vcc_supply=NULL;
		}
		else
		{
			dev_err(ap1302->dev, "Can't get vcc supply: %ld\n",
				PTR_ERR(ap1302->vcc_supply));
			return PTR_ERR(ap1302->vcc_supply);
		}
	}

	/* Bus configuration */
	ep = fwnode_graph_get_next_endpoint(dev_fwnode(ap1302->dev), NULL);
	if (!ep)
		return -EINVAL;

	ap1302->bus_cfg.bus_type = V4L2_MBUS_CSI2_DPHY;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &ap1302->bus_cfg);
	if (ret < 0) {
		dev_err(ap1302->dev, "Failed to parse bus configuration\n");
		return ret;
	}

	if (!ap1302->bus_cfg.nr_of_link_frequencies) {
		dev_err(ap1302->dev, "no link frequencies defined");
		return -EINVAL;
	}

	/* Sensors */
	sensors = of_get_child_by_name(ap1302->dev->of_node, "sensors");
	if (!sensors) {
//		dev_err(ap1302->dev, "'sensors' child node not found\n");
//		return -EINVAL;

		memcpy(&ap1302->sensor_info,
				&ap1302_sensor_info_tpg,
				sizeof(ap1302_sensor_info_tpg));
		dev_warn(ap1302->dev, "no sensors node found, using tpg\n");
	}
	else
	{
		ret = of_property_read_string(sensors, "sensor,model",
				&ap1302->sensor_info.model);
		if (ret < 0) {
			dev_err(ap1302->dev, "missing sensor,model info\n");
			ret = -EINVAL;
			goto done;
		}
		dev_dbg(ap1302->dev, "sensor,model '%s'\n",
				ap1302->sensor_info.model);


		ret = of_property_read_u32_array(sensors, "sensor,resolution",
					(u32*)&ap1302->sensor_info.resolution,
					2);
		if (ret < 0) {
			dev_err(ap1302->dev, "missing sensor,resolution info\n");
			ret = -EINVAL;
			goto done;
		}
		dev_dbg(ap1302->dev, "sensor,resolution %d x %d\n",
				ap1302->sensor_info.resolution.width,
				ap1302->sensor_info.resolution.height);

		ret = of_property_read_u32(sensors, "sensor,format",
						&ap1302->sensor_info.format);
		if (ret < 0) {
			dev_err(ap1302->dev, "missing sensor,format info\n");
			ret = -EINVAL;
			goto done;
		}
		dev_dbg(ap1302->dev, "sensor,format 0x%x\n",
				ap1302->sensor_info.format);

		ap1302->primary_clk_rst_only = of_property_read_bool(sensors,
				"sensor,primary-clk-rst-only");
		if (ap1302->primary_clk_rst_only) {
			dev_dbg(ap1302->dev,
				"using primary sensor clock and reset only");
		}
	}


	for_each_child_of_node(sensors, node) {
		if (of_node_name_eq(node, "sensor")) {
			ret = ap1302_sensor_parse_of(ap1302, node);
			if (ret < 0) {
				ret = -EINVAL;
				goto done;
			}
		}
	}

done:
	of_node_put(sensors);
	return ret;
}

static void ap1302_cleanup(struct ap1302_device *ap1302)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		if (!sensor->ap1302)
			continue;

		ap1302_sensor_cleanup(sensor);
	}

	v4l2_fwnode_endpoint_free(&ap1302->bus_cfg);

	device_remove_file(ap1302->dev, &dev_attr_stall_standby);
	device_remove_file(ap1302->dev, &dev_attr_lane_status);

	mutex_destroy(&ap1302->lock);
}

static int ap1302_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ap1302_device *ap1302;
	unsigned int i;
	int ret;

	ap1302 = devm_kzalloc(&client->dev, sizeof(*ap1302), GFP_KERNEL);
	if (!ap1302)
		return -ENOMEM;

	ap1302->dev = &client->dev;
	ap1302->client = client;

	dev_info(ap1302->dev, "AP1302 Driver: %s\n",AP1302_DRV_TAG);

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

	for (i = 0; i < ARRAY_SIZE(ap1302->sensors); ++i) {
		struct ap1302_sensor *sensor = &ap1302->sensors[i];

		/*
		 * all sensor initialized, v4l link determine single/dual
		 */
		ap1302->sensors[i].ap1302 = ap1302;

		ret = ap1302_sensor_init(sensor, i);
		if (ret < 0)
			goto error;
	}

	ret = ap1302_hw_init(ap1302);
	if (ret)
		goto error;

	ret = device_create_file(ap1302->dev, &dev_attr_stall_standby);
	ret |= device_create_file(ap1302->dev, &dev_attr_lane_status);
	if (ret) {
		dev_err(ap1302->dev, "could not register sysfs entry\n");
		goto error_hw_cleanup;
	}

	ap1302_debugfs_init(ap1302);

	ret = ap1302_config_v4l2(ap1302);
	if (ret)
		goto error_hw_cleanup;

	return 0;

error_hw_cleanup:
	ap1302_hw_cleanup(ap1302);
error:
	ap1302_cleanup(ap1302);
	return ret;
}

static int ap1302_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ap1302_device *ap1302 = to_ap1302(sd);

	ap1302_debugfs_cleanup(ap1302);

	ap1302_hw_cleanup(ap1302);

	release_firmware(ap1302->fw);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	ap1302_ctrls_cleanup(ap1302);

	ap1302_cleanup(ap1302);

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
MODULE_AUTHOR("Stan Bertrand <sbertrand@witekio.com>");
MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_AUTHOR("Anil Kumar M <anil.mamidala@xilinx.com>");

MODULE_DESCRIPTION("ON Semiconductor AP1302 ISP driver");
MODULE_LICENSE("GPL");
