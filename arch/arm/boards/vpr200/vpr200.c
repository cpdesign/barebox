/*
 * Copyright (C) 2007 Sascha Hauer, Pengutronix
 *               2009 Marc Kleine-Budde, Pengutronix
 *               2010 Marc Reilly, Creative Product Design
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Derived from:
 *
 * * mx35_3stack.c - board file for uboot-v1
 *   Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *   (C) Copyright 2008-2009 Freescale Semiconductor, Inc.
 *
 */

#include <common.h>
#include <environment.h>
#include <magicvar.h>
#include <errno.h>
#include <fcntl.h>
#include <fec.h>
#include <fs.h>
#include <init.h>
#include <nand.h>
#include <net.h>
#include <partition.h>
#include <command.h>
#include <getopt.h>
#include <linux/stat.h>

#include <asm/armlinux.h>
#include <io.h>
#include <asm/mmu.h>
#include <generated/mach-types.h>

#include <mach/gpio.h>
#include <mach/imx-nand.h>
#include <mach/imx-regs.h>
#include <mach/iomux-mx35.h>
#include <mach/iomux-v3.h>
#include <mach/imx-ipu-fb.h>
#include <mach/generic.h>
#include <mach/devices-imx35.h>

#include <i2c/i2c.h>
#include <mfd/mc13xxx.h>
#include <led.h>
#include <misc/isl22316.h>
#include <misc/isl22346.h>

/* ------------------------------------------------------------------------- */
/* Board revs for the VPR CPU */
#define VPR_CPU_V2	0x11
#define VPR_CPU_V3	0x19
#define VPR_CPU_V4	0x1a

/* Main board revs. V1 board had no rev ID lines */
#define VPR_BOARD_V2	0x1
#define VPR_BOARD_V3	0x2
/* changed bmp085 to bmp280, no difference to barebox */
#define VPR_BOARD_V4	0x3
/* changed isl2236 to isl22346 */
#define VPR_BOARD_V5	0x4

#define VPR_DIAG_LED_NAME "diag"

/* GPIO numbers */
#define LCD_PWR_GPIO		IMX_GPIO_NR(1, 2)
#define LCD_LED_ENABLE_GPIO	IMX_GPIO_NR(2, 7)

#define MB3V3REG_ENABLE_GPIO	IMX_GPIO_NR(1, 15)
#define MB5VREG_ENABLE_GPIO	IMX_GPIO_NR(1, 3)

#define FEC_RESET_GPIO		IMX_GPIO_NR(3, 0)

#define DIAG_RED_GPIO		IMX_GPIO_NR(3, 14)
#define DIAG_GREEN_GPIO_V3	IMX_GPIO_NR(3, 15)
#define DIAG_GREEN_GPIO		IMX_GPIO_NR(1, 14)
#define DIAG_BLUE_GPIO		IMX_GPIO_NR(3, 30)

#define CPU_CFG_0_GPIO		IMX_GPIO_NR(2, 31)
#define CPU_CFG_1_GPIO		IMX_GPIO_NR(2, 27)
#define CPU_CFG_2_GPIO		IMX_GPIO_NR(2, 28)
#define CPU_CFG_3_GPIO		IMX_GPIO_NR(2, 29)
#define CPU_CFG_4_GPIO		IMX_GPIO_NR(2, 30)

#define BOARD_CFG_0_GPIO	IMX_GPIO_NR(2, 8)
#define BOARD_CFG_1_GPIO	IMX_GPIO_NR(2, 9)
#define BOARD_CFG_2_GPIO	IMX_GPIO_NR(2, 10)
#define BOARD_CFG_3_GPIO	IMX_GPIO_NR(2, 11)

#define BUTTON1_GPIO		IMX_GPIO_NR(1, 4)
#define BUTTON2_GPIO		IMX_GPIO_NR(1, 5)
#define BUTTON3_GPIO		IMX_GPIO_NR(1, 7)
#define BUTTON4_GPIO		IMX_GPIO_NR(1, 8)
#define BUTTON5_GPIO		IMX_GPIO_NR(1, 9)
#define BUTTON6_GPIO		IMX_GPIO_NR(1, 10)
#define BUTTON7_GPIO		IMX_GPIO_NR(1, 11)
#define BUTTON8_GPIO		IMX_GPIO_NR(1, 12)

#define BUZZER_GPIO_V2		IMX_GPIO_NR(1, 13)
#define BUZZER_GPIO		IMX_GPIO_NR(1, 1)
#define SPEAKER_GPIO_V3		IMX_GPIO_NR(1, 14)

#define BP_RESET_GPIO		IMX_GPIO_NR(2, 18)
#define BP_EOC_GPIO		IMX_GPIO_NR(2, 17)

#define RF_RESET_GPIO		IMX_GPIO_NR(2, 12)
#define RF_TDI_GPIO		IMX_GPIO_NR(2, 13)
#define RF_TDO_GPIO		IMX_GPIO_NR(2, 14)
#define RF_TCK_GPIO		IMX_GPIO_NR(2, 15)
#define RF_TMS_GPIO		IMX_GPIO_NR(2, 16)

#define GSM_GPIO		IMX_GPIO_NR(1, 13)

#define GPIO_SD_CD		IMX_GPIO_NR(3, 1)
#define GPIO_SD_WP		IMX_GPIO_NR(3, 2)

#define GPIO_SHUNT_ENABLE	IMX_GPIO_NR(2, 1)
#define GPIO_SHUNT_SENSE	IMX_GPIO_NR(2, 3)

/* ------------------------------------------------------------------------- */

static uint32_t vpr_cpu_rev;
static uint32_t vpr_board_rev;

/* ------------------------------------------------------------------------- */

static int vpr_setup_ethaddr(void)
{
	unsigned char buf[6];
	int fd;
	int ret = -EINVAL;

	fd = open("/dev/eeprom0", O_RDONLY);
	if (fd < 0) {
		fd = open("/dev/eeprom", O_RDONLY);
		if (fd < 0) {
			printf("Couldn't open eeprom to get MAC\n");
			goto out;
		}
	}

	if (lseek(fd, 0, SEEK_SET) < 0) {
		printf("Couldn't locate MAC in eeprom\n");
		goto out_close;
	}

	if (read(fd, buf, 6) < 0) {
		printf("Couldn't read eeprom to get MAC\n");
		goto out_close;
	}

	ret = is_valid_ether_addr(buf) ? 0 : -EINVAL;

	if (ret)
		printf("WARN: MAC address stored in eeprom is invalid\n");
	else
		eth_register_ethaddr(0, buf);

out_close:
	close(fd);
out:
	return ret;
}

static struct fec_platform_data fec_info = {
	.xcv_type	= MII100,
	.phy_addr	= 0x0,
};

static int vpr200_mem_init(void)
{
	arm_add_mem_device("ram0", IMX_SDRAM_CS0, 128 * 1024 * 1024);
	arm_add_mem_device("ram1", IMX_SDRAM_CS1, 128 * 1024 * 1024);

	return 0;
}
mem_initcall(vpr200_mem_init);

static int vpr200_mmu_init(void)
{
	l2x0_init((void __iomem *)0x30000000, 0x00030024, 0x00000000);

	return 0;
}
postmmu_initcall(vpr200_mmu_init);

struct imx_nand_platform_data nand_info = {
	.hw_ecc		= 1,
	.flash_bbt	= 1,
};

static struct i2c_board_info i2c0_devices[] = {
	{
		I2C_BOARD_INFO("mc13xxx-i2c", 0x08),
	}, {
		I2C_BOARD_INFO("at24", 0x50),
	},
};

static struct i2c_board_info i2c1_devices[] = {
	{
		I2C_BOARD_INFO("isl22316", 0x2b),
	}, {
		I2C_BOARD_INFO("isl22346", 0x50),
	},
};

static struct gpio_rgb_led vpr_diag_led_v2_v3 = {
	.gpio_r = DIAG_RED_GPIO,
	.gpio_g = DIAG_GREEN_GPIO_V3,
	.gpio_b = DIAG_BLUE_GPIO,
	.active_low = 1,
	.led.name = VPR_DIAG_LED_NAME,
};

static struct gpio_rgb_led vpr_diag_led_v4 = {
	.gpio_r = DIAG_RED_GPIO,
	.gpio_g = DIAG_GREEN_GPIO,
	.gpio_b = DIAG_BLUE_GPIO,
	.active_low = 1,
	.led.name = VPR_DIAG_LED_NAME,
};

static void vpr_backlight_set_V4(int value)
{
	struct isl22316 *isl22316 = isl22316_get();
	int setval = value;

	if (!isl22316) {
		printf("WARN: Couldn't get isl22316\n");
		return;
	}

	if (vpr_board_rev == VPR_BOARD_V2) {
		/* proto boards had inverted wiper */
		setval = 127 - value;
	}

	/* Force the backlight off */
	gpio_direction_output(LCD_LED_ENABLE_GPIO, value == 0 ? 0 : 1);

	isl22316_set_value(isl22316, setval, 0);
}

static void vpr_backlight_set_V5(int value)
{
	struct isl22346 *isl22346 = isl22346_get();
	int setval = value;

	if (!isl22346) {
		printf("WARN: Couldn't get isl22316\n");
		return;
	}

	/* Force the backlight off */
	gpio_direction_output(LCD_LED_ENABLE_GPIO, value == 0 ? 0 : 1);

	isl22346_set_value(isl22346, setval, 0);
}

static void vpr_backlight_set(int value)
{
	if (vpr_board_rev <= VPR_BOARD_V4) {
		vpr_backlight_set_V4(value);
	} else {
		vpr_backlight_set_V5(value);
	}
}

static int vpr_backlight_init_V4(void)
{
	struct isl22316 *isl22316;

	isl22316 = isl22316_get();

	if (!isl22316) {
		printf("WARN: Couldn't get isl22316\n");
		return -EINVAL;
	}

	return 0;
}
static int vpr_backlight_init_V5(void)
{
	struct isl22346 *isl22346;

	isl22346 = isl22346_get();

	if (!isl22346) {
		printf("WARN: Couldn't get isl22316\n");
		return -EINVAL;
	}
	return 0;
}

static void vpr_backlight_init(void)
{
	int initOk = -1;
	if (vpr_board_rev <= VPR_BOARD_V4) {
		initOk = vpr_backlight_init_V4();
	} else {
		initOk = vpr_backlight_init_V5();
	}

	if (initOk < 0) {
		printf("WARN: couldn't init backlight\n");
		return;
	}

	/* force the eeprom value to backlight off */
	vpr_backlight_set(0);
}

/* ------------------------------------------------------------------------- */
static void vpr_cpu_cfg_init(void)
{
	iomux_v3_cfg_t cpu_cfg_pads[] = {
		/* CPU Sys config */
		MX35_PAD_ATA_DATA14__GPIO2_27,
		MX35_PAD_ATA_DATA15__GPIO2_28,
		MX35_PAD_ATA_INTRQ__GPIO2_29,
		MX35_PAD_ATA_BUFF_EN__GPIO2_30,
		MX35_PAD_ATA_DMARQ__GPIO2_31,
	};

	mxc_iomux_v3_setup_multiple_pads(cpu_cfg_pads,
			ARRAY_SIZE(cpu_cfg_pads));
}

static void vpr_board_cfg_init(void)
{
	iomux_v3_cfg_t board_cfg_pads[] = {
		MX35_PAD_ATA_CS1__GPIO2_7,
		MX35_PAD_ATA_DIOR__GPIO2_8,
		MX35_PAD_ATA_DIOW__GPIO2_9,
		MX35_PAD_ATA_DMACK__GPIO2_10,
	};

	mxc_iomux_v3_setup_multiple_pads(board_cfg_pads,
			ARRAY_SIZE(board_cfg_pads));
}

/**
 * Board has 5 lines that are tied high or low to give a system configuration
 * ID. (ie board revision).
 */
static uint32_t vpr_read_cpu_cfg(void)
{
	uint32_t ret = 0;

	ret |= gpio_get_value(CPU_CFG_0_GPIO);
	ret |= gpio_get_value(CPU_CFG_1_GPIO) << 1;
	ret |= gpio_get_value(CPU_CFG_2_GPIO) << 2;
	ret |= gpio_get_value(CPU_CFG_3_GPIO) << 3;
	ret |= gpio_get_value(CPU_CFG_4_GPIO) << 4;

	return ret;
}

static uint32_t vpr_read_board_cfg(void)
{
	uint32_t ret = 0;

	ret |= gpio_get_value(BOARD_CFG_0_GPIO);
	ret |= gpio_get_value(BOARD_CFG_1_GPIO) << 1;
	ret |= gpio_get_value(BOARD_CFG_2_GPIO) << 2;
	ret |= gpio_get_value(BOARD_CFG_3_GPIO) << 3;

	return ret;
}

static int vpr_export_esn(void)
{
	unsigned char buf[2];
	unsigned char esn_lo[3];
	unsigned char esn_hi[3];
	int fd;
	int ret = -EINVAL;

	fd = open("/dev/eeprom0", O_RDONLY);
	if (fd < 0) {
		fd = open("/dev/eeprom", O_RDONLY);
		if (fd < 0) {
			printf("Couldn't open eeprom to get MAC\n");
			goto out;
		}
	}

	if (lseek(fd, 4, SEEK_SET) < 0) {
		printf("Couldn't locate ESN in eeprom\n");
		goto out_close;
	}

	if (read(fd, buf, 2) < 0) {
		printf("Couldn't read eeprom to get MAC\n");
		goto out_close;
	}

	if (buf[0] == 0xff && buf[1] == 0xff) {
		printf("VPR: Invalid ESN\n");
	} else {
		snprintf(esn_lo, 3, "%02x", buf[1]);
		snprintf(esn_hi, 3, "%02x", buf[0]);

		setenv("vpr_esn_lo", esn_lo);
		export("vpr_esn_lo");

		setenv("vpr_esn_hi", esn_hi);
		export("vpr_esn_hi");

		ret = 0;
		armlinux_set_serial((buf[0] & 0xff) << 8 | (buf[1] & 0xff));
	}

out_close:
	close(fd);
out:
	return ret;
}

BAREBOX_MAGICVAR(vpr_esn_lo, "The low byte of the unit's ESN, as ascii hex");
BAREBOX_MAGICVAR(vpr_esn_hi, "The high byte of the unit's ESN, as ascii hex");

/* ------------------------------------------------------------------------- */

/**
 * Enum matching all color component triplets to
 * named identifiers.
 */
enum diag_led_color {
	DIAG_LED_OFF        = 0x1,
	DIAG_LED_BLUE       = 0x0,
	DIAG_LED_GREEN      = 0x3,
	DIAG_LED_AQUA       = 0x2,
	DIAG_LED_RED        = 0x5,
	DIAG_LED_MAGENTA    = 0x4,
	DIAG_LED_YELLOW     = 0x7,
	DIAG_LED_WHITE      = 0x6,
};

/* ------------------------------------------------------------------------- */
enum vpr_button {
	BUTTONNONE = 0,
	BUTTON1 = 0x01,
	BUTTON2 = 0x02,
	BUTTON3 = 0x04,
	BUTTON4 = 0x08,
	BUTTON5 = 0x10,
	BUTTON6 = 0x20,
	BUTTON7 = 0x40,
	BUTTON8 = 0x80
};

static void vpr_button_init(void)
{
	gpio_direction_input(BUTTON1_GPIO);
	gpio_direction_input(BUTTON2_GPIO);
	gpio_direction_input(BUTTON3_GPIO);
	gpio_direction_input(BUTTON4_GPIO);
	gpio_direction_input(BUTTON5_GPIO);
	gpio_direction_input(BUTTON6_GPIO);
	gpio_direction_input(BUTTON7_GPIO);
	gpio_direction_input(BUTTON8_GPIO);
}

static int vpr_button_state(void)
{
	uint32_t ret = 0;

	ret |= (!gpio_get_value(BUTTON1_GPIO));
	ret |= (!gpio_get_value(BUTTON2_GPIO)) << 1;
	ret |= (!gpio_get_value(BUTTON3_GPIO)) << 2;
	ret |= (!gpio_get_value(BUTTON4_GPIO)) << 3;
	ret |= (!gpio_get_value(BUTTON5_GPIO)) << 4;
	ret |= (!gpio_get_value(BUTTON6_GPIO)) << 5;
	ret |= (!gpio_get_value(BUTTON7_GPIO)) << 6;
	ret |= (!gpio_get_value(BUTTON8_GPIO)) << 7;

	return ret;
}

/* ------------------------------------------------------------------------- */
static void vpr_buzzer_init(void)
{
	if (vpr_board_rev >= VPR_BOARD_V3)
		gpio_direction_output(BUZZER_GPIO, 0);
	else
		gpio_direction_output(BUZZER_GPIO_V2, 0);
}

static void vpr_buzzer_set(int on)
{
	if (vpr_board_rev >= VPR_BOARD_V3)
		gpio_set_value(BUZZER_GPIO, !!on);
	else
		gpio_set_value(BUZZER_GPIO_V2, !!on);
}

/* ------------------------------------------------------------------------- */
static void vpr_gsm_init(void)
{
	if (vpr_board_rev >= VPR_BOARD_V3)
		gpio_direction_output(GSM_GPIO, 0);
	else
		printf("GSM power not supported on this main board version\n");
}

static void vpr_gsm_set(int on)
{
	if (vpr_board_rev >= VPR_BOARD_V3)
		gpio_set_value(GSM_GPIO, !!on);
	else
		printf("GSM power not supported on this main board version\n");
}

/* ------------------------------------------------------------------------- */
static void vpr_bp_init(void)
{
	iomux_v3_cfg_t bp_pads[] = {
		MX35_PAD_ATA_DATA4__GPIO2_17,
		MX35_PAD_ATA_DATA5__GPIO2_18,
	};

	mxc_iomux_v3_setup_multiple_pads(bp_pads, ARRAY_SIZE(bp_pads));

	gpio_direction_output(BP_RESET_GPIO, 1);
	gpio_direction_input(BP_EOC_GPIO);
}

static void vpr_rfprog_init(void)
{
	iomux_v3_cfg_t rfprog_pads[] = {
		MX35_PAD_ATA_IORDY__GPIO2_12,
		MX35_PAD_ATA_DATA0__GPIO2_13,
		MX35_PAD_ATA_DATA1__GPIO2_14,
		MX35_PAD_ATA_DATA2__GPIO2_15,
		MX35_PAD_ATA_DATA3__GPIO2_16,
	};

	mxc_iomux_v3_setup_multiple_pads(rfprog_pads, ARRAY_SIZE(rfprog_pads));

	gpio_direction_output(RF_RESET_GPIO, 1);
	gpio_direction_output(RF_TDI_GPIO, 1);
	gpio_direction_output(RF_TCK_GPIO, 1);
	gpio_direction_output(RF_TMS_GPIO, 1);
	gpio_direction_input(RF_TDO_GPIO);
}

/* ------------------------------------------------------------------------- */

static struct esdhc_platform_data vpr_esdhc0 = {
	.cd_gpio = GPIO_SD_CD,
	.cd_type = ESDHC_CD_GPIO,
	.wp_gpio = GPIO_SD_WP,
	.wp_type = ESDHC_WP_GPIO,
};

/*
 * Board initialization order:
 *  core_initcall
 *  postcore_initcall
 *  console_initcall
 *  postconsole_initcall
 *  coredevice_initcall
 *  fs_initcall
 *  device_initcall
 *  late_initcall
 */

static int vpr_devices_init(void)
{
#define MAX_BOOTSRC_MSG 80
	uint32_t reg;
	char bootsrc_msg[MAX_BOOTSRC_MSG+1] = {0,};
	struct led *diagled;

	/* take care of WDT PDE */
	writew(0x0000, IMX_WDT_BASE + 8);

	/* CS0: Nor Flash */
	writel(0x0000cf03, CSCR_U(0));
	writel(0x10000d03, CSCR_L(0));
	writel(0x00720900, CSCR_A(0));

	reg = readl(IMX_CCM_BASE + CCM_RCSR);
	/* some fuses provide us vital information about connected hardware */
	if (reg & 0x20000000)
		nand_info.width = 2;	/* 16 bit */
	else
		nand_info.width = 1;	/* 8 bit */

	printf("reset source: ");
	switch (reg & 0x0f) {
	case 0x8:
		printf("Watchdog\n");
		break;
	case 0x4:
		printf("External\n");
		break;
	case 0x2:
		printf("JTAG\n");
		break;
	case 0x0:
		printf("POR\n");
		break;
	default:
		printf("Unknown, %x\n", reg & 0x0f);
		break;
	}

	diagled = led_by_name(VPR_DIAG_LED_NAME);

	/* This platform supports NOR, NAND and SD */
	imx35_add_nand(&nand_info);
	add_cfi_flash_device(-1, IMX_CS0_BASE, 64 * 1024 * 1024, 0);
	imx35_add_mmc0(&vpr_esdhc0);

	switch ((reg >> 25) & 0x3) {
	case 0x03:		/* SD/MMC is the source */
		led_set(diagled, DIAG_LED_MAGENTA);
		snprintf(bootsrc_msg, MAX_BOOTSRC_MSG, "SD boot: ");
		devfs_add_partition("disk0", 0x00000, 0x80000, DEVFS_PARTITION_FIXED, "self0");
		devfs_add_partition("disk0", 0x80000, 0x80000, DEVFS_PARTITION_FIXED, "env0");
		protect_file("/dev/self0", 1);
		protect_file("/dev/env0", 1);
		break;

	case 0x01:		/* NAND is the source */
		devfs_add_partition("nand0", 0x00000, 0x40000, DEVFS_PARTITION_FIXED, "self_raw");
		dev_add_bb_dev("self_raw", "self0");
		devfs_add_partition("nand0", 0x40000, 0x80000, DEVFS_PARTITION_FIXED, "env_raw");
		dev_add_bb_dev("env_raw", "env0");
		break;

	case 0x00:		/* NOR is the source */
		led_set(diagled, DIAG_LED_AQUA);
		snprintf(bootsrc_msg, MAX_BOOTSRC_MSG, "NOR boot: ");
		devfs_add_partition("nor0", 0x00000, 0x80000, DEVFS_PARTITION_FIXED, "self0");
		devfs_add_partition("nor0", 0x80000, 0x80000, DEVFS_PARTITION_FIXED, "env0");
		protect_file("/dev/self0", 1);
		protect_file("/dev/env0", 1);
		break;
	default:
		printf("WARN: Unrecognized boot source.\n");
		break;
	}

	/* delay printing message so protect file doesn't get in the middle */
	printf(bootsrc_msg);

	switch ((reg >> 10) & 0x3) {
	case 0x03:
		printf("Bootstrap mode\n");
		break;
	case 0x02:
		printf("External\n");
		break;
	case 0x00:
		printf("Internal\n");
		break;
	default:
		break;
	}

	/* Init the i2c before the FEC, so that the MAC address can be read */
	i2c_register_board_info(0, i2c0_devices, ARRAY_SIZE(i2c0_devices));
	i2c_register_board_info(1, i2c1_devices, ARRAY_SIZE(i2c1_devices));

	imx35_add_i2c0(NULL);
	imx35_add_i2c1(NULL);

	vpr_setup_ethaddr();
	imx35_add_fec(&fec_info);

	vpr_bp_init();
	vpr_rfprog_init();
	vpr_button_init();
	vpr_buzzer_init();

	armlinux_set_bootparams((void *)0x80000100);
	armlinux_set_architecture(MACH_TYPE_VPR200);

	armlinux_set_revision( ((vpr_cpu_rev & 0xff) << 8) |
				(vpr_board_rev & 0xff));

	vpr_export_esn();

	return 0;
}

device_initcall(vpr_devices_init);

static iomux_v3_cfg_t vpr_pads[] = {
	/* PMIC int */
	MX35_PAD_GPIO2_0__GPIO2_0,
	/* VSD_EN */
	MX35_PAD_CSI_PIXCLK__GPIO1_31,
	/* FEC */
	MX35_PAD_FEC_RX_DV__FEC_RX_DV,
	MX35_PAD_FEC_COL__FEC_COL,
	MX35_PAD_FEC_TX_EN__FEC_TX_EN,
	MX35_PAD_FEC_MDC__FEC_MDC,
	MX35_PAD_FEC_MDIO__FEC_MDIO,
	MX35_PAD_FEC_TX_ERR__FEC_TX_ERR,
	MX35_PAD_FEC_RX_ERR__FEC_RX_ERR,
	MX35_PAD_FEC_CRS__FEC_CRS,
	MX35_PAD_FEC_RDATA0__FEC_RDATA_0,
	MX35_PAD_FEC_TDATA0__FEC_TDATA_0,
	MX35_PAD_FEC_RDATA1__FEC_RDATA_1,
	MX35_PAD_FEC_TDATA1__FEC_TDATA_1,
	MX35_PAD_FEC_RDATA2__FEC_RDATA_2,
	MX35_PAD_FEC_TDATA2__FEC_TDATA_2,
	MX35_PAD_FEC_RDATA3__FEC_RDATA_3,
	MX35_PAD_FEC_TDATA3__FEC_TDATA_3,
	/* FEC reset */
	MX35_PAD_GPIO3_0__GPIO3_0,
	/* OLD -- MainBoard V1 - 3v3 reg enable */
	/* CPU board V2 - 5V reg enable */
	MX35_PAD_D3_REV__GPIO1_3,
	MX35_PAD_TX0__GPIO1_15,
	/* UART 1*/
	MX35_PAD_RXD1__UART1_RXD_MUX,
	MX35_PAD_TXD1__UART1_TXD_MUX,
	MX35_PAD_RTS1__UART1_RTS,
	MX35_PAD_CTS1__UART1_CTS,
	/* UART 2 */
	MX35_PAD_RXD2__UART2_RXD_MUX,
	MX35_PAD_TXD2__UART2_TXD_MUX,
	MX35_PAD_RTS2__UART2_RTS,
	MX35_PAD_CTS2__UART2_CTS,
	/* I2C1 */
	MX35_PAD_I2C1_CLK__I2C1_SCL,
	MX35_PAD_I2C1_DAT__I2C1_SDA,
	/* I2C2 */
	MX35_PAD_I2C2_CLK__I2C2_SCL,
	MX35_PAD_I2C2_DAT__I2C2_SDA,
	/* LCD enable */
	MX35_PAD_D3_VSYNC__GPIO1_2,
	MX35_PAD_ATA_CS1__GPIO2_7,
	/* SDCARD */
	MX35_PAD_SD1_CMD__ESDHC1_CMD,
	MX35_PAD_SD1_CLK__ESDHC1_CLK,
	MX35_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX35_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX35_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX35_PAD_SD1_DATA3__ESDHC1_DAT3,
	MX35_PAD_ATA_DA1__GPIO3_1,
	MX35_PAD_ATA_DA2__GPIO3_2,
	/* Display */
	MX35_PAD_LD0__IPU_DISPB_DAT_0,
	MX35_PAD_LD1__IPU_DISPB_DAT_1,
	MX35_PAD_LD2__IPU_DISPB_DAT_2,
	MX35_PAD_LD3__IPU_DISPB_DAT_3,
	MX35_PAD_LD4__IPU_DISPB_DAT_4,
	MX35_PAD_LD5__IPU_DISPB_DAT_5,
	MX35_PAD_LD6__IPU_DISPB_DAT_6,
	MX35_PAD_LD7__IPU_DISPB_DAT_7,
	MX35_PAD_LD8__IPU_DISPB_DAT_8,
	MX35_PAD_LD9__IPU_DISPB_DAT_9,
	MX35_PAD_LD10__IPU_DISPB_DAT_10,
	MX35_PAD_LD11__IPU_DISPB_DAT_11,
	MX35_PAD_LD12__IPU_DISPB_DAT_12,
	MX35_PAD_LD13__IPU_DISPB_DAT_13,
	MX35_PAD_LD14__IPU_DISPB_DAT_14,
	MX35_PAD_LD15__IPU_DISPB_DAT_15,
	MX35_PAD_LD16__IPU_DISPB_DAT_16,
	MX35_PAD_LD17__IPU_DISPB_DAT_17,
	MX35_PAD_D3_FPSHIFT__IPU_DISPB_D3_CLK,
	MX35_PAD_D3_DRDY__IPU_DISPB_D3_DRDY,
	MX35_PAD_CONTRAST__IPU_DISPB_CONTR,
	/* Buttons */
	MX35_PAD_SCKR__GPIO1_4,
	MX35_PAD_COMPARE__GPIO1_5,
	MX35_PAD_SCKT__GPIO1_7,
	MX35_PAD_FST__GPIO1_8,
	MX35_PAD_HCKT__GPIO1_9,
	MX35_PAD_TX5_RX0__GPIO1_10,
	MX35_PAD_TX4_RX1__GPIO1_11,
	MX35_PAD_TX3_RX2__GPIO1_12,
	/* Buzzer (PWMO)*/
	MX35_PAD_GPIO1_1__GPIO1_1,
	/* GSM power or V1 buzzer */
	MX35_PAD_TX2_RX3__GPIO1_13,
	/* GSM board*/
	MX35_PAD_MLB_CLK__GPIO3_3,
	MX35_PAD_MLB_DAT__GPIO3_4,
	MX35_PAD_MLB_SIG__GPIO3_5,
	MX35_PAD_CSI_MCLK__GPIO1_28,
	MX35_PAD_TX2_RX3__GPIO1_13,
	/* Speaker (V2, V3) or Diag LED, green */
	MX35_PAD_TX1__GPIO1_14,
	/* Diag LED */
	MX35_PAD_USBOTG_PWR__GPIO3_14,
	MX35_PAD_D3_HSYNC__GPIO3_30,
	/* WDOG */
	MX35_PAD_WDOG_RST__WDOG_WDOG_B,
	/* MRFC */
	MX35_PAD_ATA_IORDY__GPIO2_12,
	MX35_PAD_ATA_DATA0__GPIO2_13,
	MX35_PAD_ATA_DATA1__GPIO2_14,
	MX35_PAD_ATA_DATA2__GPIO2_15,
	MX35_PAD_ATA_DATA3__GPIO2_16,
	/* Charge Shunt GPIO*/
	MX35_PAD_SD2_CLK__GPIO2_1,
	MX35_PAD_SD2_DATA0__GPIO2_2,
	MX35_PAD_SD2_DATA1__GPIO2_3,
	/* Spare GPIO */
	MX35_PAD_SD2_DATA2__GPIO2_4,
	MX35_PAD_SD2_DATA3__GPIO2_5,
	MX35_PAD_ATA_CS0__GPIO2_6,
};

static iomux_v3_cfg_t vpr_pads_v2_v3[] = {
	MX35_PAD_USBOTG_OC__GPIO3_15,
};

static iomux_v3_cfg_t vpr_pads_v4[] = {
	MX35_PAD_USBOTG_OC__USB_TOP_USBOTG_OC,
};

static int vpr_console_init(void)
{
	vpr_cpu_cfg_init();
	vpr_cpu_rev = vpr_read_cpu_cfg();

	vpr_board_cfg_init();
	vpr_board_rev = vpr_read_board_cfg();

	mxc_iomux_v3_setup_multiple_pads(vpr_pads, ARRAY_SIZE(vpr_pads));

	if (vpr_cpu_rev == VPR_CPU_V2 || vpr_cpu_rev == VPR_CPU_V3) {
		mxc_iomux_v3_setup_multiple_pads(vpr_pads_v2_v3,
				ARRAY_SIZE(vpr_pads_v2_v3));
		led_gpio_rgb_register(&vpr_diag_led_v2_v3);
	} else if (vpr_cpu_rev == VPR_CPU_V4) {
		mxc_iomux_v3_setup_multiple_pads(vpr_pads_v4,
				ARRAY_SIZE(vpr_pads_v4));
		led_gpio_rgb_register(&vpr_diag_led_v4);
	}

	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_RED);

	/* get the GSM init done early as possible */
	vpr_gsm_init();

	/* Force the backlight off */
	gpio_direction_output(LCD_LED_ENABLE_GPIO, 0);
	gpio_direction_output(LCD_PWR_GPIO, 1);

	imx35_add_uart0();
	return 0;
}

console_initcall(vpr_console_init);

static int vpr_post_console_init(void)
{
	return 0;
}

postconsole_initcall(vpr_post_console_init);

static int vpr_core_init(void)
{
	u32 reg;

	/* enable clock for SDHC1, I2C[1] and FEC */
	reg = readl(IMX_CCM_BASE + CCM_CGR1);
	reg |= 0x3 << CCM_CGR1_FEC_SHIFT;
	reg |= 0x3 << CCM_CGR1_I2C1_SHIFT;
	reg |= 0x3 << CCM_CGR1_SDHC1_SHIFT;
	reg = writel(reg, IMX_CCM_BASE + CCM_CGR1);

	/* AIPS setup - Only setup MPROTx regs. The PACR defaults are good.*/
	/*
	 * Set all MPROTx to be non-bufferable, trusted for R/W,
	 * not forced to user-mode.
	 */
	writel(0x77777777, IMX_AIPS1_BASE);
	writel(0x77777777, IMX_AIPS1_BASE + 0x4);
	writel(0x77777777, IMX_AIPS2_BASE);
	writel(0x77777777, IMX_AIPS2_BASE + 0x4);

	/*
	 * Clear the on and off peripheral modules Supervisor Protect bit
	 * for SDMA to access them. Did not change the AIPS control registers
	 * (offset 0x20) access type
	 */
	writel(0x0, IMX_AIPS1_BASE + 0x40);
	writel(0x0, IMX_AIPS1_BASE + 0x44);
	writel(0x0, IMX_AIPS1_BASE + 0x48);
	writel(0x0, IMX_AIPS1_BASE + 0x4C);
	reg = readl(IMX_AIPS1_BASE + 0x50);
	reg &= 0x00FFFFFF;
	writel(reg, IMX_AIPS1_BASE + 0x50);

	writel(0x0, IMX_AIPS2_BASE + 0x40);
	writel(0x0, IMX_AIPS2_BASE + 0x44);
	writel(0x0, IMX_AIPS2_BASE + 0x48);
	writel(0x0, IMX_AIPS2_BASE + 0x4C);
	reg = readl(IMX_AIPS2_BASE + 0x50);
	reg &= 0x00FFFFFF;
	writel(reg, IMX_AIPS2_BASE + 0x50);

	/* MAX (Multi-Layer AHB Crossbar Switch) setup */

	/* MPR - priority is M4 > M2 > M3 > M5 > M0 > M1 */
#define MAX_PARAM1 0x00302154
	writel(MAX_PARAM1, IMX_MAX_BASE + 0x000); /* for S0 */
	writel(MAX_PARAM1, IMX_MAX_BASE + 0x100); /* for S1 */
	writel(MAX_PARAM1, IMX_MAX_BASE + 0x200); /* for S2 */
	writel(MAX_PARAM1, IMX_MAX_BASE + 0x300); /* for S3 */
	writel(MAX_PARAM1, IMX_MAX_BASE + 0x400); /* for S4 */

	/* SGPCR - always park on last master */
	writel(0x10, IMX_MAX_BASE + 0x10);	/* for S0 */
	writel(0x10, IMX_MAX_BASE + 0x110);	/* for S1 */
	writel(0x10, IMX_MAX_BASE + 0x210);	/* for S2 */
	writel(0x10, IMX_MAX_BASE + 0x310);	/* for S3 */
	writel(0x10, IMX_MAX_BASE + 0x410);	/* for S4 */

	/* MGPCR - restore default values */
	writel(0x0, IMX_MAX_BASE + 0x800);	/* for M0 */
	writel(0x0, IMX_MAX_BASE + 0x900);	/* for M1 */
	writel(0x0, IMX_MAX_BASE + 0xa00);	/* for M2 */
	writel(0x0, IMX_MAX_BASE + 0xb00);	/* for M3 */
	writel(0x0, IMX_MAX_BASE + 0xc00);	/* for M4 */
	writel(0x0, IMX_MAX_BASE + 0xd00);	/* for M5 */

	/*
	 * M3IF Control Register (M3IFCTL)
	 * MRRP[0] = L2CC0 not on priority list (0 << 0)	= 0x00000000
	 * MRRP[1] = MAX1 not on priority list (0 << 0)		= 0x00000000
	 * MRRP[2] = L2CC1 not on priority list (0 << 0)	= 0x00000000
	 * MRRP[3] = USB  not on priority list (0 << 0)		= 0x00000000
	 * MRRP[4] = SDMA not on priority list (0 << 0)		= 0x00000000
	 * MRRP[5] = GPU not on priority list (0 << 0)		= 0x00000000
	 * MRRP[6] = IPU1 on priority list (1 << 6)		= 0x00000040
	 * MRRP[7] = IPU2 not on priority list (0 << 0)		= 0x00000000
	 *                                                       ------------
	 *                                                        0x00000040
	 */
	writel(0x40, IMX_M3IF_BASE);

	return 0;
}
core_initcall(vpr_core_init);

/* -------------------------------------------------------------------------*/
static int vpr_pmic_init(struct mc13xxx *mc13xxx)
{
	int err = 0;
	unsigned int mask = 0;
	unsigned int val = 0;

	/* VGEN2[2:0] = b111 --> output to 3.15V */
	mc13xxx_set_bits(mc13xxx, MC13892_REG_SETTING_0, 0x7 << 6, 0x7 << 6);

	val = 0;
	/* VCHRG[2:0] = 0b011, Charge reg output voltage 4.200 */
	val |= 0x3;
	/* ICHRG[3:0] = 0b1101, 1200mA charger current */
	val |= 0xd << 3;
	/* Disable power limiting,  PLIM[1:1] = 0b11, Power limit 1100mW */
	val |= 3 << 15;
	val |= 1 << 17;
	/* TREN */
	val |= 1 << 7;
	/* CYCLB = 0 */
	val &= ~(1<<22);
	/* THCHKB = 1, disable thermistor check*/
	val |= (1 << 9);
	/* Enable setting of V I */
	val |= 1 << 23;
	err |= mc13xxx_reg_write(mc13xxx, MC13892_REG_CHARGE, val);

	/* global reset enable */
	mc13xxx_set_bits(mc13xxx, MC13892_REG_POWER_CTL0, 0x1 << 7, 0x0);

	/* pwron1 reset enable */
	val = 0;
	mask = 0;
	val |= (0x1 << 1);
	mask |= (0x1 << 3) | (0x1 << 2) | (0x1 << 1);
	val |= (0x3 << 4); /* debounce pwron1 to 750 ms */
	mask |= (0x3 < 4);
	mc13xxx_set_bits(mc13xxx, MC13892_REG_POWER_CTL2, mask, val);

	/* Enable GPIO shunt, active low */
	gpio_direction_output(GPIO_SHUNT_ENABLE, 0);

	return err;
}

static int vpr_fec_init_v2(struct mc13xxx *mc13xxx)
{
	int err = 0;

	gpio_direction_output(FEC_RESET_GPIO, 1);

	/*turn on the FEC power supply */
	/* VGEN1[1:0] = 0b11*/
	err |= mc13xxx_set_bits(mc13xxx, MC13892_REG_SETTING_0, 0x03, 0x03);
	/* VGEN1EN = 1, VGEN1STBY = 0, VGEN1MODE = 0 */
	err |= mc13xxx_set_bits(mc13xxx, MC13892_REG_MODE_0, 0x07, 0x01);
	if (err) {
		dev_err(&mc13xxx->client->dev, "Init sequence failed!\n");
	}

	mdelay(25);
	gpio_direction_output(FEC_RESET_GPIO, 0);

	mdelay(10);
	gpio_set_value(FEC_RESET_GPIO, 1);

	return err;
}

static void vpr_regs_init(void)
{
	gpio_direction_output(MB3V3REG_ENABLE_GPIO, 1);
	gpio_direction_output(MB5VREG_ENABLE_GPIO, 1);
}

static int vpr_final_init(void)
{
	struct mc13xxx *mc13xxx;

	mc13xxx = mc13xxx_get();
	if (!mc13xxx) {
		printf("FAILED to get mc13xxx handle!\n");
		return 0;
	}

	printf("VPR CPU board version 0x%02x.\n", vpr_cpu_rev);
	switch (vpr_cpu_rev) {
	case VPR_CPU_V2:
	case VPR_CPU_V3:
	case VPR_CPU_V4:
		printf("VPR Main board version 0x%02x.\n", vpr_board_rev);
		/*
		 * init the pmic first so the charge current is increased
		 * before any ancilliaries are powered up.
		 */
		vpr_pmic_init(mc13xxx);
		vpr_fec_init_v2(mc13xxx);
		vpr_regs_init();
		vpr_backlight_init();
		break;
	default:
		printf("Unknown revision 0x%02x.\n", vpr_cpu_rev);
		return 0;
	}

	return 0;
}

late_initcall(vpr_final_init);

#ifdef CONFIG_NAND_IMX_BOOT
void __bare_init nand_boot(void)
{
	/*
	 * The driver is able to detect NAND's pagesize by CPU internal
	 * fuses or external pull ups. But not the blocksize...
	 */
	imx_nand_load_image((void *)TEXT_BASE, 256 * 1024);
}
#endif

/* ------------------------------------------------------------------------ */

static int do_button(int argc, char *argv[])
{
	int opt;
	uint64_t start;
	ulong timeout = 0;
	uint32_t bstate;
	uint32_t mask = 0;
	uint32_t ignore = 0;
	int bnum = 0;
	int printstate = 0;

	while ((opt = getopt(argc, argv, "i:m:t:n:p")) > 0) {
		switch (opt) {
		case 'i':
			ignore = simple_strtoul(optarg, NULL, 0);
			break;
		case 'm':
			mask = simple_strtoul(optarg, NULL, 0);
			break;
		case 't':
			timeout = simple_strtol(optarg, NULL, 0);
			break;
		case 'n':
			bnum = simple_strtol(optarg, NULL, 0);
			break;
		case 'p':
			printstate = 1;
			break;
		}
	}

	start = get_time_ns();

	do {
		bstate = vpr_button_state();
		if (bstate || !timeout) {
			if (printstate)
				printf("0x%02x\n", bstate);

			if (mask)
				return ((bstate & ~ignore) == (mask & ~ignore))
					? 0 : 1;

			if (bnum)
				return (bstate & (0x01 << (bnum - 1))) ? 0 : 1;
		}
		if (ctrlc())
			return -EINTR;
	} while (timeout && !is_timeout(start, timeout * SECOND));

	return 1;
}

BAREBOX_CMD_HELP_START(button)
BAREBOX_CMD_HELP_USAGE("button [-t]\n")
BAREBOX_CMD_HELP_SHORT("Test whether a button (or buttons) is pressed.\n")
BAREBOX_CMD_HELP_OPT("-t <seconds>", "time in seconds to wait. "
			"(default 0 - no wait).\n")
BAREBOX_CMD_HELP_OPT("-m <mask>", "mask to match for multiple buttons. "
			"(takes precedence\n)")
BAREBOX_CMD_HELP_OPT("-n <num>", "button number. (starts from 1)\n")
BAREBOX_CMD_HELP_OPT("-p", "print the state of the buttons\n");
BAREBOX_CMD_HELP_END

BAREBOX_CMD_START(button)
	.cmd	= do_button,
	.usage	= "Check whether a button is pressed.",
	BAREBOX_CMD_HELP(cmd_button_help)
BAREBOX_CMD_END

/* ------------------------------------------------------------------------ */

static int do_buzzer(int argc, char *argv[])
{
	int opt;
	uint64_t start;
	ulong durationms = 200;
	ulong freq = 440;
	ulong delay = 120;

	while ((opt = getopt(argc, argv, "d:f:")) > 0) {
		switch (opt) {
		case 'd':
			durationms = simple_strtol(optarg, NULL, 0);
			break;
		case 'f':
			freq = simple_strtol(optarg, NULL, 0);
			if (freq == 0)
				freq = 1;
			break;
		}
	}

	delay = 1000000 / freq / 2;
	vpr_buzzer_set(1);

	start = get_time_ns();
	while (!is_timeout(start, durationms * MSECOND)) {

		vpr_buzzer_set(1);
		udelay(delay);
		vpr_buzzer_set(0);
		udelay(delay);

		if (ctrlc())
			break;
	}

	vpr_buzzer_set(0);
	return 0;
}

BAREBOX_CMD_HELP_START(buzzer)
BAREBOX_CMD_HELP_USAGE("buzzer [-d]\n")
BAREBOX_CMD_HELP_SHORT("Buzzer beep.\n")
BAREBOX_CMD_HELP_OPT("-d <ms>", "time in milli-seconds to stay on. (200ms)\n")
BAREBOX_CMD_HELP_END

BAREBOX_CMD_START(buzzer)
	.cmd	= do_buzzer,
	.usage	= "Buzzer beep",
	BAREBOX_CMD_HELP(cmd_buzzer_help)
BAREBOX_CMD_END

static int do_gsmpwr(int argc, char *argv[])
{
	int opt;
	ulong set = 0;

	while ((opt = getopt(argc, argv, "s:")) > 0) {
		switch (opt) {
		case 's':
			set = simple_strtoul(optarg, NULL, 0);
			break;
		}
	}

	printf("GSM power: %s\n", set ? "on" : "off");
	vpr_gsm_set(set);
	return 0;
}

BAREBOX_CMD_START(gsmpwr)
	.cmd = do_gsmpwr,
	.usage = "Turn GSM power on/off",
BAREBOX_CMD_END

static void dump_binary(unsigned int val)
{
	int ii;

	for (ii=24; ii > 0; --ii) {
		if (!(ii % 4))
			printf(" ");
		printf("%1d", (val >> (ii-1)) & 0x01);
	}
}

static int do_mc13xxx_dump(int argc, char* argv[])
{
	int regnum;
	unsigned int val;

	struct mc13xxx *mc13xxx = mc13xxx_get();

	for(regnum = 0; regnum < 55; ++regnum) {
		mc13xxx_reg_read(mc13xxx, regnum, &val);
		printf("%d\t %02x %02x %02x \t", regnum,
				(val >> 16) & 0xff,
				(val >> 8) & 0xff,
				(val >> 0) & 0xff );
		dump_binary(val);
		printf("\n");
	}

	return 0;
}

BAREBOX_CMD_START(mc13xxx_dump)
	.cmd	= do_mc13xxx_dump,
	.usage  = "Dump mc13xxx registers",
BAREBOX_CMD_END

static int do_vprregs(int argc, char* argv[])
{
	int enable_3v3 = -1;
	int enable_5v = -1;
	int opt;

	while ((opt = getopt(argc, argv, "t:f:")) > 0) {
		switch (opt) {
		case 't':
			enable_3v3 = simple_strtol(optarg, NULL, 0);
			break;
		case 'f':
			enable_5v = simple_strtol(optarg, NULL, 0);
			break;
		}
	}

	if (enable_3v3 >= 0) {
		gpio_direction_output(MB3V3REG_ENABLE_GPIO, !!enable_3v3);
		printf("Enable 3v3: %d\n", enable_3v3);
	}

	if (enable_5v >= 0) {
		gpio_direction_output(MB5VREG_ENABLE_GPIO, !!enable_5v);
		printf("Enable 5v : %d\n", enable_5v);
	}
	return 0;
}

BAREBOX_CMD_START(vprreg)
	.cmd	= do_vprregs,
	.usage  = "Enable regulators",
BAREBOX_CMD_END

static int do_isl22316(int argc, char *argv[])
{
	int opt;
	int value = -1;
	int save = 0;
	int mode = 0;
	int ret;
	struct isl22316 *isl22316;

	while ((opt = getopt(argc, argv, "gsv:e")) > 0) {
		switch (opt) {
		case 'v':
			value = simple_strtol(optarg, NULL, 0);
			break;
		case 'e':
			save = 1;
			break;
		case 'g':
			mode = 0;
			break;
		case 's':
			mode = 1;
			break;
		}
	}

	isl22316 = isl22316_get();

	if (!isl22316) {
		printf("WARN: couldn't get isl22316\n");
		return -EINVAL;
	}

	if (mode > 0) {
		if (value < 0)
			return -EINVAL;

		ret = isl22316_set_value(isl22316, value, save);
	} else {
		u8 val;
		ret = isl22316_get_value(isl22316, &val, save);

		printf("%d\n", val);
	}

	return ret;
}

BAREBOX_CMD_START(isl22316)
	.cmd	= do_isl22316,
	.usage  = "Set isl22316 device value",
BAREBOX_CMD_END

static int do_vprbacklight(int argc, char* argv[])
{
	int setval = -1;
	int opt;

	while ((opt = getopt(argc, argv, "s:")) > 0) {
		switch (opt) {
		case 's':
			setval = simple_strtol(optarg, NULL, 0);
			break;
		}
	}

	if (setval >= 0) {
		vpr_backlight_set(setval);
		return 0;
	}

	return -EINVAL;
}

BAREBOX_CMD_START(vprbacklight)
	.cmd	= do_vprbacklight,
	.usage  = "Set backlight value",
BAREBOX_CMD_END

#define MC13XXX_IRQSENS0_BPONS (1 << 12)

static int do_battery_check(int argc, char *argv[])
{
	unsigned int val;
	struct mc13xxx *mc13xxx = mc13xxx_get();

	if (mc13xxx_reg_read(mc13xxx, MC13892_REG_INT_SENSE0, &val)) {
		return -EINVAL;
	}

	if (!(val & MC13XXX_IRQSENS0_BPONS)) {
		printf("Battery: low\n");
		return 1;
	} else {
		printf("Battery: OK\n");
		return 0;
	}

	return -EINVAL;
}

BAREBOX_CMD_START(battery_check)
	.cmd	= do_battery_check,
	.usage  = "Check that the battery voltage is above a minimum"
		  " acceptable level",
BAREBOX_CMD_END
