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
#include <poller.h>

/* ------------------------------------------------------------------------- */
/* Board revs for the VPR CPU */
#define VPR_CPU_V2	0x11
#define VPR_CPU_V3	0x19
#define VPR_CPU_V4	0x1a

/* Main board revs. V1 board had no rev ID lines */
#define VPR_BOARD_V2	0x1
#define VPR_BOARD_V3	0x2

#define VPR_DIAG_LED_NAME "diag"

#define LCD_PWR_GPIO		IMX_GPIO_NR(1, 2)
#define LCD_LED_ENABLE_GPIO	IMX_GPIO_NR(2, 7)

#define FEC_ENABLE_GPIO		IMX_GPIO_NR(1, 3)	/* prototype board */

#define MB3V3REG_ENABLE_GPIO	IMX_GPIO_NR(1, 15)	/* V2 CPU board */
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
/* ------------------------------------------------------------------------- */

static uint32_t vpr_cpu_rev;

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


	/* This platform supports NOR, NAND and SD */
	imx35_add_nand(&nand_info);
	add_cfi_flash_device(-1, IMX_CS0_BASE, 64 * 1024 * 1024, 0);;
	imx35_add_mmc0(NULL);

	switch ((reg >> 25) & 0x3) {
	case 0x03:		/* SD/MMC is the source */
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

	i2c_register_board_info(0, i2c0_devices, ARRAY_SIZE(i2c0_devices));

	imx35_add_i2c0(NULL);

	return 0;
}

device_initcall(vpr_devices_init);

static void wdt_feed_func(struct poller_struct *poller)
{
	writew(0x5555, IMX_WDT_BASE + 2);
	writew(0xaaaa, IMX_WDT_BASE + 2);
}

static struct poller_struct wdt_poller = {
	.func = wdt_feed_func,
};

int wdt_feed_init(void)
{
	return poller_register(&wdt_poller);
}
late_initcall(wdt_feed_init);


static iomux_v3_cfg_t vpr_pads[] = {
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
	/* Speaker */
	MX35_PAD_TX1__GPIO1_14,
	/* Diag LED */
	MX35_PAD_USBOTG_PWR__GPIO3_14,
	MX35_PAD_USBOTG_OC__GPIO3_15,
	MX35_PAD_D3_HSYNC__GPIO3_30,
	/* WDOG */
	MX35_PAD_WDOG_RST__WDOG_WDOG_B,
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

	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_YELLOW);

	imx35_add_uart0();
	return 0;
}

console_initcall(vpr_console_init);

static int vpr_post_console_init(void)
{
	gpio_direction_output(LCD_LED_ENABLE_GPIO, 0);

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
	mask |= 0x7;
	/* ICHRG[3:0], 1200mA charger current */
	val |= 0xd << 3;
	mask |= 0xf << 3;
	/* PLIM[1:1] = 0b11, Power limit 1100mW */
	val |= 3 << 15;
	mask |= 0x3 << 15;
	/* Enable setting of V I */
	val |= 1 << 23;
	mask |= 0x1 << 23;
	err |= mc13xxx_set_bits(mc13xxx, MC13892_REG_CHARGE, mask, val);

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

	return err;
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
		/*
		 * init the pmic first so the charge current is increased
		 * before any ancilliaries are powered up.
		 */
		vpr_pmic_init(mc13xxx);
		vpr_fec_init_v2(mc13xxx);
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
static int do_vpr200_sdraminit(int argc, char* argv[])
{
	writel(0x00000304, 0xB8001010);
	/* LPDDR delay line soft reset */
	writel(0x0000030C, 0xB8001010);
	/* bank1 */
	writel(0x007ffc2f, 0xB8001004);
	writel(0x92220000, 0xB8001000);
	writel(0x12345678, 0x80000400);
	writel(0xA2220000, 0xB8001000);
	writel(0x87654321, 0x80000000);
	writel(0x87654321, 0x80000000);
	writel(0xB2220000, 0xB8001000);
	writeb(0xda, 0x80000233);
	writeb(0xda, 0x82000780);
	writeb(0xda, 0x82000400);
	writel(0x82226080,  0xB8001000);
	/* bank 2 */
	writel(0x007ffc2f,  0xB800100C);
	writel(0x92220000,  0xB8001008);
	writel(0x12345678,  0x90000400);
	writel(0xA2220000,  0xB8001008);
	writel(0x87654321,  0x90000000);
	writel(0x87654321,  0x90000000);
	writel(0xB2220000,  0xB8001008);
	writeb(0xda, 0x90000233);
	writeb(0xda, 0x92000780);
	writeb(0xda, 0x92000400);
	writel(0x82226080,  0xB8001008);
    /* finish off */
	writel(0x00000304,  0xB8001010);

    return 0;
};

BAREBOX_CMD_START(vpr200_sdraminit)
	.cmd	= do_vpr200_sdraminit,
	.usage  = "initialize SDRAM registers",
BAREBOX_CMD_END

/* ------------------------------------------------------------------------ */
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
				(val >> 0) & 0xff);
		dump_binary(val);
		printf("\n");
	}

	return 0;
}

BAREBOX_CMD_START(mc13xxx_dump)
	.cmd	= do_mc13xxx_dump,
	.usage  = "Dump mc13xxx registers",
BAREBOX_CMD_END

/* ------------------------------------------------------------------------ */

struct test_pad_info {
	iomux_v3_cfg_t normaldesc;
	iomux_v3_cfg_t gpiodesc;
	int gpionum;
	int hasPullup;
} test_pad_info;

static struct test_pad_info vpr_pads_test[] = {

// BANK 1
	{ // LCD_CONTRAST
		MX35_PAD_CONTRAST__IPU_DISPB_CONTR,
		MX35_PAD_CONTRAST__GPIO1_1,
		IMX_GPIO_NR(1, 1),
		0
	},

	{ // LCD_DRDY
		MX35_PAD_D3_DRDY__IPU_DISPB_D3_DRDY,
		MX35_PAD_D3_DRDY__GPIO1_0,
		IMX_GPIO_NR(1, 0),
		0
	},

	{ // LCD_LSCLK
		MX35_PAD_D3_FPSHIFT__IPU_DISPB_D3_CLK,
		MX35_PAD_D3_FPSHIFT__GPIO3_31,
		IMX_GPIO_NR(3, 31),
		0
	},

	{ // LCD_D0
		MX35_PAD_LD0__IPU_DISPB_DAT_0,
		MX35_PAD_LD0__GPIO2_0,
		IMX_GPIO_NR(2, 0),
		0
	},

	{ // LCD_D1
		MX35_PAD_LD1__IPU_DISPB_DAT_1,
		MX35_PAD_LD1__GPIO2_1,
		IMX_GPIO_NR(2, 1),
		0
	},

	{ // LCD_D2
		MX35_PAD_LD2__IPU_DISPB_DAT_2,
		MX35_PAD_LD2__GPIO2_2,
		IMX_GPIO_NR(2, 2),
		0
	},

	{ // LCD_D3
		MX35_PAD_LD3__IPU_DISPB_DAT_3,
		MX35_PAD_LD3__GPIO2_3,
		IMX_GPIO_NR(2, 3),
		0
	},

	{ // LCD_D4
		MX35_PAD_LD4__IPU_DISPB_DAT_4,
		MX35_PAD_LD4__GPIO2_4,
		IMX_GPIO_NR(2, 4),
		0
	},

	{ // /BUTTON1
		MX35_PAD_SCKR__ESAI_SCKR,
		MX35_PAD_SCKR__GPIO1_4,
		IMX_GPIO_NR(1, 4),
		0
	},

// BANK 2
	{ // LCD_D5
		MX35_PAD_LD5__IPU_DISPB_DAT_5,
		MX35_PAD_LD5__GPIO2_5,
		IMX_GPIO_NR(2, 5),
		0
	},

	{ // LCD_D6
		MX35_PAD_LD6__IPU_DISPB_DAT_6,
		MX35_PAD_LD6__GPIO2_6,
		IMX_GPIO_NR(2, 6),
		0
	},

	{ // LCD_D7
		MX35_PAD_LD7__IPU_DISPB_DAT_7,
		MX35_PAD_LD7__GPIO2_7,
		IMX_GPIO_NR(2, 7),
		0
	},

	{ // LCD_D8
		MX35_PAD_LD8__IPU_DISPB_DAT_8,
		MX35_PAD_LD8__GPIO2_8,
		IMX_GPIO_NR(2, 8),
		0
	},

	{ // LCD_D9
		MX35_PAD_LD9__IPU_DISPB_DAT_9,
		MX35_PAD_LD9__GPIO2_9,
		IMX_GPIO_NR(2, 9),
		0
	},

	{ // LCD_D10
		MX35_PAD_LD10__IPU_DISPB_DAT_10,
		MX35_PAD_LD10__GPIO2_10,
		IMX_GPIO_NR(2, 10),
		0
	},

	{ // LCD_D11
		MX35_PAD_LD11__IPU_DISPB_DAT_11,
		MX35_PAD_LD11__GPIO2_11,
		IMX_GPIO_NR(2, 11),
		0
	},

	{ // LCD_D12
		MX35_PAD_LD12__IPU_DISPB_DAT_12,
		MX35_PAD_LD12__GPIO2_12,
		IMX_GPIO_NR(2, 12),
		0
	},

	{ // /BUTTON2
		MX35_PAD_COMPARE__GPT_CMPOUT1,
		MX35_PAD_COMPARE__GPIO1_5,
		IMX_GPIO_NR(1, 5),
		0
	},

// BANK 3
	{ // LCD_D13
		MX35_PAD_LD13__IPU_DISPB_DAT_13,
		MX35_PAD_LD13__GPIO2_13,
		IMX_GPIO_NR(2, 13),
		0
	},

	{ // LCD_D14
		MX35_PAD_LD14__IPU_DISPB_DAT_14,
		MX35_PAD_LD14__GPIO2_14,
		IMX_GPIO_NR(2, 14),
		0
	},

	{ // LCD_D15
		MX35_PAD_LD15__IPU_DISPB_DAT_15,
		MX35_PAD_LD15__GPIO2_15,
		IMX_GPIO_NR(2, 15),
		0
	},

	{ // LCD_D16
		MX35_PAD_LD16__IPU_DISPB_DAT_16,
		MX35_PAD_LD16__GPIO2_16,
		IMX_GPIO_NR(2, 16),
		0
	},

	{ // LCD_D17
		MX35_PAD_LD17__IPU_DISPB_DAT_17,
		MX35_PAD_LD17__GPIO2_17,
		IMX_GPIO_NR(2, 17),
		0
	},

	{ // /LCD_PWR
		MX35_PAD_D3_VSYNC__IPU_DISPB_D3_VSYNC,
		MX35_PAD_D3_VSYNC__GPIO1_2,
		IMX_GPIO_NR(1, 2),
		0
	},

	{ // GPIO2_1
		MX35_PAD_SD2_CLK__ESDHC2_CLK,
		MX35_PAD_SD2_CLK__GPIO2_1,
		IMX_GPIO_NR(2, 1),
		0
	},

	{ // GPIO2_2
		MX35_PAD_SD2_DATA0__ESDHC2_DAT0,
		MX35_PAD_SD2_DATA0__GPIO2_2,
		IMX_GPIO_NR(2, 2),
		0
	},

	{ // /BUTTON3
		MX35_PAD_SCKT__ESAI_SCKT,
		MX35_PAD_SCKT__GPIO1_7,
		IMX_GPIO_NR(1, 7),
		0
	},

// BANK 4
	{ // GPIO2_3
		MX35_PAD_SD2_DATA1__ESDHC2_DAT1,
		MX35_PAD_SD2_DATA1__GPIO2_3,
		IMX_GPIO_NR(2, 3),
		0
	},

	{ // GPIO2_4
		MX35_PAD_SD2_DATA2__ESDHC2_DAT2,
		MX35_PAD_SD2_DATA2__GPIO2_4,
		IMX_GPIO_NR(2, 4),
		0
	},

	{ // GPIO2_5
		MX35_PAD_SD2_DATA3__ESDHC2_DAT3,
		MX35_PAD_SD2_DATA3__GPIO2_5,
		IMX_GPIO_NR(2, 5),
		0
	},

	{ // GPIO2_6
		MX35_PAD_ATA_CS0__ATA_CS0,
		MX35_PAD_ATA_CS0__GPIO2_6,
		IMX_GPIO_NR(2, 6),
		0
	},

	{ // LED_PWR
		MX35_PAD_ATA_CS1__ATA_CS1,
		MX35_PAD_ATA_CS1__GPIO2_7,
		IMX_GPIO_NR(2, 7),
		0
	},

	{ // MB_CFG_0
		MX35_PAD_ATA_DIOR__ATA_DIOR,
		MX35_PAD_ATA_DIOR__GPIO2_8,
		IMX_GPIO_NR(2, 8),
		0
	},

	{ // MB_CFG_1
		MX35_PAD_ATA_DIOW__ATA_DIOW,
		MX35_PAD_ATA_DIOW__GPIO2_9,
		IMX_GPIO_NR(2, 9),
		0
	},

	{ // MB_CFG_2
		MX35_PAD_ATA_DMACK__ATA_DMACK,
		MX35_PAD_ATA_DMACK__GPIO2_10,
		IMX_GPIO_NR(2, 10),
		0
	},

	{ // /BP_RESET
		MX35_PAD_ATA_DATA5__ATA_DATA_5,
		MX35_PAD_ATA_DATA5__GPIO2_18,
		IMX_GPIO_NR(2, 18),
		0
	},

// BANK 5
	{ // MB_CFG_3
		MX35_PAD_ATA_RESET_B__ATA_RESET_B,
		MX35_PAD_ATA_RESET_B__GPIO2_11,
		IMX_GPIO_NR(2, 11),
		0
	},

	{ // /RESET_RFCPU
		MX35_PAD_ATA_IORDY__ATA_IORDY,
		MX35_PAD_ATA_IORDY__GPIO2_12,
		IMX_GPIO_NR(2, 12),
		0
	},

	{ // TDI_RFCPU
		MX35_PAD_ATA_DATA0__ATA_DATA_0,
		MX35_PAD_ATA_DATA0__GPIO2_13,
		IMX_GPIO_NR(2, 13),
		0
	},

	{ // TDO_RFCPU
		MX35_PAD_ATA_DATA1__ATA_DATA_1,
		MX35_PAD_ATA_DATA1__GPIO2_14,
		IMX_GPIO_NR(2, 14),
		0
	},

	{ // TCK_RFCPU
		MX35_PAD_ATA_DATA2__ATA_DATA_2,
		MX35_PAD_ATA_DATA2__GPIO2_15,
		IMX_GPIO_NR(2, 15),
		0
	},

	{ // TMS_RFCPU
		MX35_PAD_ATA_DATA3__ATA_DATA_3,
		MX35_PAD_ATA_DATA3__GPIO2_16,
		IMX_GPIO_NR(2, 16),
		0
	},

	{ // BP_EOC
		MX35_PAD_ATA_DATA4__ATA_DATA_4,
		MX35_PAD_ATA_DATA4__GPIO2_17,
		IMX_GPIO_NR(2, 17),
		0
	},

	{ // /BUTTON4
		MX35_PAD_FST__ESAI_FST,
		MX35_PAD_FST__GPIO1_8,
		IMX_GPIO_NR(1, 8),
		0
	},

	{ // /BUTTON5
		MX35_PAD_HCKT__ESAI_HCKT,
		MX35_PAD_HCKT__GPIO1_9,
		IMX_GPIO_NR(1, 9),
		0
	},

// BANK 6
	{ // /CSPI2_SSO
		MX35_PAD_HCKR__ESAI_HCKR,
		MX35_PAD_HCKR__GPIO1_6,
		IMX_GPIO_NR(1, 6),
		0
	},

	{ // /CSPI2_RDY
		MX35_PAD_STXFS5__AUDMUX_AUD5_TXFS,
		MX35_PAD_STXFS5__GPIO1_3,
		IMX_GPIO_NR(1, 3),
		0
	},

	{ // CSPI2_SCLK
		MX35_PAD_SCK5__AUDMUX_AUD5_TXC,
		MX35_PAD_SCK5__GPIO1_2,
		IMX_GPIO_NR(1, 2),
		0
	},

	{ // CSPI2_MOSI
		MX35_PAD_STXD5__AUDMUX_AUD5_TXD,
		MX35_PAD_STXD5__GPIO1_0,
		IMX_GPIO_NR(1, 0),
		0
	},

	{ // CSPI2_MISO
		MX35_PAD_SRXD5__AUDMUX_AUD5_RXD,
		MX35_PAD_SRXD5__GPIO1_1,
		IMX_GPIO_NR(1, 1),
		0
	},

	{ // SSI4_TXFS
		MX35_PAD_STXFS4__AUDMUX_AUD4_TXFS,
		MX35_PAD_STXFS4__GPIO2_31,
		IMX_GPIO_NR(2, 31),
		0
	},

	{ // SSI4_SCK
		MX35_PAD_SCK4__AUDMUX_AUD4_TXC,
		MX35_PAD_SCK4__GPIO2_30,
		IMX_GPIO_NR(2, 30),
		0
	},

	{ // SSI4_RXD
		MX35_PAD_SRXD4__AUDMUX_AUD4_RXD,
		MX35_PAD_SRXD4__GPIO2_29,
		IMX_GPIO_NR(2, 29),
		0
	},

	{ // /BUTTON6
		MX35_PAD_TX5_RX0__ESAI_TX5_RX0,
		MX35_PAD_TX5_RX0__GPIO1_10,
		IMX_GPIO_NR(1, 10),
		0
	},

// BANK 7
	{ // SS14_TXD
		MX35_PAD_STXD4__AUDMUX_AUD4_TXD,
		MX35_PAD_STXD4__GPIO2_28,
		IMX_GPIO_NR(2, 28),
		0
	},

	{ // I2C3_DAT
		MX35_PAD_ATA_DATA13__ATA_DATA_13,
		MX35_PAD_ATA_DATA13__GPIO2_26,
		IMX_GPIO_NR(2, 26),
		1
	},

	{ // I2C3_CLK
		MX35_PAD_ATA_DATA12__ATA_DATA_12,
		MX35_PAD_ATA_DATA12__GPIO2_25,
		IMX_GPIO_NR(2, 25),
		1
	},

	{ // I2C2_DAT
		MX35_PAD_I2C2_DAT__I2C2_SDA,
		MX35_PAD_I2C2_DAT__GPIO2_27,
		IMX_GPIO_NR(2, 27),
		1
	},

	{ // I2C2_CLK
		MX35_PAD_I2C2_CLK__I2C2_SCL,
		MX35_PAD_I2C2_CLK__GPIO2_26,
		IMX_GPIO_NR(2, 26),
		1
	},

	{ // UART3_RXD
		MX35_PAD_ATA_DATA10__ATA_DATA_10,
		MX35_PAD_ATA_DATA10__GPIO2_23,
		IMX_GPIO_NR(2, 23),
		1
	},

	{ // UART3_TXD
		MX35_PAD_ATA_DATA11__ATA_DATA_11,
		MX35_PAD_ATA_DATA11__GPIO2_24,
		IMX_GPIO_NR(2, 24),
		1
	},

	{ // MB5VREG_ON
		MX35_PAD_D3_REV__IPU_DISPB_D3_REV,
		MX35_PAD_D3_REV__GPIO1_3,
		IMX_GPIO_NR(1, 3),
		0
	},

	{ // /LEFT_PWR
		MX35_PAD_TX4_RX1__ESAI_TX4_RX1,
		MX35_PAD_TX4_RX1__GPIO1_11,
		IMX_GPIO_NR(1, 11),
		0
	},

// BANK 8
	{ // CAN1_RX
		MX35_PAD_ATA_DATA7__ATA_DATA_7,
		MX35_PAD_ATA_DATA7__GPIO2_20,
		IMX_GPIO_NR(2, 20),
		1
	},

	{ // CAN1_TX
		MX35_PAD_ATA_DATA6__ATA_DATA_6,
		MX35_PAD_ATA_DATA6__GPIO2_19,
		IMX_GPIO_NR(2, 19),
		0
	},

	{ // /CSPI1_SS1
		MX35_PAD_CSPI1_SS1__CSPI1_SS1,
		MX35_PAD_CSPI1_SS1__GPIO1_19,
		IMX_GPIO_NR(1, 19),
		0
	},

	{ // /CSPI1_SS0
		MX35_PAD_CSPI1_SS0__CSPI1_SS0,
		MX35_PAD_CSPI1_SS0__GPIO1_18,
		IMX_GPIO_NR(1, 18),
		0
	},

	{ // /CSPI1_RDY
		MX35_PAD_CSPI1_SPI_RDY__CSPI1_RDY,
		MX35_PAD_CSPI1_SPI_RDY__GPIO3_5,
		IMX_GPIO_NR(3, 5),
		0
	},

	{ // CSPI1_SCLK
		MX35_PAD_CSPI1_SCLK__CSPI1_SCLK,
		MX35_PAD_CSPI1_SCLK__GPIO3_4,
		IMX_GPIO_NR(3, 4),
		0
	},

	{ // CSPI1_MOSI
		MX35_PAD_CSPI1_MOSI__CSPI1_MOSI,
		MX35_PAD_CSPI1_MOSI__GPIO1_16,
		IMX_GPIO_NR(1, 16),
		0
	},

	{ // CSPI1_MISO
		MX35_PAD_CSPI1_MISO__CSPI1_MISO,
		MX35_PAD_CSPI1_MISO__GPIO1_17,
		IMX_GPIO_NR(1, 17),
		0
	},

	{ // /RIGHT_PWR
		MX35_PAD_TX3_RX2__ESAI_TX3_RX2,
		MX35_PAD_TX3_RX2__GPIO1_12,
		IMX_GPIO_NR(1, 12),
		0
	},

// BANK 9
	{ // MB3V3REG_ON
		MX35_PAD_TX0__ESAI_TX0,
		MX35_PAD_TX0__GPIO1_15,
		IMX_GPIO_NR(1, 15),
		0
	},

	{ // /FEC_RESET
		MX35_PAD_GPIO3_0__GPIO3_0,
		MX35_PAD_GPIO3_0__GPIO3_0,
		IMX_GPIO_NR(3, 0),
		0
	},

	{ // FEC_TX_ERR
		MX35_PAD_FEC_TX_ERR__FEC_TX_ERR,
		MX35_PAD_FEC_TX_ERR__GPIO3_15,
		IMX_GPIO_NR(3, 15),
		0
	},

	{ // FEC_CRS
		MX35_PAD_FEC_CRS__FEC_CRS,
		MX35_PAD_FEC_CRS__GPIO3_17,
		IMX_GPIO_NR(3, 17),
		0
	},

	{ // FEC_COL
		MX35_PAD_FEC_COL__FEC_COL,
		MX35_PAD_FEC_COL__GPIO3_9,
		IMX_GPIO_NR(3, 9),
		0
	},

	{ // FEC_TXD3
		MX35_PAD_FEC_TDATA3__FEC_TDATA_3,
		MX35_PAD_FEC_TDATA3__GPIO3_23,
		IMX_GPIO_NR(3, 23),
		0
	},

	{ // FEC_TXD2
		MX35_PAD_FEC_TDATA2__FEC_TDATA_2,
		MX35_PAD_FEC_TDATA2__GPIO3_21,
		IMX_GPIO_NR(3, 21),
		0
	},

	{ // FEC_TXD1
		MX35_PAD_FEC_TDATA1__FEC_TDATA_1,
		MX35_PAD_FEC_TDATA1__GPIO3_19,
		IMX_GPIO_NR(3, 19),
		0
	},

	{ // GSM_PWR
		MX35_PAD_TX2_RX3__ESAI_TX2_RX3,
		MX35_PAD_TX2_RX3__GPIO1_13,
		IMX_GPIO_NR(1, 13),
		0
	},

// BANK 10
	{ // FEC_TXD0
		MX35_PAD_FEC_TDATA0__FEC_TDATA_0,
		MX35_PAD_FEC_TDATA0__GPIO3_11,
		IMX_GPIO_NR(3, 11),
		0
	},

	{ // FEC_TX_EN
		MX35_PAD_FEC_TX_EN__FEC_TX_EN,
		MX35_PAD_FEC_TX_EN__GPIO3_12,
		IMX_GPIO_NR(3, 12),
		0
	},

	{ // FEC_TX_CLK
		MX35_PAD_FEC_TX_CLK__FEC_TX_CLK,
		MX35_PAD_FEC_TX_CLK__GPIO3_6,
		IMX_GPIO_NR(3, 6),
		0
	},

	{ // FEC_RX_ER
		MX35_PAD_FEC_RX_ERR__FEC_RX_ERR,
		MX35_PAD_FEC_RX_ERR__GPIO3_16,
		IMX_GPIO_NR(3, 16),
		0
	},

	{ // FEC_RX_CLK
		MX35_PAD_FEC_RX_CLK__FEC_RX_CLK,
		MX35_PAD_FEC_RX_CLK__GPIO3_7,
		IMX_GPIO_NR(3, 7),
		0
	},

	{ // FEC_RX_DV
		MX35_PAD_FEC_RX_DV__FEC_RX_DV,
		MX35_PAD_FEC_RX_DV__GPIO3_8,
		IMX_GPIO_NR(3, 8),
		0
	},

	{ // FEC_RXD3
		MX35_PAD_FEC_RDATA3__FEC_RDATA_3,
		MX35_PAD_FEC_RDATA3__GPIO3_22,
		IMX_GPIO_NR(3, 22),
		0
	},

	{ // FEC_RXD2
		MX35_PAD_FEC_RDATA2__FEC_RDATA_2,
		MX35_PAD_FEC_RDATA2__GPIO3_20,
		IMX_GPIO_NR(3, 20),
		0
	},

	{ // /SPEAKER
		MX35_PAD_TX1__ESAI_TX1,
		MX35_PAD_TX1__GPIO1_14,
		IMX_GPIO_NR(1, 14),
		0
	},

// BANK 11
	{ // 1WB
		MX35_PAD_GPIO1_0__GPIO1_0,
		MX35_PAD_GPIO1_0__GPIO1_0,
		IMX_GPIO_NR(1, 0),
		0
	},

	{ // FEC_RXD0
		MX35_PAD_FEC_RDATA0__FEC_RDATA_0,
		MX35_PAD_FEC_RDATA0__GPIO3_10,
		IMX_GPIO_NR(3, 10),
		0
	},

	{ // FEC_MDC
		MX35_PAD_FEC_MDC__FEC_MDC,
		MX35_PAD_FEC_MDC__GPIO3_13,
		IMX_GPIO_NR(3, 13),
		0
	},

	{ // FEC_MDC
		MX35_PAD_FEC_MDIO__FEC_MDIO,
		MX35_PAD_FEC_MDIO__GPIO3_14,
		IMX_GPIO_NR(3, 14),
		0
	},

	{ // CLKO
		MX35_PAD_CLKO__CCM_CLKO,
		MX35_PAD_CLKO__GPIO1_8,
		IMX_GPIO_NR(1, 8),
		0
	},

	{ // BUZZER
		MX35_PAD_GPIO1_1__GPIO1_1,
		MX35_PAD_GPIO1_1__GPIO1_1,
		IMX_GPIO_NR(1, 1),
		0
	},

	{ // FEC_RXD1
		MX35_PAD_FEC_RDATA1__FEC_RDATA_1,
		MX35_PAD_FEC_RDATA1__GPIO3_18,
		IMX_GPIO_NR(3, 18),
		0
	},

};

#define TEST_BANK_SIZE	10
struct bank {
	int outputs[TEST_BANK_SIZE];
	int feedback;
} bank;

struct bank banks[] = {

	// BANK 1
	{
		{0,1,2,3,4,5,6,7,-1,-1},
		8,
	},
	// BANK 2
	{
		{9,10,11,12,13,14,15,16,-1,-1},
		17,
	},
	// BANK 3
	{
		{18,19,20,21,22,23,24,25,-1,-1},
		26,
	},
	// BANK 4
	{
		{27,28,29,30,31,32,33,34,-1,-1},
		35,
	},
	// BANK 5
	{
		{36,37,38,39,40,41,42,43,-1,-1},
		44,
	},
	// BANK 6
	{
		{45,46,47,48,49,50,51,52,-1,-1},
		53,
	},
	// BANK 7
	{
		{54,55,56,57,58,59,60,61,-1,-1},
		62,
	},
	// BANK 8
	{
		{63,64,65,66,67,68,69,70,-1,-1},
		71,
	},
	// BANK 9
	{
		{72,73,74,75,76,77,78,79,-1,-1},
		80,
	},
	// BANK 10
	{
		{81,82,83,84,85,86,87,88,-1,-1},
		89,
	},
	// BANK 11
	{
		{90,91,92,93,94,95,-1,-1,-1,-1},
		96,
	},
};
#define NUM_TEST_PINS		(sizeof(vpr_pads_test)/sizeof(test_pad_info))
#define NUM_TEST_BANKS		(sizeof(banks)/sizeof(bank))

static int test_all_banks(int testingGPIONum, int feedbackGPIONum, int setValue);

static int set_bank_to_defaults(int bankIndex);
static int set_bank_to_gpio(int bankIndex);
static int test_bank(int bankIndex);

static int do_vpr_test(int argc, char* argv[])
{
	int ii;

	// Red
	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_RED);

	udelay(1000000);
	// Green
	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_GREEN);

	udelay(1000000);
	// Blue
	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_BLUE);

	udelay(1000000);
	// Red
	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_RED);

	udelay(1000000);
	// Green
	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_GREEN);

	udelay(1000000);
	// Blue
	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_BLUE);


	udelay(1000000);
	// Off
	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_OFF);

	// Set all banks to inputs
	for (ii = 0; ii < NUM_TEST_BANKS; ++ii) {
		set_bank_to_defaults(ii);
	}

	for (ii = 0; ii < NUM_TEST_BANKS; ++ii) {
		set_bank_to_gpio(ii);

		// Wait 10ms for pin to change state
		udelay(10);

		if (test_bank(ii)) {
			led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_RED);
			printf("TEST FAILED!\n");
			return 1;
		}
		set_bank_to_defaults(ii);
	}

	led_set(led_by_name(VPR_DIAG_LED_NAME), DIAG_LED_GREEN);
	printf("TEST PASSED!\n");

	return 0;
}

static int set_bank_to_defaults(int bankIndex)
{
	int ii;

	for (ii = 0; ii < TEST_BANK_SIZE; ++ii) {
		if (banks[bankIndex].outputs[ii] == -1)
			continue;

		if (vpr_pads_test[banks[bankIndex].outputs[ii]].hasPullup)
			gpio_direction_output(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum, 0);
		else
			gpio_direction_input(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum);

		mxc_iomux_v3_setup_pad(vpr_pads_test[banks[bankIndex].outputs[ii]].normaldesc);
	}

	gpio_direction_input(vpr_pads_test[banks[bankIndex].feedback].gpionum);
	mxc_iomux_v3_setup_pad(vpr_pads_test[banks[bankIndex].feedback].normaldesc);

	return 0;
}

static int set_bank_to_gpio(int bankIndex)
{
	int ii;

	for (ii = 0; ii < TEST_BANK_SIZE; ++ii) {
		if (banks[bankIndex].outputs[ii] == -1)
			continue;

		mxc_iomux_v3_setup_pad(vpr_pads_test[banks[bankIndex].outputs[ii]].gpiodesc);

		// If pin has a pull up on CPU board, then drive it low so we dont bias the test
		if (vpr_pads_test[banks[bankIndex].outputs[ii]].hasPullup)
			gpio_direction_output(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum, 0);
		else
			gpio_direction_input(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum);
	}

	mxc_iomux_v3_setup_pad(vpr_pads_test[banks[bankIndex].feedback].gpiodesc);
	gpio_direction_input(vpr_pads_test[banks[bankIndex].feedback].gpionum);

	return 0;
}

static int test_bank(int bankIndex)
{
	int ii, jj;
	char buffer[50];

	sprintf(buffer, "Bank: %i \n", bankIndex+1);
	printf(buffer);

	// Test that each output on the bank has gone low
	for (ii = 0; ii < TEST_BANK_SIZE; ++ii) {
		if (banks[bankIndex].outputs[ii] == -1) continue;

		// All other pins should be low
		if (gpio_get_value(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum) == 1) {
			sprintf(buffer, "Pin High (should be low) on GPIO Num: %i, Index: %i\n",
					vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum, ii+1);
			printf(buffer);
			return 1;
		}
	}

	// Put each pin high and check that its not shorted to another bank pin and that the feedback pin goes high
	for (ii = 0; ii < TEST_BANK_SIZE; ++ii) {
		if (banks[bankIndex].outputs[ii] == -1) continue;

		sprintf(buffer, "  Index: %i, GPIO Num: %i, Feedback pin: %i\n",
				ii+1, vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum,
				vpr_pads_test[banks[bankIndex].feedback].gpionum);
		printf(buffer);

		// Set the pin to output, and high
		gpio_direction_output(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum, 1);

		// Wait 10ms for pin to change state
		udelay(10);

		// Test all other pins make sure they arent shorted to the current tested pin
		for (jj = 0; jj < TEST_BANK_SIZE; ++jj) {
			// Ignore matching pin
			if (jj == ii)
				continue;

			// Ignore any pin with a pull up on it
			if (vpr_pads_test[banks[bankIndex].outputs[jj]].hasPullup)
				continue;

			// Ingore -1 pins
			if (banks[bankIndex].outputs[jj] == -1)
				continue;

			// All other pins should be low
			if (gpio_get_value(vpr_pads_test[banks[bankIndex].outputs[jj]].gpionum) == 1) {
				sprintf(buffer, "     SCCT on GPIO Num: %i, Index: %i\n",
						vpr_pads_test[banks[bankIndex].outputs[jj]].gpionum, jj + 1);
				printf(buffer);
				return 0;
			}
		}

		// Feedback pin should be high
		if (gpio_get_value(vpr_pads_test[banks[bankIndex].feedback].gpionum) == 0) {
			sprintf(buffer, "     Feedback Error GPIO Num: %i\n",
					vpr_pads_test[banks[bankIndex].feedback].gpionum);
			printf(buffer);
			return 1;
		}
		// Set the pin to back to low and a input
		gpio_direction_output(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum, 0);

		// If pin has a pull up reset back to output driven low so it doesnt affect other tests
		if (vpr_pads_test[banks[bankIndex].outputs[ii]].hasPullup)
			gpio_direction_output(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum, 0);
		else
			gpio_direction_input(vpr_pads_test[banks[bankIndex].outputs[ii]].gpionum);
	}

	return 0;
}

static int test_all_banks(int testingGPIONum, int feedbackGPIONum, int setValue)
{
	int ii, jj;
	char buffer[50];
	int ignoreFeedbackPin;

	for (ii = 0; ii < NUM_TEST_BANKS; ++ii) {
		ignoreFeedbackPin = 0;
		// Test the pins in the bank
		for (jj = 0; jj < TEST_BANK_SIZE; ++jj) {
			// Ignore -1 pins
			if (banks[ii].outputs[jj] == -1)
				continue;

			// Ignore pullup pins since they cant be checked properly
			if (vpr_pads_test[banks[ii].outputs[jj]].hasPullup)
				continue;

			// Ignore the pin we are testing and any other pins that are replicated on it
			if (testingGPIONum == vpr_pads_test[banks[ii].outputs[jj]].gpionum) {
				ignoreFeedbackPin = 1;
				continue;
			}

			// The other pins should be the inverse of setValue
			if (gpio_get_value(vpr_pads_test[banks[ii].outputs[jj]].gpionum) == setValue) {
				sprintf(buffer, "SCCT on Bank: %i, Index: %i, GPIO Num: %i \n",
						ii, jj, vpr_pads_test[banks[ii].outputs[jj]].gpionum);
				printf(buffer);
				return 0;
			}
		}

		// Dont test this feedback pin
		if (ignoreFeedbackPin)
			continue;

		// The feedback pin for this bank should be in the same as setValue,
		// hence if its not equal to setValue flag an error
		if (feedbackGPIONum == vpr_pads_test[banks[ii].feedback].gpionum) {
			// If pin is something other to the value
			if (gpio_get_value(vpr_pads_test[banks[ii].feedback].gpionum) != setValue) {
				sprintf(buffer, "Feedback Pin Doesnt Match: %i \n",
						vpr_pads_test[banks[ii].feedback].gpionum);
				printf(buffer);
				return 0;
			}
		} else {
		// Other feedback pins should be the inverse to setValue,
		// hence if they equal setValue flag an error
			if (gpio_get_value(vpr_pads_test[banks[ii].feedback].gpionum) == setValue) {
				sprintf(buffer, "SCCT on Bank: %i, Feedback GPIO Num: %i \n",
						ii, vpr_pads_test[banks[ii].feedback].gpionum);
				printf(buffer);
				return 0;
			}
		}
	}

	return 1;
}
BAREBOX_CMD_START(vpr_test)
	.cmd	= do_vpr_test,
	.usage  = "Do VPR200 CPU board test",
BAREBOX_CMD_END

