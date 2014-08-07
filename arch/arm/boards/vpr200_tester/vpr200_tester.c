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

#if 0
static int vpr200_mem_init(void)
{
	arm_add_mem_device("ram0", IMX_SDRAM_CS0, 128 * 1024 * 1024);
	arm_add_mem_device("ram1", IMX_SDRAM_CS1, 128 * 1024 * 1024);

	return 0;
}
mem_initcall(vpr200_mem_init);
#endif


#if 0
static int vpr200_mmu_init(void)
{
	l2x0_init((void __iomem *)0x30000000, 0x00030024, 0x00000000);

	return 0;
}
postmmu_initcall(vpr200_mmu_init);
#endif

#if 0
struct imx_nand_platform_data nand_info = {
	.hw_ecc		= 1,
	.flash_bbt	= 1,
};
#endif

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

