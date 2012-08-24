/*
 * Copyright (C) 2012 Steffen Trumtrar, Pengutronix
 *
 * based on arch/arm/boards/freescale-mx6-arm2/board.c
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
 * Foundation.
 *
 */

#include <common.h>
#include <init.h>
#include <environment.h>
#include <mach/imx-regs.h>
#include <fec.h>
#include <mach/gpio.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <partition.h>
#include <miidev.h>
#include <asm/io.h>
#include <asm/mmu.h>
#include <mach/generic.h>
#include <sizes.h>
#include <net.h>
#include <mach/imx6.h>
#include <mach/devices-imx6.h>
#include <mach/iomux-mx6.h>
#include <mach/gpio.h>
#include <spi/spi.h>
#include <mach/spi.h>

#define SABRELITE_SD3_WP	IMX_GPIO_NR(7, 1)
#define SABRELITE_SD3_CD	IMX_GPIO_NR(7, 0)

#define SABRELITE_SD4_CD	IMX_GPIO_NR(2, 6)

static iomux_v3_cfg_t sabrelite_pads[] = {
	/* UART1 */
	MX6Q_PAD_SD3_DAT6__UART1_RXD,
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* SD3 (bottom) */
	MX6Q_PAD_SD3_CMD__USDHC3_CMD,
	MX6Q_PAD_SD3_CLK__USDHC3_CLK,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3,
	MX6Q_PAD_SD3_DAT4__GPIO_7_1, /* WP */
	MX6Q_PAD_SD3_DAT5__GPIO_7_0, /* CD */

	/* SD4 (top) */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3,
	MX6Q_PAD_NANDF_D6__GPIO_2_6, /* CD */

	/* ECSPI */
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/* CS1 */
};

static iomux_v3_cfg_t sabrelite_enet_pads[] = {
	/* Ethernet */
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_REF_CLK__GPIO_1_23,	// LED mode
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_EIM_D23__GPIO_3_23,		/* RGMII_nRST */
	MX6Q_PAD_RGMII_RXC__GPIO_6_30,		/* PHYAD */
	MX6Q_PAD_RGMII_RD0__GPIO_6_25,		/* MODE0 */
	MX6Q_PAD_RGMII_RD1__GPIO_6_27,		/* MODE1 */
	MX6Q_PAD_RGMII_RD2__GPIO_6_28,		/* MODE2 */
	MX6Q_PAD_RGMII_RD3__GPIO_6_29,		/* MODE3 */
	MX6Q_PAD_RGMII_RX_CTL__GPIO_6_24,
};

static iomux_v3_cfg_t sabrelite_enet2_pads[] = {
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
};

static int sabrelite_mem_init(void)
{
	arm_add_mem_device("ram0", 0x10000000, SZ_1G);

	return 0;
}
mem_initcall(sabrelite_mem_init);

static struct fec_platform_data fec_info = {
	.xcv_type = RGMII,
	.phy_addr = 6,
};

int mx6_rgmii_rework(void)
{
	struct mii_device *mdev;

	mdev = mii_open("phy0");
	if (!mdev) {
		printf("unable to open phy0\n");
		return -ENODEV;
	}

	mii_write(mdev, mdev->address, 0x09, 0x0f00);

	/* do same as linux kernel */
	/* min rx data delay */
	mii_write(mdev, mdev->address, 0x0b, 0x8105);
	mii_write(mdev, mdev->address, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	mii_write(mdev, mdev->address, 0x0b, 0x8104);
	mii_write(mdev, mdev->address, 0x0c, 0xf0f0);
	mii_write(mdev, mdev->address, 0x0b, 0x104);

	mii_close(mdev);

	return 0;
}

static int sabrelite_ksz9021rn_setup(void)
{
	mxc_iomux_v3_setup_multiple_pads(sabrelite_enet_pads, ARRAY_SIZE(sabrelite_enet_pads));

	gpio_direction_output(87, 0);  /* GPIO 3-23 */

	gpio_direction_output(190, 1); /* GPIO 6-30: PHYAD2 */

	/* LED-Mode: Tri-Color Dual LED Mode */
	gpio_direction_output(23 , 0); /* GPIO 1-23 */

	/* MODE strap-in pins: advertise all capabilities */
	gpio_direction_output(185, 1); /* GPIO 6-25 */
	gpio_direction_output(187, 1); /* GPIO 6-27 */
	gpio_direction_output(188, 1); /* GPIO 6-28*/
	gpio_direction_output(189, 1); /* GPIO 6-29 */

	/* Enable 125 MHz clock output */
        gpio_direction_output(184, 1); /* GPIO 6-24 */

	mdelay(10);
	gpio_set_value(87, 1);

	mxc_iomux_v3_setup_multiple_pads(sabrelite_enet2_pads, ARRAY_SIZE(sabrelite_enet2_pads));

	return 0;
}

static inline int imx6_iim_register_fec_ethaddr(void)
{
	u32 value;
	u8 buf[6];

	value = readl(MX6_OCOTP_BASE_ADDR + 0x630);
	buf[0] = (value >> 8);
	buf[1] = value;

	value = readl(MX6_OCOTP_BASE_ADDR + 0x620);
	buf[2] = value >> 24;
	buf[3] = value >> 16;
	buf[4] = value >> 8;
	buf[5] = value;

	eth_register_ethaddr(0, buf);

	return 0;
}

static int sabrelite_spi_cs[] = {GPIO_PORTC + 19};

static struct spi_imx_master sabrelite_spi_0_data = {
	.chipselect = sabrelite_spi_cs,
	.num_chipselect = ARRAY_SIZE(sabrelite_spi_cs),
};

static const struct spi_board_info sabrelite_spi_board_info[] = {
	{
		.name = "m25p",
		.max_speed_hz = 40000000,
		.bus_num = 0,
		.chip_select = 0,
	}
};

static struct esdhc_platform_data sabrelite_sd3_data = {
	.cd_gpio = SABRELITE_SD3_CD,
	.cd_type = ESDHC_CD_GPIO,
	.wp_gpio = SABRELITE_SD3_WP,
	.wp_type = ESDHC_WP_GPIO,
};

static struct esdhc_platform_data sabrelite_sd4_data = {
	.cd_gpio = SABRELITE_SD4_CD,
	.cd_type = ESDHC_CD_GPIO,
	.wp_type = ESDHC_WP_NONE,
};

static int sabrelite_devices_init(void)
{
	imx6_add_mmc2(&sabrelite_sd3_data);
	imx6_add_mmc3(&sabrelite_sd4_data);

	sabrelite_ksz9021rn_setup();
	imx6_iim_register_fec_ethaddr();
	imx6_add_fec(&fec_info);
	mx6_rgmii_rework();

	spi_register_board_info(sabrelite_spi_board_info,
			ARRAY_SIZE(sabrelite_spi_board_info));
	imx6_add_spi0(&sabrelite_spi_0_data);

	armlinux_set_bootparams((void *)0x10000100);
	armlinux_set_architecture(3769);

	devfs_add_partition("m25p0", 0, SZ_512K, DEVFS_PARTITION_FIXED, "self0");
	devfs_add_partition("m25p0", SZ_512K, SZ_512K, DEVFS_PARTITION_FIXED, "env0");

	return 0;
}

device_initcall(sabrelite_devices_init);

static int sabrelite_console_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(sabrelite_pads, ARRAY_SIZE(sabrelite_pads));

	imx6_init_lowlevel();

	imx6_add_uart1();

	return 0;
}
console_initcall(sabrelite_console_init);
