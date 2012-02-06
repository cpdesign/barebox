/*
 * arch/arm/mach-at91/at91sam9260_devices.c
 *
 *  Copyright (C) 2006 Atmel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <common.h>
#include <sizes.h>
#include <asm/armlinux.h>
#include <asm/hardware.h>
#include <mach/board.h>
#include <mach/at91_pmc.h>
#include <mach/at91sam9260_matrix.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/cpu.h>

#include "generic.h"

void at91_add_device_sdram(u32 size)
{
	arm_add_mem_device("ram0", AT91_CHIPSELECT_1, size);
	if (cpu_is_at91sam9g20()) {
		add_mem_device("sram0", AT91SAM9G20_SRAM0_BASE,
			AT91SAM9G20_SRAM0_SIZE, IORESOURCE_MEM_WRITEABLE);
		add_mem_device("sram1", AT91SAM9G20_SRAM1_BASE,
			AT91SAM9G20_SRAM1_SIZE, IORESOURCE_MEM_WRITEABLE);
	} else {
		add_mem_device("sram0", AT91SAM9260_SRAM0_BASE,
			AT91SAM9260_SRAM0_SIZE, IORESOURCE_MEM_WRITEABLE);
		add_mem_device("sram1", AT91SAM9260_SRAM1_BASE,
			AT91SAM9260_SRAM1_SIZE, IORESOURCE_MEM_WRITEABLE);
	}
}

#if defined(CONFIG_USB_OHCI)
void __init at91_add_device_usbh_ohci(struct at91_usbh_data *data)
{
	if (!data)
		return;

	add_generic_device("at91_ohci", -1, NULL, AT91SAM9260_UHP_BASE, 1024 * 1024,
			   IORESOURCE_MEM, data);
}
#else
void __init at91_add_device_usbh_ohci(struct at91_usbh_data *data) {}
#endif

/* --------------------------------------------------------------------
 *  USB Device (Gadget)
 * -------------------------------------------------------------------- */

#ifdef CONFIG_USB_GADGET_DRIVER_AT91
void __init at91_add_device_udc(struct at91_udc_data *data)
{
	if (data->vbus_pin > 0) {
		at91_set_gpio_input(data->vbus_pin, 0);
		at91_set_deglitch(data->vbus_pin, 1);
	}

	add_generic_device("at91_udc", -1, NULL, AT91SAM9260_BASE_UDP, SZ_16K,
			   IORESOURCE_MEM, data);
}
#else
void __init at91_add_device_udc(struct at91_udc_data *data) {}
#endif

#if defined(CONFIG_DRIVER_NET_MACB)
void at91_add_device_eth(struct at91_ether_platform_data *data)
{
	if (!data)
		return;

	/* Pins used for MII and RMII */
	at91_set_A_periph(AT91_PIN_PA19, 0);	/* ETXCK_EREFCK */
	at91_set_A_periph(AT91_PIN_PA17, 0);	/* ERXDV */
	at91_set_A_periph(AT91_PIN_PA14, 0);	/* ERX0 */
	at91_set_A_periph(AT91_PIN_PA15, 0);	/* ERX1 */
	at91_set_A_periph(AT91_PIN_PA18, 0);	/* ERXER */
	at91_set_A_periph(AT91_PIN_PA16, 0);	/* ETXEN */
	at91_set_A_periph(AT91_PIN_PA12, 0);	/* ETX0 */
	at91_set_A_periph(AT91_PIN_PA13, 0);	/* ETX1 */
	at91_set_A_periph(AT91_PIN_PA21, 0);	/* EMDIO */
	at91_set_A_periph(AT91_PIN_PA20, 0);	/* EMDC */

	if (!(data->flags & AT91SAM_ETHER_RMII)) {
		at91_set_B_periph(AT91_PIN_PA28, 0);	/* ECRS */
		at91_set_B_periph(AT91_PIN_PA29, 0);	/* ECOL */
		at91_set_B_periph(AT91_PIN_PA25, 0);	/* ERX2 */
		at91_set_B_periph(AT91_PIN_PA26, 0);	/* ERX3 */
		at91_set_B_periph(AT91_PIN_PA27, 0);	/* ERXCK */
		if (data->flags & AT91SAM_ETX2_ETX3_ALTERNATIVE) {
			at91_set_B_periph(AT91_PIN_PA10, 0);	/* ETX2 */
			at91_set_B_periph(AT91_PIN_PA11, 0);	/* ETX3 */
		} else {
			at91_set_B_periph(AT91_PIN_PA23, 0);	/* ETX2 */
			at91_set_B_periph(AT91_PIN_PA24, 0);	/* ETX3 */
		}
		at91_set_B_periph(AT91_PIN_PA22, 0);	/* ETXER */
	}

	add_generic_device("macb", 0, NULL, AT91SAM9260_BASE_EMAC, 0x1000,
			   IORESOURCE_MEM, data);
}
#else
void at91_add_device_eth(struct at91_ether_platform_data *data) {}
#endif

#if defined(CONFIG_NAND_ATMEL)
static struct resource nand_resources[] = {
	[0] = {
		.start	= AT91_CHIPSELECT_3,
		.size	= SZ_256M,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AT91_BASE_SYS + AT91_ECC,
		.size	= 512,
		.flags	= IORESOURCE_MEM,
	}
};

void at91_add_device_nand(struct atmel_nand_data *data)
{
	unsigned long csa;

	if (!data)
		return;

	csa = at91_sys_read(AT91_MATRIX_EBICSA);
	at91_sys_write(AT91_MATRIX_EBICSA, csa | AT91_MATRIX_CS3A_SMC_SMARTMEDIA);

	/* enable pin */
	if (data->enable_pin)
		at91_set_gpio_output(data->enable_pin, 1);

	/* ready/busy pin */
	if (data->rdy_pin)
		at91_set_gpio_input(data->rdy_pin, 1);

	/* card detect pin */
	if (data->det_pin)
		at91_set_gpio_input(data->det_pin, 1);

	add_generic_device_res("atmel_nand", 0, nand_resources,
			       ARRAY_SIZE(nand_resources), data);
}
#else
void at91_add_device_nand(struct atmel_nand_data *data) {}
#endif

static inline void configure_dbgu_pins(void)
{
	at91_set_A_periph(AT91_PIN_PB14, 0);		/* DRXD */
	at91_set_A_periph(AT91_PIN_PB15, 1);		/* DTXD */
}

static inline void configure_usart0_pins(unsigned pins)
{
	at91_set_A_periph(AT91_PIN_PB4, 1);		/* TXD0 */
	at91_set_A_periph(AT91_PIN_PB5, 0);		/* RXD0 */

	if (pins & ATMEL_UART_RTS)
		at91_set_A_periph(AT91_PIN_PB26, 0);	/* RTS0 */
	if (pins & ATMEL_UART_CTS)
		at91_set_A_periph(AT91_PIN_PB27, 0);	/* CTS0 */
	if (pins & ATMEL_UART_DTR)
		at91_set_A_periph(AT91_PIN_PB24, 0);	/* DTR0 */
	if (pins & ATMEL_UART_DSR)
		at91_set_A_periph(AT91_PIN_PB22, 0);	/* DSR0 */
	if (pins & ATMEL_UART_DCD)
		at91_set_A_periph(AT91_PIN_PB23, 0);	/* DCD0 */
	if (pins & ATMEL_UART_RI)
		at91_set_A_periph(AT91_PIN_PB25, 0);	/* RI0 */
}

static inline void configure_usart1_pins(unsigned pins)
{
	at91_set_A_periph(AT91_PIN_PB6, 1);		/* TXD1 */
	at91_set_A_periph(AT91_PIN_PB7, 0);		/* RXD1 */

	if (pins & ATMEL_UART_RTS)
		at91_set_A_periph(AT91_PIN_PB28, 0);	/* RTS1 */
	if (pins & ATMEL_UART_CTS)
		at91_set_A_periph(AT91_PIN_PB29, 0);	/* CTS1 */
}

static inline void configure_usart2_pins(unsigned pins)
{
	at91_set_A_periph(AT91_PIN_PB8, 1);		/* TXD2 */
	at91_set_A_periph(AT91_PIN_PB9, 0);		/* RXD2 */

	if (pins & ATMEL_UART_RTS)
		at91_set_A_periph(AT91_PIN_PA4, 0);	/* RTS2 */
	if (pins & ATMEL_UART_CTS)
		at91_set_A_periph(AT91_PIN_PA5, 0);	/* CTS2 */
}

static inline void configure_usart3_pins(unsigned pins)
{
	at91_set_A_periph(AT91_PIN_PB10, 1);		/* TXD3 */
	at91_set_A_periph(AT91_PIN_PB11, 0);		/* RXD3 */

	if (pins & ATMEL_UART_RTS)
		at91_set_B_periph(AT91_PIN_PC8, 0);	/* RTS3 */
	if (pins & ATMEL_UART_CTS)
		at91_set_B_periph(AT91_PIN_PC10, 0);	/* CTS3 */
}

static inline void configure_usart4_pins(void)
{
	at91_set_B_periph(AT91_PIN_PA31, 1);		/* TXD4 */
	at91_set_B_periph(AT91_PIN_PA30, 0);		/* RXD4 */
}

static inline void configure_usart5_pins(void)
{
	at91_set_A_periph(AT91_PIN_PB12, 1);		/* TXD5 */
	at91_set_A_periph(AT91_PIN_PB13, 0);		/* RXD5 */
}

void at91_register_uart(unsigned id, unsigned pins)
{
	resource_size_t start;

	switch (id) {
		case 0:		/* DBGU */
			configure_dbgu_pins();
			start = AT91_BASE_SYS + AT91_DBGU;
			id = 0;
			break;
		case AT91SAM9260_ID_US0:
			configure_usart0_pins(pins);
			start = AT91SAM9260_BASE_US0;
			id = 1;
			break;
		case AT91SAM9260_ID_US1:
			configure_usart1_pins(pins);
			start = AT91SAM9260_BASE_US1;
			id = 2;
			break;
		case AT91SAM9260_ID_US2:
			configure_usart2_pins(pins);
			start = AT91SAM9260_BASE_US2;
			id = 3;
			break;
		case AT91SAM9260_ID_US3:
			configure_usart3_pins(pins);
			start = AT91SAM9260_BASE_US3;
			id = 4;
			break;
		case AT91SAM9260_ID_US4:
			configure_usart4_pins();
			start = AT91SAM9260_BASE_US4;
			id = 5;
			break;
		case AT91SAM9260_ID_US5:
			configure_usart5_pins();
			start = AT91SAM9260_BASE_US5;
			id = 6;
			break;
		default:
			return;
	}

	add_generic_device("atmel_usart", id, NULL, start, 4096,
			   IORESOURCE_MEM, NULL);
}

#if defined(CONFIG_MCI_ATMEL)
/* Consider only one slot : slot 0 */
void at91_add_device_mci(short mmc_id, struct atmel_mci_platform_data *data)
{
	struct device_d *dev;

	if (!data)
		return;

	/* need bus_width */
	if (!data->bus_width)
		return;

	/* input/irq */
	if (data->detect_pin) {
		at91_set_gpio_input(data->detect_pin, 1);
		at91_set_deglitch(data->detect_pin, 1);
	}

	if (data->wp_pin)
		at91_set_gpio_input(data->wp_pin, 1);

	/* CLK */
	at91_set_A_periph(AT91_PIN_PA8, 0);


	if (data->slot_b) {
		/* CMD */
		at91_set_B_periph(AT91_PIN_PA1, 1);

		/* DAT0, maybe DAT1..DAT3 */
		at91_set_B_periph(AT91_PIN_PA0, 1);
		if (data->bus_width == 4) {
			at91_set_B_periph(AT91_PIN_PA3, 1);
			at91_set_B_periph(AT91_PIN_PA4, 1);
			at91_set_B_periph(AT91_PIN_PA5, 1);
		}
	} else {
		/* CMD */
		at91_set_A_periph(AT91_PIN_PA7, 1);

		/* DAT0, maybe DAT1..DAT3 */
		at91_set_A_periph(AT91_PIN_PA6, 1);
		if (data->bus_width == 4) {
			at91_set_A_periph(AT91_PIN_PA9, 1);
			at91_set_A_periph(AT91_PIN_PA10, 1);
			at91_set_A_periph(AT91_PIN_PA11, 1);
		}
	}

	dev = add_generic_device("atmel_mci", mmc_id, NULL, AT91SAM9260_BASE_MCI, SZ_16K,
			   IORESOURCE_MEM, data);
}
#else
void at91_add_device_mci(short mmc_id, struct atmel_mci_platform_data *data) {}
#endif
