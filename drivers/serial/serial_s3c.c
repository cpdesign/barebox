/*
 * (c) 2009...2011 Juergen Beisert <j.beisert@pengutronix.de>
 *
 * Based on code from:
 * (c) 2004 Sascha Hauer <sascha@saschahauer.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <common.h>
#include <driver.h>
#include <init.h>
#include <malloc.h>
#include <io.h>
#include <mach/s3c-generic.h>
#include <mach/s3c-iomap.h>

/* Note: Offsets are for little endian access */
#define ULCON 0x00		/* line control */
#define UCON 0x04		/* UART control */
#define UFCON 0x08		/* FIFO control */
#define UMCON 0x0c		/* modem control */
#define UTRSTAT 0x10		/* Rx/Tx status */
#define UERSTAT 0x14		/* error status */
#define UFSTAT 0x18		/* FIFO status */
#define UMSTAT 0x1c		/* modem status */
#define UTXH 0x20		/* transmitt */
#define URXH 0x24		/* receive */
#define UBRDIV 0x28		/* baudrate generator */

static unsigned s3c_get_arch_uart_input_clock(void __iomem *base)
{
	unsigned reg = readw(base + UCON);

	switch (reg & 0xc00) {
		case 0x000:
		case 0x800:
			return s3c_get_pclk();
		case 0x400:
			break;	/* TODO UEXTCLK */
		case 0xc00:
			break;	/* TODO FCLK/n */
	}

	return 0;	/* not nice, but we can't emit an error message! */
}

static int s3c_serial_setbaudrate(struct console_device *cdev, int baudrate)
{
	struct device_d *dev = cdev->dev;
	void __iomem *base = dev->priv;
	unsigned val;

	val = s3c_get_arch_uart_input_clock(base) / (16 * baudrate) - 1;
	writew(val, base + UBRDIV);

	return 0;
}

static int s3c_serial_init_port(struct console_device *cdev)
{
	struct device_d *dev = cdev->dev;
	void __iomem *base = dev->priv;

	/* FIFO enable, Tx/Rx FIFO clear */
	writeb(0x07, base + UFCON);
	writeb(0x00, base + UMCON);

	/* Normal,No parity,1 stop,8 bit */
	writeb(0x03, base + ULCON);
	/*
	 * tx=level,rx=edge,disable timeout int.,enable rx error int.,
	 * normal,interrupt or polling
	 */
	writew(0x0245, base + UCON);

#ifdef CONFIG_DRIVER_SERIAL_S3C_AUTOSYNC
	writeb(0x10, base + UMCON); /* enable auto flow control */
#else
	writeb(0x01, base + UMCON); /* RTS up */
#endif

	return 0;
}

static void s3c_serial_putc(struct console_device *cdev, char c)
{
	struct device_d *dev = cdev->dev;
	void __iomem *base = dev->priv;

	/* Wait for Tx FIFO not full */
	while (!(readb(base + UTRSTAT) & 0x2))
		;

	writeb(c, base + UTXH);
}

static int s3c_serial_tstc(struct console_device *cdev)
{
	struct device_d *dev = cdev->dev;
	void __iomem *base = dev->priv;

	/* If receive fifo is empty, return false */
	if (readb(base + UTRSTAT) & 0x1)
		return 1;

	return 0;
}

static int s3c_serial_getc(struct console_device *cdev)
{
	struct device_d *dev = cdev->dev;
	void __iomem *base = dev->priv;

	/* wait for a character */
	while (!(readb(base + UTRSTAT) & 0x1))
		;

	return readb(base + URXH);
}

static void s3c_serial_flush(struct console_device *cdev)
{
	struct device_d *dev = cdev->dev;
	void __iomem *base = dev->priv;

	while (!(readb(base + UTRSTAT) & 0x4))
		;
}

static int s3c_serial_probe(struct device_d *dev)
{
	struct console_device *cdev;

	cdev = xzalloc(sizeof(struct console_device));
	dev->type_data = cdev;
	dev->priv = dev_request_mem_region(dev, 0);
	cdev->dev = dev;
	cdev->f_caps = CONSOLE_STDIN | CONSOLE_STDOUT | CONSOLE_STDERR;
	cdev->tstc = s3c_serial_tstc;
	cdev->putc = s3c_serial_putc;
	cdev->getc = s3c_serial_getc;
	cdev->flush = s3c_serial_flush;
	cdev->setbrg = s3c_serial_setbaudrate;

	s3c_serial_init_port(cdev);

	/* Enable UART */
	console_register(cdev);

	return 0;
}

static void s3c_serial_remove(struct device_d *dev)
{
	struct console_device *cdev = dev->type_data;

	s3c_serial_flush(cdev);
	free(cdev);
	dev->type_data = NULL;
}

static struct driver_d s3c_serial_driver = {
	.name   = "s3c_serial",
	.probe  = s3c_serial_probe,
	.remove = s3c_serial_remove,
};

static int s3c_serial_init(void)
{
	register_driver(&s3c_serial_driver);
	return 0;
}

console_initcall(s3c_serial_init);
