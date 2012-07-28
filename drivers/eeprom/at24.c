/*
 * Copyright (C) 2007 Sascha Hauer, Pengutronix
 *               2009 Marc Kleine-Budde <mkl@pengutronix.de>
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
 */

#include <common.h>
#include <init.h>
#include <clock.h>
#include <driver.h>
#include <xfuncs.h>
#include <errno.h>

#include <i2c/i2c.h>

#define DRIVERNAME		"eeprom"

struct at24 {
	struct cdev		cdev;
	struct i2c_client	*client;
	/* size in bytes */
	unsigned int		size;
};

#define to_at24(a)		container_of(a, struct at24, cdev)

static ssize_t at24_read(struct cdev *cdev, void *_buf, size_t count,
		ulong offset, ulong flags)
{
	struct at24 *priv = to_at24(cdev);
	u8 *buf = _buf;
	size_t i = count;
	ssize_t numwritten = 0;
	int retries = 5;
	int ret;

	while (i && retries) {
		ret = i2c_read_reg(priv->client, offset, buf, i);
		if (ret < 0)
			return (ssize_t)ret;
		else if (ret == 0)
			--retries;

		numwritten += ret;
		i -= ret;
		offset += ret;
		buf += ret;
	}

	return numwritten;
}

static ssize_t at24_write(struct cdev *cdev, const void *_buf, size_t count,
		ulong offset, ulong flags)
{
	struct at24 *priv = to_at24(cdev);
	const u8 *buf = _buf;
	const int pagesize = 8;
	int numtowrite;
	ssize_t numwritten = 0;

	while (count) {
		int ret;
		u8 dummy;
		uint64_t start;
		int page_remain = pagesize - (offset % pagesize);

		numtowrite = count;
		if (numtowrite > pagesize)
			numtowrite = pagesize;
		/* don't write past page */
		if (numtowrite > page_remain)
			numtowrite = page_remain;

		ret = i2c_write_reg(priv->client, offset, buf, numtowrite);
		if (ret < 0)
			return (ssize_t)ret;

		/*
		 * eeprom can take 5-10ms to write, during which time it
		 * will not respond. Poll it by attempting reads.
		 */
		start = get_time_ns();
		while (!is_timeout(start, 20 * MSECOND)) {
			ret = i2c_master_recv(priv->client, &dummy, 1);
			if (ret > 0)
				break;
		}

		numwritten += ret;
		buf += ret;
		count -= ret;
		offset += ret;
	}

	return numwritten;
}

static struct file_operations at24_fops = {
	.lseek	= dev_lseek_default,
	.read	= at24_read,
	.write	= at24_write,
};

static int at24_probe(struct device_d *dev)
{
	struct at24 *at24;
	at24 = xzalloc(sizeof(*at24));

	dev->priv = at24;

	at24->cdev.name = DRIVERNAME;
	at24->client = to_i2c_client(dev);
	at24->cdev.size = 256;
	at24->cdev.dev = dev;
	at24->cdev.ops = &at24_fops;

	devfs_create(&at24->cdev);

	return 0;
}

static struct driver_d at24_driver = {
	.name  = DRIVERNAME,
	.probe = at24_probe,
};

static int at24_init(void)
{
	register_driver(&at24_driver);
	return 0;
}

device_initcall(at24_init);
