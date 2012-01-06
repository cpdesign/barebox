/*
 * Copyright (C) 2007 Sascha Hauer, Pengutronix
 *               2009 Marc Kleine-Budde <mkl@pengutronix.de>
 * Copyright (C) 2010 Baruch Siach <baruch@tkos.co.il>
 *               2011 Marc Reilly, Creative Product Design
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
#include <driver.h>
#include <xfuncs.h>
#include <errno.h>

#include <i2c/i2c.h>
#include <misc/isl22316.h>

#define DRIVERNAME		"isl22316"

#define to_isl22316(a)		container_of(a, struct isl22316, cdev)

#define ISL22316_WR		0
#define ISL22316_ACR		2
#define ISL22316_ACR_VOL	0x80
#define ISL22316_ACR_SHDN	0x40
#define ISL22316_ACR_WIP	0x20

static struct isl22316 *isl22316_dev;

struct isl22316 *isl22316_get(void)
{
	return isl22316_dev;
}
EXPORT_SYMBOL(isl22316_get);

int isl22316_reg_read(struct isl22316 *isl22316, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_read_reg(isl22316->client, reg, val, 1);

	return ret == 1 ? 0 : ret;
}
EXPORT_SYMBOL(isl22316_reg_read);

int isl22316_reg_write(struct isl22316 *isl22316, u8 reg, u8 val)
{
	int ret;

	ret = i2c_write_reg(isl22316->client, reg, &val, 1);

	return ret == 1 ? 0 : ret;
}
EXPORT_SYMBOL(isl22316_reg_write);

int isl22316_get_value(struct isl22316 *isl22316, u8 *val, int save)
{
	int ret;
	u8 acr = ISL22316_ACR_SHDN;

	if (!save)
		acr |= ISL22316_ACR_VOL;

	ret = isl22316_reg_write(isl22316, ISL22316_ACR, acr);
	if (ret)
		return ret;

	ret = isl22316_reg_read(isl22316, ISL22316_WR, val);

	return ret;
}
EXPORT_SYMBOL(isl22316_get_value);

int isl22316_set_value(struct isl22316 *isl22316, u8 val, int save)
{
	int ret;
	u8 acr = ISL22316_ACR_SHDN;

	if (!save)
		acr |= ISL22316_ACR_VOL;

	ret = isl22316_reg_write(isl22316, ISL22316_ACR, acr);
	if (ret)
		return ret;

	ret = isl22316_reg_write(isl22316, ISL22316_WR, val);

	return ret;
}
EXPORT_SYMBOL(isl22316_set_value);

static ssize_t isl22316_read(struct cdev *cdev, void *_buf, size_t count,
		ulong offset, ulong flags)
{
	struct isl22316 *priv = to_isl22316(cdev);
	u8 *buf = _buf;
	size_t i = count;
	int err;

	while (i) {
		err = isl22316_reg_read(priv, offset, buf);
		if (err)
			return (ssize_t)err;
		buf++;
		i--;
		offset++;
	}

	return count;
}

static ssize_t isl22316_write(struct cdev *cdev, const void *_buf, size_t count,
		ulong offset, ulong flags)
{
	struct isl22316 *isl22316 = to_isl22316(cdev);
	const u8 *buf = _buf;
	size_t i = count;
	int err;

	while (i) {
		err = isl22316_reg_write(isl22316, offset, *buf);
		if (err)
			return (ssize_t)err;
		buf++;
		i--;
		offset++;
	}

	return count;
}

static struct file_operations isl22316_fops = {
	.lseek	= dev_lseek_default,
	.read	= isl22316_read,
	.write	= isl22316_write,
};

static int isl22316_probe(struct device_d *dev)
{
	if (isl22316_dev)
		return -EBUSY;

	isl22316_dev = xzalloc(sizeof(*isl22316_dev));
	isl22316_dev->cdev.name = DRIVERNAME;
	isl22316_dev->client = to_i2c_client(dev);
	isl22316_dev->cdev.dev = dev;
	isl22316_dev->cdev.size = 3;
	isl22316_dev->cdev.ops = &isl22316_fops;

	devfs_create(&isl22316_dev->cdev);

	return 0;
}

static struct driver_d isl22316_driver = {
	.name  = DRIVERNAME,
	.probe = isl22316_probe,
};

static int isl22316_init(void)
{
	register_driver(&isl22316_driver);
	return 0;
}

device_initcall(isl22316_init);
