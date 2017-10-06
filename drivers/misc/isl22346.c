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
#include <misc/isl22346.h>

#define DRIVERNAME		"isl22346"

#define to_isl22346(a)		container_of(a, struct isl22346, cdev)

#define ISL22346_WR		0
#define ISL22346_ACR		8
#define ISL22346_ACR_VOL	0x80
#define ISL22346_ACR_SHDN	0x40
#define ISL22346_ACR_WIP	0x20

static struct isl22346 *isl22346_dev;

struct isl22346 *isl22346_get(void)
{
	return isl22346_dev;
}
EXPORT_SYMBOL(isl22346_get);

int isl22346_reg_read(struct isl22346 *isl22346, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_read_reg(isl22346->client, reg, val, 1);

	return ret == 1 ? 0 : ret;
}
EXPORT_SYMBOL(isl22346_reg_read);

int isl22346_reg_write(struct isl22346 *isl22346, u8 reg, u8 val)
{
	int ret;

	ret = i2c_write_reg(isl22346->client, reg, &val, 1);

	return ret == 1 ? 0 : ret;
}
EXPORT_SYMBOL(isl22346_reg_write);

int isl22346_get_value(struct isl22346 *isl22346, u8 *val, int save)
{
	int ret;
	u8 acr = ISL22346_ACR_SHDN;

	if (!save)
		acr |= ISL22346_ACR_VOL;

	ret = isl22346_reg_write(isl22346, ISL22346_ACR, acr);
	if (ret)
		return ret;

	ret = isl22346_reg_read(isl22346, ISL22346_WR, val);

	return ret;
}
EXPORT_SYMBOL(isl22346_get_value);

int isl22346_set_value(struct isl22346 *isl22346, u8 val, int save)
{
	int ret;
	u8 acr = ISL22346_ACR_SHDN;

	if (!save)
		acr |= ISL22346_ACR_VOL;

	ret = isl22346_reg_write(isl22346, ISL22346_ACR, acr);
	if (ret)
		return ret;

	ret = isl22346_reg_write(isl22346, ISL22346_WR, val);

	return ret;
}
EXPORT_SYMBOL(isl22346_set_value);

static ssize_t isl22346_read(struct cdev *cdev, void *_buf, size_t count,
		loff_t offset, ulong flags)
{
	struct isl22346 *priv = to_isl22346(cdev);
	u8 *buf = _buf;
	size_t i = count;
	int err;

	while (i) {
		err = isl22346_reg_read(priv, offset, buf);
		if (err)
			return (ssize_t)err;
		buf++;
		i--;
		offset++;
	}

	return count;
}

static ssize_t isl22346_write(struct cdev *cdev, const void *_buf, size_t count,
		loff_t offset, ulong flags)
{
	struct isl22346 *isl22346 = to_isl22346(cdev);
	const u8 *buf = _buf;
	size_t i = count;
	int err;

	while (i) {
		err = isl22346_reg_write(isl22346, offset, *buf);
		if (err)
			return (ssize_t)err;
		buf++;
		i--;
		offset++;
	}

	return count;
}

static struct file_operations isl22346_fops = {
	.lseek	= dev_lseek_default,
	.read	= isl22346_read,
	.write	= isl22346_write,
};

static int isl22346_probe(struct device_d *dev)
{
	if (isl22346_dev)
		return -EBUSY;

	isl22346_dev = xzalloc(sizeof(*isl22346_dev));
	isl22346_dev->cdev.name = DRIVERNAME;
	isl22346_dev->client = to_i2c_client(dev);
	isl22346_dev->cdev.dev = dev;
	isl22346_dev->cdev.size = 8;
	isl22346_dev->cdev.ops = &isl22346_fops;

	devfs_create(&isl22346_dev->cdev);

	return 0;
}

static struct driver_d isl22346_driver = {
	.name  = DRIVERNAME,
	.probe = isl22346_probe,
};

static int isl22346_init(void)
{
	register_driver(&isl22346_driver);
	return 0;
}

device_initcall(isl22346_init);
