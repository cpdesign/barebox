/*
 * devfs.c - a device file system for barebox
 *
 * Copyright (c) 2011 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <common.h>
#include <driver.h>
#include <errno.h>
#include <malloc.h>
#include <ioctl.h>
#include <linux/err.h>
#include <linux/mtd/mtd.h>

LIST_HEAD(cdev_list);

/**
 * Allocates a unique name for a cdev based on existing cdevs.
 *
 * @param template Starting point for the name of the device.
 * The name can optionally contain a "%d" marker to designate where the
 * device number should be inserted into the device string. If no marker
 * is specified then one is appended.
 *
 * @param prefid Preferred device id. Any id less than zero will default
 * to start from 0.
 */
char *make_cdev_name(const char *template, int prefid)
{
	char *name = 0;
	char *temp;
	int id;
	struct cdev *cdev;

	/* if there is no template for number, append one */
	if (!strstr(template, "%d"))
		temp = asprintf("%s%%d", template);
	else
		temp = strdup(template);

	id = (prefid < 0) ? 0 : prefid;

	do {
		free(name);
		name = asprintf(temp, id);

		cdev = cdev_by_name(name);
		if (cdev && (id == prefid))
			printf("WARN: preferred device name %s already used\n",
					name);
		++id;
	} while (cdev);

	free(temp);
	return name;
}

struct cdev *cdev_by_name(const char *filename)
{
	struct cdev *cdev;

	list_for_each_entry(cdev, &cdev_list, list) {
		if (!strcmp(cdev->name, filename))
			return cdev;
	}
	return NULL;
}

struct cdev *cdev_open(const char *name, unsigned long flags)
{
	struct cdev *cdev = cdev_by_name(name);
	int ret;

	if (!cdev)
		return NULL;

	if (cdev->ops->open) {
		ret = cdev->ops->open(cdev);
		if (ret)
			return NULL;
	}

	return cdev;
}

void cdev_close(struct cdev *cdev)
{
	if (cdev->ops->close)
		cdev->ops->close(cdev);
}

ssize_t cdev_read(struct cdev *cdev, void *buf, size_t count, ulong offset, ulong flags)
{
	if (!cdev->ops->read)
		return -ENOSYS;

	return cdev->ops->read(cdev, buf, count, cdev->offset +offset, flags);
}

ssize_t cdev_write(struct cdev *cdev, const void *buf, size_t count, ulong offset, ulong flags)
{
	if (!cdev->ops->write)
		return -ENOSYS;

	return cdev->ops->write(cdev, buf, count, cdev->offset + offset, flags);
}

int cdev_flush(struct cdev *cdev)
{
	if (!cdev->ops->flush)
		return 0;

	return cdev->ops->flush(cdev);
}

static int partition_ioctl(struct cdev *cdev, int request, void *buf)
{
	size_t offset;
	struct mtd_info_user *user = buf;

	switch (request) {
	case MEMSETBADBLOCK:
	case MEMGETBADBLOCK:
		offset = (off_t)buf;
		offset += cdev->offset;
		return cdev->ops->ioctl(cdev, request, (void *)offset);
	case MEMGETINFO:
		if (cdev->mtd) {
			user->type	= cdev->mtd->type;
			user->flags	= cdev->mtd->flags;
			user->size	= cdev->mtd->size;
			user->erasesize	= cdev->mtd->erasesize;
			user->oobsize	= cdev->mtd->oobsize;
			user->mtd	= cdev->mtd;
			/* The below fields are obsolete */
			user->ecctype	= -1;
			user->eccsize	= 0;
			return 0;
		}
		if (!cdev->ops->ioctl)
			return -EINVAL;
		return cdev->ops->ioctl(cdev, request, buf);
	default:
		return -EINVAL;
	}
}

int cdev_ioctl(struct cdev *cdev, int request, void *buf)
{
	if (cdev->flags & DEVFS_IS_PARTITION)
		return partition_ioctl(cdev, request, buf);

	if (!cdev->ops->ioctl)
		return -EINVAL;

	return cdev->ops->ioctl(cdev, request, buf);
}

int cdev_erase(struct cdev *cdev, size_t count, unsigned long offset)
{
	if (!cdev->ops->erase)
		return -ENOSYS;

	return cdev->ops->erase(cdev, count, cdev->offset + offset);
}

int devfs_create(struct cdev *new)
{
	struct cdev *cdev;

	cdev = cdev_by_name(new->name);
	if (cdev)
		return -EEXIST;

	list_add_tail(&new->list, &cdev_list);
	if (new->dev)
		list_add_tail(&new->devices_list, &new->dev->cdevs);

	return 0;
}

int devfs_remove(struct cdev *cdev)
{
	if (cdev->open)
		return -EBUSY;

	list_del(&cdev->list);
	if (cdev->dev)
		list_del(&cdev->devices_list);

	return 0;
}

int devfs_add_partition(const char *devname, unsigned long offset, size_t size,
		int flags, const char *name)
{
	struct cdev *cdev, *new;

	cdev = cdev_by_name(name);
	if (cdev)
		return -EEXIST;

	cdev = cdev_by_name(devname);
	if (!cdev)
		return -ENOENT;

	if (offset + size > cdev->size)
		return -EINVAL;

	new = xzalloc(sizeof (*new));
	new->name = strdup(name);
	new->ops = cdev->ops;
	new->priv = cdev->priv;
	new->size = size;
	new->offset = offset + cdev->offset;
	new->dev = cdev->dev;
	new->flags = flags | DEVFS_IS_PARTITION;

#ifdef CONFIG_PARTITION_NEED_MTD
	if (cdev->mtd) {
		new->mtd = mtd_add_partition(cdev->mtd, offset, size, flags, name);
		if (IS_ERR(new->mtd)) {
			int ret = PTR_ERR(new->mtd);
			free(new);
			return ret;
		}
	}
#endif

	devfs_create(new);

	return 0;
}

int devfs_del_partition(const char *name)
{
	struct cdev *cdev;
	int ret;

	cdev = cdev_by_name(name);
	if (!cdev)
		return -ENOENT;

	if (!(cdev->flags & DEVFS_IS_PARTITION))
		return -EINVAL;
	if (cdev->flags & DEVFS_PARTITION_FIXED)
		return -EPERM;

#ifdef CONFIG_PARTITION_NEED_MTD
	if (cdev->mtd)
		mtd_del_partition(cdev->mtd);
#endif

	ret = devfs_remove(cdev);
	if (ret)
		return ret;

	free(cdev->name);
	free(cdev);

	return 0;
}
