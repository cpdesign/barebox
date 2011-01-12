/*
 * Copyright (C) 2010 Creative Product Design
 *
 * This file is released under the GPLv2
 */

#ifndef __AT24_EEPROM_H
#define __AT24_EEPROM_H

struct at24 {
	struct cdev		cdev;
	struct i2c_client	*client;
	/* size in bytes */
	unsigned int		size;
};

struct at24_platform_data {
	/* size in bytes */
	unsigned int	size;
	int		id;
};

#endif
