/*
 * Copyright (C) 2011 Creative Product Design
 *
 * This file is released under the GPLv2
 */
#ifndef __ISL22316_H
#define __ISL22316_H

struct isl22316 {
	struct cdev		cdev;
	struct i2c_client	*client;
};

struct isl22316 *isl22316_get(void);

int isl22316_reg_read(struct isl22316 *isl22316, u8 reg, u8 *val);
int isl22316_reg_write(struct isl22316 *isl22316, u8 reg, u8 val);

/*
 * Uses non volatile storage if save is non zero.
 * (remember that saving to non-vol also sets the working value)
 */
int isl22316_get_value(struct isl22316 *isl22316, u8 *val, int save);
int isl22316_set_value(struct isl22316 *isl22316, u8 val, int save);

#endif
