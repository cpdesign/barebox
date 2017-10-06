/*
 * Copyright (C) 2011 Creative Product Design
 *
 * This file is released under the GPLv2
 */
#ifndef __ISL22346_H
#define __ISL22346_H

struct isl22346 {
	struct cdev		cdev;
	struct i2c_client	*client;
};

struct isl22346 *isl22346_get(void);

int isl22346_reg_read(struct isl22346 *isl22346, u8 reg, u8 *val);
int isl22346_reg_write(struct isl22346 *isl22346, u8 reg, u8 val);

/*
 * Uses non volatile storage if save is non zero.
 * (remember that saving to non-vol also sets the working value)
 */
int isl22346_get_value(struct isl22346 *isl22346, u8 *val, int save);
int isl22346_set_value(struct isl22346 *isl22346, u8 val, int save);

#endif
