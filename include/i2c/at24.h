#ifndef _EEPROM_AT24_H
#define _EEPROM_AT24_H

struct at24_platform_data {
	/* preferred device name */
	const char name[10];
	/* page write size in bytes */
	u8 page_size;
	/* device size in bytes */
	u16 size_bytes;
};

#endif /* _EEPROM_AT24_H */
