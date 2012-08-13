#include <common.h>
#include <mach/imx-flash-header.h>
#include <mach/imx-regs.h>

extern void exception_vectors(void);

void __naked __flash_header_start go(void)
{
	__asm__ __volatile__("b exception_vectors\n");
}

/*
 * This define can be set to 0 to skip initialization of the first mem
 * bank.
 * This is useful for testing boards which don't boot with suspect RAM,
 * (Use this in conjunction with setting TEXT_BASE to 0x97f00000)
 */
#define USE_MEM_BANK_1 1

struct imx_dcd_entry __dcd_entry_0x1000 nor_dcd_entry[] = {
	{ .ptr_type = 4, .addr = 0xB8001010, .val = 0x00000304, },
	/* LPDDR delay line soft reset */
	{ .ptr_type = 4, .addr = 0xB8001010, .val = 0x0000030C, },
	/* bank1 */
#if USE_MEM_BANK_1
	{ .ptr_type = 4, .addr = 0xB8001004, .val = 0x007ffc2f, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0x92220000, },
	{ .ptr_type = 4, .addr = 0x80000400, .val = 0x12345678, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0xA2220000, },
	{ .ptr_type = 4, .addr = 0x80000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0x80000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0xB2220000, },
	{ .ptr_type = 1, .addr = 0x80000233, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x82000780, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x82000400, .val = 0xda, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0x82226080, },
#else
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0x00002000, },
#endif
	/* bank2 */
	{ .ptr_type = 4, .addr = 0xB800100C, .val = 0x007ffc2f, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0x92220000, },
	{ .ptr_type = 4, .addr = 0x90000400, .val = 0x12345678, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0xA2220000, },
	{ .ptr_type = 4, .addr = 0x90000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0x90000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0xB2220000, },
	{ .ptr_type = 1, .addr = 0x90000233, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x92000780, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x92000400, .val = 0xda, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0x82226080, },
	{ .ptr_type = 4, .addr = 0xB8001010, .val = 0x00000304, },
};

/*
 * NOR is not automatically copied anywhere by the boot ROM
 */
struct imx_flash_header __flash_header_0x1000 nor_flash_header = {
	.app_code_jump_vector	= IMX_CS0_BASE + ((unsigned int)&exception_vectors - TEXT_BASE),
	.app_code_barker	= APP_CODE_BARKER,
	.app_code_csf		= 0,
	.dcd_ptr_ptr		= IMX_CS0_BASE + 0x1000 + offsetof(struct imx_flash_header, dcd),
	.super_root_key		= 0,
	.dcd			= IMX_CS0_BASE + 0x1000 + offsetof(struct imx_flash_header, dcd_barker),
	.app_dest		= IMX_CS0_BASE,
	.dcd_barker		= DCD_BARKER,
	.dcd_block_len		= sizeof(nor_dcd_entry),
};

unsigned long __image_len_0x1000 nor_barebox_len = 0x40000;

struct imx_dcd_entry __dcd_entry_0x0400 sd_dcd_entry[] = {
	{ .ptr_type = 4, .addr = 0xB8001010, .val = 0x00000304, },
	/* LPDDR delay line soft reset */
	{ .ptr_type = 4, .addr = 0xB8001010, .val = 0x0000030C, },
	/* bank1 */
#if USE_MEM_BANK_1
	{ .ptr_type = 4, .addr = 0xB8001004, .val = 0x007ffc2f, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0x92220000, },
	{ .ptr_type = 4, .addr = 0x80000400, .val = 0x12345678, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0xA2220000, },
	{ .ptr_type = 4, .addr = 0x80000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0x80000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0xB2220000, },
	{ .ptr_type = 1, .addr = 0x80000233, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x82000780, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x82000400, .val = 0xda, },
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0x82226080, },
#else
	{ .ptr_type = 4, .addr = 0xB8001000, .val = 0x00002000, },
#endif
	/* bank2 */
	{ .ptr_type = 4, .addr = 0xB800100C, .val = 0x007ffc2f, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0x92220000, },
	{ .ptr_type = 4, .addr = 0x90000400, .val = 0x12345678, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0xA2220000, },
	{ .ptr_type = 4, .addr = 0x90000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0x90000000, .val = 0x87654321, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0xB2220000, },
	{ .ptr_type = 1, .addr = 0x90000233, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x92000780, .val = 0xda, },
	{ .ptr_type = 1, .addr = 0x92000400, .val = 0xda, },
	{ .ptr_type = 4, .addr = 0xB8001008, .val = 0x82226080, },
	{ .ptr_type = 4, .addr = 0xB8001010, .val = 0x00000304, },
};

struct imx_flash_header __flash_header_0x0400 sd_flash_header = {
	.app_code_jump_vector	= TEXT_BASE + ((unsigned int)&exception_vectors - TEXT_BASE),
	.app_code_barker	= APP_CODE_BARKER,
	.app_code_csf		= 0,
	.dcd_ptr_ptr		= TEXT_BASE + 0x0400 + offsetof(struct imx_flash_header, dcd),
	.super_root_key		= 0,
	.dcd			= TEXT_BASE + 0x0400 + offsetof(struct imx_flash_header, dcd_barker),
	.app_dest		= TEXT_BASE,
	.dcd_barker		= DCD_BARKER,
	.dcd_block_len		= sizeof(sd_dcd_entry),
};

unsigned long __image_len_0x0400 sd_barebox_len = 0x40000;
