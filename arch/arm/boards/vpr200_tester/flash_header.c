#include <common.h>
#include <mach/imx-flash-header.h>
#include <mach/imx-regs.h>

extern void exception_vectors(void);

/*
 * You can enable this at compile time and set TEXT_BASE to 0x1000d000
 * Then, send the image via the iROM bootloader and execute from internal
 * RAM. (note internal RAM size is 128K so barebox image needs to be less
 * than 0x13000 bytes). (Note also stack and heap default to be before 
 * TEXT_BASE. So far, used 0x4000 and 0x8000 respecively, OK)
 *
 * This header must be at the download position
 */
#define RUNNABLE_FROM_IROM_BOOTLOADER 0

#if RUNNABLE_FROM_IROM_BOOTLOADER
struct imx_flash_header __flash_header_start test_flash_header = {
	.app_code_jump_vector	=((unsigned int)&exception_vectors),
	.app_code_barker	= 0,
	.app_code_csf		= 0,
	.dcd_ptr_ptr		= TEXT_BASE + offsetof(struct imx_flash_header, dcd),
	.super_root_key		= 0,
	.dcd			= 0,
	.app_dest		= 0,
	.dcd_barker		= 0,
	.dcd_block_len	= 0,
};
#else
void __naked __flash_header_start go(void)
{
	__asm__ __volatile__("b exception_vectors\n");
}
#endif

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

/*
 * This will poke the UART1 registers to output a string
 * as the DCD entries are processed.
 */
#define FLASH_HEADER_UART_TX_STARTUP 1

struct imx_dcd_entry __dcd_entry_0x0400 sd_dcd_entry[] = {
#if FLASH_HEADER_UART_TX_STARTUP
	{ .ptr_type = 4, .addr = 0x43F90080, .val = 0x00000001, }, //UCR1
	{ .ptr_type = 4, .addr = 0x43F90084, .val = 0x00004025, },
	{ .ptr_type = 4, .addr = 0x43F90090, .val = 0x00000a81, }, //UFCR
	{ .ptr_type = 4, .addr = 0x43F900a4, .val = 0x0000047f, }, // UBIR
	{ .ptr_type = 4, .addr = 0x43F900a8, .val = 0x0000f423, }, // UBIR
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x0000000d, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x0000000a, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x00000056, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x00000050, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x00000052, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x00000032, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x00000030, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x00000030, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x0000000d, }, // tx
	{ .ptr_type = 4, .addr = 0x43F90040, .val = 0x0000000a, }, // tx
#endif
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
