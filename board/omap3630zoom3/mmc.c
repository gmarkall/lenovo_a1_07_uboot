/*
 * (C) Copyright 2004-2009 Texas Instruments, <www.ti.com>
 * Kishore Kadiyala <kishore.kadiyala@ti.com>
 * Moiz Sonasath <m-sonasath@ti.com>
 * See file CREDITS for list of people who contributed to this
 * project.
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
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mem.h>

#if (CONFIG_FASTBOOT)
#include <fastboot.h>
#endif

void board_mmc_init(void)
{
#if (CONFIG_FASTBOOT)

	/* Partitons on EMMC preasent on OMAP4SDP required for Fastboot*/
	fastboot_ptentry ptn[10] = {
		{
			.name   = "mbr",
			.start  = 0x00, /*Sector Start */
			.length = 0x200, /*512B */
			.flags  = 0,
		},
		{
			.name   = "xloader",
			.start  = 0x100, /*Sector Start */
			.length = 0x0060000, /*384KB */
			.flags  = 0,
		},
		{
			.name   = "bootloader",
			.start  = 0x400, /*Sector Start */
			.length = 0x0060000, /* 384KB */
			.flags  = 0,
		},
		{
			.name   = "environment",
			.start  = 0x700,  /* Sector Start */
			.length = 0x0020000, /*128KB */
			.flags  = FASTBOOT_PTENTRY_FLAGS_WRITE_ENV,
		},
		{
			.name   = "kernel",
			.start  = 0x800,  /* Sector Start */
			.length = 0x00680000, /*6.5MB */
			.flags  = 0,
		},
		{
			.name   = "misc",
			.start  = 0x3C00, /*Sector Start */
			.length = 0x200, /*512B */
			.flags  = 0,
		},
		{
			.name   = "system",
			.start  = 0x3EC1,  /* Sector Start */
			.length = 0xC800000, /*200MB */
			.flags  = 0,
		},
		{
			.name   = "userdata",
			.start  = 0x69E5B,  /* Sector Start */
			.length = 0x2000000, /*32MB */
			.flags  = 0,
		},
		{
			.name   = "cache",
			.start  = 0x7D820,  /* Sector Start */
			.length = 0x2000000, /*32MB */
			.flags  = 0,
		},
		{
			.name   = "recovery",
			.start  = 0x911E5, /*Sector Start */
			.length = 0x12C00000, /*300MB */
			.flags  = 0,
		},

		/* Rest of the RAW Partitions can start from Sector start 0x1271E5 */
	};
	int i;
	for (i = 0; i < 10; i++)
		fastboot_flash_add_ptn(&ptn[i]);
#endif
}
