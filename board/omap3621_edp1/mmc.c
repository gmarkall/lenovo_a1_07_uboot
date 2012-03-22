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
#define MMCSD_SECTOR_SIZE	(512)

/*
-------------------------------------------------------------------------
| Revision | Date       | Description                                   |
-------------------------------------------------------------------------
|  2.0     | 2011-06-20 | 1. Change emmc_raw_header section address (0x20000)
------------------------| 2. Change xloader section address (0x20200)   |
-------------------------------------------------------------------------
| NAME             | START Addr  | LENGTH     | BYTE(s)  | Storage Type |
=========================================================================
== BINARY DATA
=========================================================================
| inand-mbr        | 0x0         | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------

                   GUID Partititon Table (GPT) Region

-------------------------------------------------------------------------
| inand-raw-header | 0x20000     | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------
| inand-xloader    | 0x20200     | 0x1FE00    | 128K-512B| Raw Data
-------------------------------------------------------------------------
* inand-env        | 0x40000     | 0x40000    | 256KB    | Raw Data
-------------------------------------------------------------------------
| inand-uboot      | 0x80000     | 0x180000   | 1.5 MB   | Raw Data 
-------------------------------------------------------------------------
| inand-kernel     | 0x300000    | 0x800000   | 8MB      | Raw Data
-------------------------------------------------------------------------
| inand-id         | 0xB00000    | 0x100000   | 1MB      | Raw Data
-------------------------------------------------------------------------
| inand-recovery   | 0xC00000    | 0x800000   | 4MB x 2  | RAMDISK
-------------------------------------------------------------------------
| inand-logo       | 0x1400000   | 0x400000   | 4MB      | Raw Data
-------------------------------------------------------------------------
| inand-ramdisk    | 0x1800000   | 0x300000   | 1.5MB x2 | RAMDISK
-------------------------------------------------------------------------
|                  |             |            |          |
-------------------------------------------------------------------------
| inand-batteryid  | 0x2DFFE00   | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------
| inand-version    | 0x2E00000   | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------
| inand-flags      | 0x2E00200   | 0x200      | 512B     | Raw Data
------------------------------------------------------------------------------
| NAME             | Starting Sector | Sector Length | BYTE(s)  | Filesystem |
==============================================================================
==	Partition Table: gpt
==============================================================================
Number  Start   End     Size    File system  Name      Flags
 1      50.0MB  250MB   200MB   ext3         system
 2      250MB   285MB   35.0MB  ext3         cache
 3      285MB   550MB   265MB   ext3         userdata
 4      550MB   750MB   200MB    ext3        secure
 5      750MB   *MB     *MB 	    fat32    storage
==============================================================================
*/

/*
-------------------------------------------------------------------------
| Revision | Date       | Description                                   |
-------------------------------------------------------------------------
|  1.2     | 2011-06-13 | 1. Added Battery Id section                   |
------------------------|	2. Modify Uboot env section adderss(0x40000)  |
-------------------------------------------------------------------------
| NAME             | START Addr  | LENGTH     | BYTE(s)  | Storage Type |
=========================================================================
== BINARY DATA
=========================================================================
| inand-mbr        | 0x0         | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------
| inand-xloader    | 0x200       | 0x7C00     | 31KB     | Raw Data
-------------------------------------------------------------------------
* inand-env        | 0x40000     | 0x40000    | 256KB    | Raw Data
-------------------------------------------------------------------------
| inand-uboot      | 0x80000     | 0x180000   | 1.5 MB   | Raw Data 
-------------------------------------------------------------------------
| inand-kernel     | 0x300000    | 0x800000   | 8MB      | Raw Data
-------------------------------------------------------------------------
| inand-id         | 0xB00000    | 0x100000   | 1MB      | Raw Data
-------------------------------------------------------------------------
| inand-recovery   | 0xC00000    | 0x800000   | 4MB x 2  | RAMDISK
-------------------------------------------------------------------------
| inand-logo       | 0x1400000   | 0x400000   | 4MB      | Raw Data
-------------------------------------------------------------------------
| inand-ramdisk    | 0x1800000   | 0x300000   | 1.5MB x2 | RAMDISK
-------------------------------------------------------------------------
|                  |             |            |          |
-------------------------------------------------------------------------
| inand-batteryid  | 0x2DFFE00   | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------
| inand-version    | 0x2E00000   | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------
| inand-flags      | 0x2E00200   | 0x200      | 512B     | Raw Data
------------------------------------------------------------------------------
| NAME             | Starting Sector | Sector Length | BYTE(s)  | Filesystem |
==============================================================================
==	MSDOS type partition
==============================================================================
| inand-filesystem | 96390           | 1028160       | 502MB    | EXT3
------------------------------------------------------------------------------
| inand-secure     | 1124550         | 2040255       | 996MB    | EXT3
------------------------------------------------------------------------------
| inand-storage    | 3164805         | 11904165      | 5812MB   | FAT32
------------------------------------------------------------------------------
****************************************************************************/

/*
-------------------------------------------------------------------------
| Revision | Date       | Description                                   |
-------------------------------------------------------------------------
|  1.0     | 3-Mar-2011 | Preliminary                                   |	
|------------------------------------------------------------------------
------------
|  v1.0    |
-------------------------------------------------------------------------
| NAME             | START Addr  | LENGTH     | BYTE(s)  | Storage Type |
=========================================================================
== BINARY DATA
=========================================================================
| inand-mbr        | 0x0         | 0x200      | 512B     | Raw Data
-------------------------------------------------------------------------
| inand-xloader    | 0x200       | 0x7C00     | 31KB     | Raw Data
-------------------------------------------------------------------------
| inand-env        | 0x20000     | 0x40000    | 256KB    | Raw Data
-------------------------------------------------------------------------
| inand-uboot      | 0x80000     | 0x180000   | 1.5 MB   | Raw Data 
-------------------------------------------------------------------------
| inand-kernel     | 0x300000    | 0x800000   | 8MB      | Raw Data
-------------------------------------------------------------------------
| inand-id         | 0xB00000    | 0x100000   | 1MB      | Raw Data
-------------------------------------------------------------------------
| inand-recovery   | 0xC00000    | 0x800000   | 4MB x 2  | RAMDISK
-------------------------------------------------------------------------
| inand-logo       | 0x1400000   | 0x400000   | 4MB      | Raw Data
-------------------------------------------------------------------------
| inand-ramdisk    | 0x1800000   | 0x300000   | 1.5MB x2 | RAMDISK
-------------------------------------------------------------------------
|                  |             |            |          |
-------------------------------------------------------------------------
| inand-version    | 0x2E00000   | 0x200      | 512      | Raw Data
-------------------------------------------------------------------------
| inand-flags      | 0x2E00200   | 0x200      | 512      | Raw Data
------------------------------------------------------------------------------
| NAME             | Starting Sector | Sector Length | BYTE(s)  | Filesystem |
==============================================================================
==	MSDOS type partition
==============================================================================
| inand-filesystem | 96390           | 1028160       | 502MB    | EXT3
------------------------------------------------------------------------------
| inand-secure     | 1124550         | 2040255       | 996MB    | EXT3
------------------------------------------------------------------------------
| inand-storage    | 3164805         | 11904165      | 5812MB   | FAT32
------------------------------------------------------------------------------
****************************************************************************/

void board_mmc_init(void)
{
#if (CONFIG_FASTBOOT)
	/* Partitons on EMMC preasent on OMAP36XX required for Fastboot*/
	fastboot_ptentry ptn[] = {
		{
			.name	= "inand-mbr",
			.start	= 0x0,
			.length = 0x200,   /* 512 Bytes */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND, 
		},
		{
			.name	= "inand-raw-header",
			.start	= 0x20000,
			.length = 0x200,   /* 512 Bytes */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-xloader",
			.start	= 0x20200,
			.length = 0x1FE00, /* 128K - 512B */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},		
		{
			.name	= "inand-env",
			.start	= 0x40000,
			.length = 0x40000,	/* 256 KB */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND, 
					
		},
		{
			.name	= "inand-uboot",
			.start	= 0x80000,
			.length = 0x180000, /* 1.5 MB */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-kernel",
			/* The real start */
			.start	= 0x300000,
			.length = 0x800000, /* 8MB */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-id",
			/* The real start */
			.start	= 0xB00000,
			.length = 0x100000, /* 1MB */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-recovery",
			/* The real start */
			.start	= 0xC00000,
			.length = 0x800000, /* 8MB */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-logo",
			/* The real start */
			.start	= 0x1400000,
			.length = 0x400000, /* 4MB */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-ramdisk",
			/* The real start */
			.start	= 0x1800000,
			.length = 0x300000, /* 3MB */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-batteryid",
			/* The real start */
			.start	= 0x2DFFE00,
			.length = 0x200, /* 512B */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-version",
			/* The real start */
			.start	= 0x2E00000,
			.length = 0x200, /* 512B */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
		{
			.name	= "inand-flags",
			/* The real start */
			.start	= 0x2E00200,
			.length = 0x200, /* 512B */
			.flags	= FASTBOOT_PTENTRY_FLAGS_WRITE_INAND,
		},
	};
	int i;
	for (i = 0; i < sizeof(ptn)/sizeof(fastboot_ptentry); i++)
		fastboot_flash_add_ptn(&ptn[i]);
#endif
}

int determine_boot_type(void)
{
        /* FIXME: Initialise LCD here */
        
        if (running_from_sd()) {
                lcd_putc('S');
                } else {
                lcd_putc('E'); }

        switch(get_boot_action()) {
        case BOOT_SD_NORMAL:
                setenv ("bootcmd", "setenv setbootargs setenv bootargs ${sdbootargs}; run setbootargs; mmcinit 0; fatload mmc 0:1 0x81000000 boot.img; booti 0x81000000");
                setenv ("altbootcmd", "run bootcmd"); // for sd boot altbootcmd is the same as bootcmd
                display_feedback(BOOT_SD_NORMAL);
                break;

        case BOOT_SD_RECOVERY:
                setenv ("bootcmd", "setenv setbootargs setenv bootargs ${sdbootargs}; run setbootargs; mmcinit 0; fatload mmc 0:1 0x81000000 recovery.img; booti 0x81000000");
                setenv ("altbootcmd", "run bootcmd"); // for sd boot altbootcmd is the same as bootcmd
                display_feedback(BOOT_SD_RECOVERY);
                break;

        case BOOT_SD_ALTBOOT:
                setenv ("bootcmd", "setenv setbootargs setenv bootargs ${sdbootargs}; run setbootargs; mmcinit 0; fatload mmc 0:1 0x81000000 altboot.img; booti 0x81000000");
                setenv ("altbootcmd", "run bootcmd"); // for sd boot altbootcmd is the same as bootcmd
                display_feedback(BOOT_SD_ALTBOOT);
                break;

        //actually, boot from boot+512K -- thanks bauwks!
        case BOOT_EMMC_NORMAL:
                setenv("bootcmd", "mmcinit 1; booti mmc1 boot 0x80000");
                display_feedback(BOOT_EMMC_NORMAL);
                break;

        //actually, boot from recovery+512K -- thanks bauwks!
        case BOOT_EMMC_RECOVERY:
                setenv("bootcmd", "mmcinit 1; booti mmc1 recovery 0x80000");
                display_feedback(BOOT_EMMC_RECOVERY);
                break;

        case BOOT_EMMC_ALTBOOT:  // no 512K offset, this is just a file.
                setenv ("bootcmd", "setenv setbootargs setenv bootargs ${emmcbootargs}; run setbootargs; mmcinit 1; fatload mmc 1:5 0x81000000 altboot.img; booti 0x81000000");
                setenv ("altbootcmd", "run bootcmd"); // for emmc altboot altbootcmd is the same as bootcmd
                display_feedback(BOOT_EMMC_ALTBOOT);
                break;

        case BOOT_FASTBOOT:
                display_feedback(BOOT_FASTBOOT);
                run_command("fastboot", 0);
                break;
        case INVALID:
        default:
                printf("Aborting boot!\n");
                return 1;
        }

        return 0;
}


}
