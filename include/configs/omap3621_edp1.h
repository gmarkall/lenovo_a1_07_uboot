/*
 * (C) Copyright 2010 MM Solutions
 *
 * Configuration settings for the 3621 EDP1 board
 *
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* For EPXX */
#define CONFIG_EPXX_DDR_512MB	1
#define CONFIG_GPS_DEBUG_UART_SWITCH	1
#define CONFIG_CHANGE_INAND_MMC_SCAN_INDEX	1
#define CONFIG_BOOTUP_LOGO		1
#define CONFIG_SHOW_BATTERY_STATE 1 		/* <--LH_SWRD_CL1_Mervins@2011.06.03--> */
#define CONFIG_SHARE_REGION		1
#define CONFIG_UPGRADE_MEM		1
#ifdef CONFIG_EPXX_DDR_512MB
#define LCD_FB_PHY_ADDR				(0xa0000000 - 0x200000)
#define LCD_LOGO_PHY_ADDR			(LCD_FB_PHY_ADDR - 0x200000) /* <--LH_SWRD_CL1_Mervins@2011.05.30--> */
#define LCD_FB_CHARGING_MESSAGE_WARNING_ADDR (LCD_LOGO_PHY_ADDR - 0x200000) /* <--LH_SWRD_CL1_Mervins@2011.06.03--> */
#else
#define LCD_FB_PHY_ADDR				(0x90000000 - 0x200000)
#define LCD_LOGO_PHY_ADDR			(LCD_FB_PHY_ADDR - 0x200000) /* <--LH_SWRD_CL1_Mervins@2011.05.30--> */
#define LCD_FB_CHARGING_MESSAGE_WARNING_ADDR (LCD_LOGO_PHY_ADDR - 0x200000) /* <--LH_SWRD_CL1_Mervins@2011.06.03--> */
#endif

//20101228_Peter ++
#define	SCAN_BLOCKS	(160*1024*1024)	//160M
//20101228_Peter --

#define CONFIG_BOOT_RAMDISK_INITRD 1
/*
 * High Level Configuration Options
 */
#define CONFIG_ARMCORTEXA8	1    /* This is an ARM V7 CPU core */
#define CONFIG_OMAP		1    /* in a TI OMAP core */
#define CONFIG_OMAP36XX		1    /* which is a 36XX */
#define CONFIG_OMAP34XX		1    /* reuse the 34XX setup */
#define CONFIG_OMAP3621_MM	1
#define CONFIG_OMAP3430		1    /* which is in a 3430 */
#define CONFIG_3630ZOOM3	1    /* working on Zoom3 board */
#define CONFIG_3430ZOOM2	1    /* reuse Zoom2 setup */
#define CONFIG_3621EDP1		1    /* working with EDP1 */
#define CONFIG_BOARD_REVISION   1    /* Board revision */

//&*&*&*20101201_Peter ++
#define CONFIG_NEW_DISPLAY_BOARD_INFO	1

#define CONFIG_STORAGE_EMMC			1		/* must be set emmc */	
//#define CONFIG_STORAGE_NAND			1
#define CONFIG_TWL4030_KEYPAD		1
#define CONFIG_TWL4030_USB			1
#define CONFIG_TWL4030_MADC			1
//#define CONFIG_TWL4030_MADC_VBAT    1	/* <--LH_SWRD_CL1_Mervins@2011.05.07--> */
//#define CONFIG_MAX17043_GAUGE       1
#define CONFIG_BQ27541_VBAT			1


#define CONFIG_DISPLAY_BOARDINFO	1
#define CFG_CONSOLE_INFO_QUIET		1   
//&*&*&*20101201_Peter --

//&*&*&*HC1_20110428, enable pwr key long press function 
#define CONFIG_PWRKEY_LONGPRESS	1
//&*&*&*HC2_20110428, enable pwr key long press function 

#define CONFIG_OFF_PADCONF	(1)  /* Enable OFFMODE pad configuration
				      * Make sure it is defined, value is
				      * is insignificant
				      */

#undef CONFIG_DSP_MEASURMENT

/* Enable TFT Display 
 * WARNING: This is incompatible with the software epaper framebuffer driver!
 */
#undef CFG_TFT_DISPLAY

#include <asm/arch/cpu.h>        /* get chip and board defs */

/* Clock Defines */
#define V_OSCK			26000000  /* Clock output from T2 */

#define V_SCLK			(V_OSCK >> 1)

#define PRCM_CLK_CFG2_400MHZ	1    /* VDD2=1.2v - 200MHz DDR */
#define PRCM_PCLK_OPP2		1    /* ARM=500MHz - VDD1=1.20v */

/* PER clock options, only uncomment 1 */
/* Uncomment to run PER M2 at 2x 96MHz */
/* #define CONFIG_PER_M2_192 */
/* Uncomment to run PER SGX at 192MHz */
/* #define CONFIG_PER_SGX_192 */

/* Uncommend to run PER at 2x 96MHz */

#undef CONFIG_USE_IRQ                 /* no support for IRQs */
#define CONFIG_MISC_INIT_R		1

#define CONFIG_CMDLINE_TAG		1    /* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_REVISION_TAG		1

/*
 * Size of malloc() pool
 */
#define CFG_ENV_SIZE		SZ_128K    /* Total Size Environment Sector */
#define CFG_MALLOC_LEN		(CFG_ENV_SIZE + SZ_128K)
#define CFG_GBL_DATA_SIZE	128  /* bytes reserved for initial data */

//&*&*&*20101201_Peter ++
#if defined(CONFIG_STORAGE_EMMC) || defined(CONFIG_STORAGE_NAND)
#define CFG_FLASH_BASE    		0x0
#define CFG_ENV_SECT_SIZE 		SZ_128K
#define CFG_ENV_OFFSET	 		0x40000
#define CFG_ENV_ADDR			0x40000
#endif
//&*&*&*20101201_Peter --

/*
 * Hardware drivers
 */

/*
 * NS16550 Configuration.
 */
#define V_NS16550_CLK		(48000000)  /* 48 Mhz */

#define CFG_NS16550
#define CFG_NS16550_SERIAL
#define CFG_NS16550_REG_SIZE	(-4)
#define CFG_NS16550_CLK		V_NS16550_CLK
#define CONFIG_BAUDRATE		115200
#define CFG_BAUDRATE_TABLE	{115200}

#define CFG_NS16550_COM1	OMAP34XX_UART1
/*#define CFG_NS16550_COM2	OMAP34XX_UART2*/
#define CFG_NS16550_COM3	OMAP34XX_UART3

/*
 * select serial console configuration
 */
#define CONFIG_SERIAL1		1    /* UART3 on board */
#define CONFIG_CONS_INDEX	1

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_MMC		1
#define CFG_MMC_BASE		0xF0000000
#define CONFIG_DOS_PARTITION	1
/* External (slot 1) or internal iNAND/eMMC (slot 2) MMC to use */
#define CONFIG_MMC_SLOT		1

//&*&*&*20101201_Peter ++
#if defined(CONFIG_STORAGE_EMMC)
#define CFG_MONITOR_BASE       CFG_EMMC_FLASH_BASE /* Monitor at start of flash */
/* eMMC Variables */
#define CFG_ENV_IS_IN_EMMC    1
#define ENV_IS_VARIABLE	    	1

#define CFG_EMMC_FLASH_BASE    		CFG_FLASH_BASE
#define CFG_EMMC_ENV_SECT_SIZE 		SZ_128K
#define CFG_EMMC_ENV_OFFSET	 		CFG_ENV_OFFSET
#define CFG_EMMC_ENV_ADDR			CFG_ENV_ADDR
#endif
//&*&*&*20101201_Peter --

#define C_MSK (CFG_CMD_IMLS | CFG_CMD_FLASH | CFG_CMD_NET)

/* Config CMD*/
//&*&*&*20101201_Peter ++
#if 0
#define CONFIG_COMMANDS		((CFG_CMD_I2C | CONFIG_CMD_DFL |\
				  CFG_CMD_FAT | CFG_CMD_MMC) & ~(C_MSK))
#else
#if defined (CONFIG_STORAGE_EMMC) && defined (CONFIG_STORAGE_NAND)
#define CONFIG_COMMANDS		((CFG_CMD_I2C | CONFIG_CMD_DFL | CFG_CMD_DHCP |\
				  CFG_CMD_FAT | CFG_CMD_MMC | CFG_CMD_NAND ) & ~(C_MSK))
#elif defined (CONFIG_STORAGE_NAND) 
#define CONFIG_COMMANDS		((CFG_CMD_I2C | CONFIG_CMD_DFL | CFG_CMD_DHCP |\
				  CFG_CMD_FAT | CFG_CMD_NAND ) & ~(C_MSK))				  
#else
#define CONFIG_COMMANDS		((CFG_CMD_I2C | CONFIG_CMD_DFL | CFG_CMD_DHCP |\
				  CFG_CMD_FAT | CFG_CMD_MMC ) & ~(C_MSK))
#endif
#endif /* End #if 0 */
//&*&*&*20101201_Peter --

//&*&*&*SJ1_20100820, Add Fastboot define.
/* Fastboot variables */
#if defined (CONFIG_STORAGE_EMMC) || defined (CONFIG_STORAGE_NAND)
#define CONFIG_FASTBOOT	        					1    /* Using fastboot interface */
#define CFG_FASTBOOT_MMC_NO               1    /* Use eMMC */
#define CFG_FASTBOOT_TRANSFER_BUFFER (PHYS_SDRAM_1 + SZ_16M)
#define CFG_FASTBOOT_TRANSFER_BUFFER_SIZE (SZ_256M - SZ_16M)
#define CFG_FASTBOOT_PREBOOT_KEYS         1
#define CFG_FASTBOOT_PREBOOT_KEY1         0x00 /* unused */ /* <--LH_SWRD_CL1_Mervins@2011.05.27--> */
#define CFG_FASTBOOT_PREBOOT_KEY2         0x00 /* unused */
#define CFG_FASTBOOT_PREBOOT_INITIAL_WAIT (0)
#define CFG_FASTBOOT_PREBOOT_LOOP_MAXIMUM (1)
#define CFG_FASTBOOT_PREBOOT_LOOP_WAIT    (0)

#define CFG_INAND_UPDATE_MODE_PREBOOT_KEY1	 0x09	/*  0x09 volume down */ /* <--LH_SWRD_CL1_Mervins@2011.05.27--> */
#define CFG_INAND_UPDATE_MODE_PREBOOT_KEY2	 0x00

#endif
//&*&*&*SJ2_20100820, Add Fastboot define.

/* Undefine to save space */
//#define CFG_HUSH_PARSER		1

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <cmd_confdefs.h>

#if (CONFIG_COMMANDS & CFG_CMD_I2C)
  #define CFG_I2C_SPEED			100
  #define CFG_I2C_SLAVE			1
  #define CFG_I2C_BUS			0
  #define CFG_I2C_BUS_SELECT		1
  #define CONFIG_DRIVER_OMAP34XX_I2C	1
#endif

//&*&*&*SJ1_20100820, Add NAND Info 
#if defined(CONFIG_STORAGE_NAND)
/*
 *  Board NAND Info.
 */
#define CFG_NAND_ADDR NAND_BASE  /* physical address to access nand*/
#define CFG_NAND_BASE NAND_BASE  /* physical address to access nand at CS0*/
#define CFG_NAND_WIDTH_16

#define CFG_MAX_NAND_DEVICE      1 /* Max number of NAND devices */
#define SECTORSIZE               512

/* To use the 256/512 byte s/w ecc define CFG_SW_ECC_(256/512) */
/* Use the 512 byte ROM CODE HW ecc */
#define CFG_HW_ECC_ROMCODE       1
#define CFG_NAND_YAFFS_WRITE     1

#define NAND_ALLOW_ERASE_ALL
#define ADDR_COLUMN              1
#define ADDR_PAGE                2
#define ADDR_COLUMN_PAGE         3

#define NAND_ChipID_UNKNOWN      0x00
#define NAND_MAX_FLOORS          1
#define NAND_MAX_CHIPS           1
#define NAND_NO_RB               1
#define CFG_NAND_WP
#else
#define NAND_MAX_CHIPS			0
#endif
//&*&*&*SJ2_20100820, Add NAND Info

/* Environment information */

#if CFG_HUSH_PARSER
  #define CONFIG_EXTRA_ENV_SETTINGS \
	"loadaddr=0x81000000\0" \
	"console=ttyO0,115200n8\0" \
	"optargs=\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=/dev/mmcblk0p2 rw " \
		"rootwait init=/init\0" \
	"loadbootscript=fatload mmc 0 ${loadaddr} boot.scr\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"autoscr ${loadaddr}\0" \
	"loaduimage=fatload mmc 0 ${loadaddr} uImage\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"run loaduimage;" \
		"bootm ${loadaddr}\0" \

  #define CONFIG_BOOTCOMMAND \
	"mmc init 0; " \
	"if run loadbootscript; then " \
		"run bootscript; " \
	"else " \
		"run mmcboot; " \
	"fi"

  #define CFG_PROMPT_HUSH_PS2	V_PROMPT
#else
  #define CONFIG_EXTRA_ENV_SETTINGS \
	"debug=0\0" /* <--LH_SWRD_CL1_Mervins@2011.05.19--> */
  #define CONFIG_BOOTARGS "console=ttyO0,115200n8 root=/dev/mmcblk0p2 rw rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:4M"
  #define CONFIG_BOOTCOMMAND "mmcinit 0; fatload mmc 0 0x81c00000 uImage; bootm 0x81c00000"
//&*&*&*20101201_Peter ++
#if defined(CONFIG_STORAGE_EMMC) || defined(CONFIG_STORAGE_NAND)

//&*&*&*20110620_SJ1
#if defined (CONFIG_BOOT_RAMDISK_INITRD)
	#define CONFIG_INITRAMFS_BOOTARGS_TFT_EMMC_BOOT	"console=ttyO0,115200n8 rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyO0"
	#define CONFIG_INITRAMFS_BOOTARGS_TFT_EMMC_GPS_UART_BOOT "console=ttyVSP1,115200n8 rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyVSP1"
	#define CONFIG_INITRAMFS_BOOTCOMMAND_EMMC  "mmcinit 1; mmc 1 read 82000000 300000 310000; mmc 1 read 83000000 1800000 100000; bootm 82000000 83000000"
#endif
//&*&*&*20110620_SJ2

  /** TFT **/
  #define CONFIG_BOOTARGS_TFT_NAND_BOOT	"console=ttyO0,115200n8 root=/dev/mtdblock6 rw rootfstype=yaffs2 init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyO0"
  #define CONFIG_BOOTARGS_TFT_EMMC_BOOT	"console=ttyO0,115200n8 root=/dev/mmcblk0p1 rw rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyO0"
#if (CONFIG_CHANGE_INAND_MMC_SCAN_INDEX)
	#define CONFIG_BOOTARGS_TFT_SD1_BOOT "console=ttyO0,115200n8 root=/dev/mmcblk1p2 rw rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyO0"
#else
	#define CONFIG_BOOTARGS_TFT_SD1_BOOT "console=ttyO0,115200n8 root=/dev/mmcblk0p2 rw rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyO0"
#endif /* End (CONFIG_CHANGE_INAND_MMC_SCAN_INDEX) */

  /** TFT GPS_UART **/
  #define CONFIG_BOOTARGS_TFT_NAND_GPS_UART_BOOT	"console=ttyO2,115200n8 root=/dev/mtdblock6 rw rootfstype=yaffs2 init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyO2"
  #define CONFIG_BOOTARGS_TFT_EMMC_GPS_UART_BOOT "console=ttyO2,115200n8 root=/dev/mmcblk0p1 rw rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyO2"
#if (CONFIG_CHANGE_INAND_MMC_SCAN_INDEX)
	#define CONFIG_BOOTARGS_TFT_SD1_GPS_UART_BOOT  "console=ttyVSP1,115200n8 root=/dev/mmcblk1p2 rw rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyVSP1"
#else
	#define CONFIG_BOOTARGS_TFT_SD1_GPS_UART_BOOT  "console=ttyVSP1,115200n8 root=/dev/mmcblk0p2 rw rootwait init=/init videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:6M androidboot.console=ttyVSP1"
#endif /* End (CONFIG_CHANGE_INAND_MMC_SCAN_INDEX) */

	/** BOOTARGS **/
	#define CONFIG_BOOTCOMMAND_NAND  "nand read 82000000 300000 380000; bootm 82000000"
//	#define CONFIG_BOOTCOMMAND_SD1   "mmcinit 0; fatload mmc 0 82000000 boot.scr; echo Running bootscript from mmc ...; autoscr 82000000"
	#define CONFIG_BOOTCOMMAND_SD1   "mmcinit 0; fatload mmc 0 0x82000000 uImage; bootm 0x82000000"
	#define CONFIG_BOOTCOMMAND_EMMC  "mmcinit 1; mmc 1 read 82000000 300000 310000; bootm 82000000"
	
	/** Update Mode Bootargs & Bootcommand **/
	#define CONFIG_BOOTARGS_UPDATE_MODE "console=ttyO0,115200n8 videoout=omap24xxvout omap_vout.video1_numbuffers=6 omap_vout.vid1_static_vrfb_alloc=y omapfb.vram=0:4M androidboot.console=ttyO0"
	#define CONFIG_BOOTCOMMAND_NAND_UPDATE_MODE "mclear 80800000 1000000; nand read 82000000 1000000 300000; bootm 82000000"
	//#define CONFIG_BOOTCOMMAND_EMMC_UPDATE_MODE "mmcinit 1; mclear 80800000 1000000;mmc 1 read 82000000 C00000 800000; bootm 82000000"
	#define CONFIG_BOOTCOMMAND_EMMC_UPDATE_MODE "mmcinit 1; mclear 80800000 1000000;mmc 1 read 82000000 300000 800000;mmc 1 read 83000000 c00000 400000; bootm 82000000 83000000" /* <--LH_SWRD_CL1_Mervins@2011.05.27--> */
#endif /* End #if defined(CONFIG_STORAGE_EMMC) | defined(CONFIG_STORAGE_NAND) */
//&*&*&*20101201_Peter --
#endif	/* CFG_HUSH_PARSER */

#define CONFIG_BOOTDELAY	1

#define CONFIG_AUTO_COMPLETE     1
/*
 * Miscellaneous configurable options
 */
//#define V_PROMPT                 "OMAP36XX EDP1 # "
#define V_PROMPT                 "EP Series # "

#define CFG_LONGHELP             /* undef to save memory */
#define CFG_PROMPT               V_PROMPT
#define CFG_CBSIZE               256  /* Console I/O Buffer Size */
/* Print Buffer Size */
#define CFG_PBSIZE               (CFG_CBSIZE+sizeof(CFG_PROMPT)+16)
#define CFG_MAXARGS              16          /* max number of command args */
#define CFG_BARGSIZE             CFG_CBSIZE  /* Boot Argument Buffer Size */

//&*&*&*20101201_Peter ++
#if 0
#define CFG_MEMTEST_START        (OMAP34XX_SDRC_CS0)  /* memtest works on */
#define CFG_MEMTEST_END          (OMAP34XX_SDRC_CS0+SZ_31M)
#else
#ifdef CONFIG_EPXX_DDR_512MB
#define CFG_MEMTEST_START        (OMAP34XX_SDRC_CS0+SZ_16M)  /* memtest works on */
#define CFG_MEMTEST_END          (OMAP34XX_SDRC_CS0+0xC800000+SZ_256M)
#else
#define CFG_MEMTEST_START        (OMAP34XX_SDRC_CS0+SZ_16M)  /* memtest works on */
#define CFG_MEMTEST_END          (OMAP34XX_SDRC_CS0+0xC800000)
#endif

#endif
//&*&*&*20101201_Peter --

#undef	CFG_CLKS_IN_HZ           /* everything, incl board info, in Hz */

#define CFG_LOAD_ADDR            (OMAP34XX_SDRC_CS0) /* default load address */

/* 2430 has 12 GP timers, they can be driven by the SysClk (12/13/19.2) or by
 * 32KHz clk, or from external sig. This rate is divided by a local divisor.
 */
#define V_PVT                    7

#define CFG_TIMERBASE            OMAP34XX_GPT2
#define CFG_PVT                  V_PVT  /* 2^(pvt+1) */
#define CFG_HZ                   ((V_SCLK)/(2 << CFG_PVT))

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	SZ_128K /* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	SZ_4K   /* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	SZ_4K   /* FIQ stack */
#endif

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	2       /* CS1 may or may not be populated */
#define PHYS_SDRAM_1		OMAP34XX_SDRC_CS0
#define PHYS_SDRAM_1_SIZE	SZ_32M            /* at least 32 meg */
#define PHYS_SDRAM_2		OMAP34XX_SDRC_CS1

/* SDRAM Bank Allocation method */
/*#define SDRC_B_R_C		1 */
/*#define SDRC_B1_R_B0_C	1 */
#define SDRC_R_B_C		1


/* No NAND on EDP */
//#define CFG_ENV_IS_NOWHERE	1

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */

/* **** PISMO SUPPORT *** */

/* Configure the PISMO */
/** REMOVE ME - REQUIRES FIXING OF MEM.C SOURCE ***/
//&*&*&*20101201_Peter ++
#if defined(CONFIG_STORAGE_NAND)
#define PISMO1_NOR_SIZE_SDPV2	GPMC_SIZE_128M
#define PISMO1_NOR_SIZE		GPMC_SIZE_64M

#define PISMO1_NAND_SIZE	GPMC_SIZE_128M
#define PISMO1_ONEN_SIZE	GPMC_SIZE_128M
#endif
//&*&*&*20101201_Peter --
#define DBG_MPDB_SIZE		GPMC_SIZE_16M
#define PISMO2_SIZE		0
#define SERIAL_TL16CP754C_SIZE	GPMC_SIZE_16M

#define CFG_MAX_FLASH_SECT	(520)		/* max number of sectors on one chip */
#define CFG_MAX_FLASH_BANKS      2		/* max number of flash banks */
#define CFG_MONITOR_LEN		SZ_256K 	/* Reserve 2 sectors */

#define PHYS_FLASH_SIZE_SDPV2	SZ_128M
#define PHYS_FLASH_SIZE		SZ_32M

//&*&*&*SJ1_20100820, modify NAND environment offset.
#if defined(CONFIG_STORAGE_NAND)
#define CFG_FLASH_BASE		boot_flash_base
#define PHYS_FLASH_SECT_SIZE	boot_flash_sec
/* Dummy declaration of flash banks to get compilation right */
#define CFG_FLASH_BANKS_LIST	{0, 0}

#define CFG_MONITOR_BASE	CFG_FLASH_BASE /* Monitor at start of flash */

#if 1
#define CFG_ENV_IS_IN_NAND	1
#define ENV_IS_VARIABLE			1
#else
/* No NAND on EDP */
#define CFG_ENV_IS_NOWHERE	1
#endif

#ifdef CONFIG_OPTIONAL_NOR_POPULATED
# define CFG_ENV_IS_IN_FLASH	1
#endif

#define SMNAND_ENV_OFFSET	0x20000 /* environment starts here  */
#define CFG_ENV_SECT_SIZE	boot_flash_sec
#define CFG_ENV_OFFSET		boot_flash_off
#define CFG_ENV_ADDR		boot_flash_env_addr
#endif /* End CONFIG_STORAGE_NAND */
//&*&*&*SJ2_20100820, modify NAND environment offset.

/*-----------------------------------------------------------------------
 * CFI FLASH driver setup
 */
#define CFG_NO_FLASH			1   /* Disable NOR Flash support */
//&*&*&*20101201_Peter ++
#if defined(CONFIG_STORAGE_NAND)
#ifndef __ASSEMBLY__
extern unsigned int nand_cs_base;
extern unsigned int boot_flash_base;
extern volatile unsigned int boot_flash_env_addr;
extern unsigned int boot_flash_off;
extern unsigned int boot_flash_sec;
extern unsigned int boot_flash_type;
#endif

#define WRITE_NAND_COMMAND(d, adr) __raw_writew(d, (nand_cs_base + GPMC_NAND_CMD))
#define WRITE_NAND_ADDRESS(d, adr) __raw_writew(d, (nand_cs_base + GPMC_NAND_ADR))
#define WRITE_NAND(d, adr) __raw_writew(d, (nand_cs_base + GPMC_NAND_DAT))
#define READ_NAND(adr) __raw_readw((nand_cs_base + GPMC_NAND_DAT))

/* Other NAND Access APIs */
#define NAND_WP_OFF()  do {*(volatile u32 *)(GPMC_CONFIG) |= 0x00000010;} while(0)
#define NAND_WP_ON()  do {*(volatile u32 *)(GPMC_CONFIG) &= ~0x00000010;} while(0)
#define NAND_DISABLE_CE(nand)
#define NAND_ENABLE_CE(nand)
#define NAND_WAIT_READY(nand)	udelay(10)
#endif
//&*&*&*20101201_Peter --

/* Clock command */
#define CONFIG_CMD_CLOCK		1
#define CONFIG_CMD_CLOCK_INFO_CPU	1

//&*&*&*20101201_Peter ++
#define __raw_readl(a)	(*(volatile unsigned int *)(a))
#define BOOTING_TYPE_NONE			0x0		
#define BOOTING_TYPE_XIP			0x1
#define BOOTING_TYPE_NAND			0x2
#define BOOTING_TYPE_ONENAND	0x3
#define BOOTING_TYPE_DOC			0x4
#define BOOTING_TYPE_eMMC			0x5
#define BOOTING_TYPE_SD1			0x6
#define BOOTING_TYPE_UART			0x10
#define BOOTING_TYPE_HS_USB		0x11
//&*&*&*20101201_Peter --

/* <--LH_SWRD_CL1_Mervins@2011.05.06 */
/* add by mervins,for edit cmdline */
#define CONFIG_CMDLINE_EDITING
/* LH_SWRD_CL1_Mervins@2011.05.06--> */

/* <--LH_SWRD_CL1_Mervins@2011.09.01:for 3G version --> */
//#define CONFIG_CL1_3G_VERSION  

#endif /* __CONFIG_H */
