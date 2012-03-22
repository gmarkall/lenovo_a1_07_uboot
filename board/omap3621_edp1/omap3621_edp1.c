/*
 * (C) Copyright 2010
 * MM Solutions AD
 *
 * Heavily based on board/omap3630zoom3/omap3630zoom3.c
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <asm/arch/cpu.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <i2c.h>
#include <asm/mach-types.h>
#include <linux/mtd/nand_ecc.h>
#include <twl4030.h>

#if defined (CONFIG_SHARE_REGION)
#include <share_region.h>
#endif

int get_boot_type(void);
void v7_flush_dcache_all(int, int);
void l2cache_enable(void);
void setup_auxcr(int, int);
void eth_init(void *);

//&*&*&*20101201_Peter ++
#ifdef CONFIG_DISPLAY_BOARDINFO
#include <hardware_version.h>
enum BoardID_enum g_BoardID;
#endif /* End CONFIG_DISPLAY_BOARDINFO */

#ifdef CONFIG_STORAGE_EMMC
#include <environment.h>

extern uchar(*boot_env_get_char_spec) (int index);
extern int (*boot_env_init) (void);
extern int (*boot_saveenv) (void);
extern void (*boot_env_relocate_spec) (void);

/* EMMC */
extern uchar mmc_env_get_char_spec(int index);
extern int mmc_env_init(void);
extern int mmc_saveenv(void);
extern void mmc_env_relocate_spec(void);
extern char *mmc_env_name_spec;

extern char *env_name_spec;
/* update these elsewhere */
extern env_t *env_ptr;

#endif /* End #ifdef CONFIG_STORAGE_EMMC */
//&*&*&*20101201_Peter --

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init();		/* in SRAM or SDRAM, finish GPMC */

//&*&*&*SJ1_20100824, Intializing env functional pointers with eMMC.
#ifdef CONFIG_STORAGE_EMMC
	{
		int boot_device;
		boot_device = __raw_readl(0x480029c0) & 0xff;
		if ((boot_device == BOOTING_TYPE_eMMC)||(boot_device != BOOTING_TYPE_NAND)) {
			/* Intializing env functional pointers with eMMC */
			boot_env_get_char_spec = mmc_env_get_char_spec;
			boot_env_init = mmc_env_init;
			boot_saveenv = mmc_saveenv;
			boot_env_relocate_spec = mmc_env_relocate_spec;
			env_ptr = (env_t *) (CFG_EMMC_FLASH_BASE + CFG_EMMC_ENV_OFFSET);
			env_name_spec = mmc_env_name_spec;
		}
	}
#endif
//&*&*&*SJ2_20100824, Intializing env functional pointers with eMMC.

	gd->bd->bi_arch_number = MACH_TYPE_OMAP3621_EDP1; /* Linux mach id*/
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100); /* boot param addr */

	return 0;
}

//&*&*&*SJ1_20100824, Add the gpio_init function.
/*****************************************
 *****************************************/
int gpio_init (void)
{
/* <--LH_SWRD_CL1_Mervins@2011.05.06 */
	/* GPIO2 */
	gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
	sr32((u32)&gpio2_base->oe, 10, 1, 0);//GPIO42 CHRG EN /* <Mervins> */
	sr32((u32)&gpio2_base->cleardataout, 10, 1, 1);//set low
/* LH_SWRD_CL1_Mervins@2011.05.06--> */
	return 0;
}
//&*&*&*SJ2_20100824, Add the gpio_init function.

//&*&*&*SJ1_20101004, display board info.
#ifdef CONFIG_DISPLAY_BOARDINFO
enum BoardID_enum GetHardwareID(unsigned int val)
{
	enum BoardID_enum id;
	/* <--LH_SWRD_CL1_Mervins@2011.06.28:modify for hardware version.*/	
	if (val == 0) id = BOARD_ID_PRO; 	
	else if ((val > 350) &&(val <= 565))	id = BOARD_ID_DVT1;
	else if ((val > 565) &&(val <= 787))	id = BOARD_ID_WIFI_DVT2;
	else if ((val > 787) &&(val <= 1000))	id = BOARD_ID_WIFI_PVT;
	else if ((val > 1000)&&(val <= 1190))	id = BOARD_ID_WIFI_MP;
	else if ((val > 1190) &&(val <= 1290))	id = BOARD_ID_3G_DVT2;
	else if ((val > 1290) &&(val <= 1400))	id = BOARD_ID_3G_PVT;
	else if ((val > 1400)&&(val <= 1600))	id = BOARD_ID_3G_MP;
	else id = BOARD_VERSION_UNKNOWN; 
	/* LH_SWRD_CL1_Mervins@2011.06.28--> */
	return id;	
}

int checkboard(void)
{
	unsigned int ADCVaule = twl4030_get_ADCIN0_voltage();
	twl4030_vadc_onoff(0);
	g_BoardID = GetHardwareID(ADCVaule);	
	switch(g_BoardID)
	{/* <--LH_SWRD_CL1_Mervins@2011.06.28:for hardware version.*/
		case BOARD_ID_PRO:  printf("Board: PROTOTYPE (ADC=%dmV)\n", ADCVaule); break;
		case BOARD_ID_DVT1: printf("Board: DVT1(ADC=%dmV)\n", ADCVaule); break;
		case BOARD_ID_WIFI_DVT2: printf("Board: WIFI_DVT2(ADC=%dmV)\n", ADCVaule); break;
		case BOARD_ID_WIFI_PVT: printf("Board: WIFI_PVT (ADC=%dmV)\n", ADCVaule); break;
		case BOARD_ID_WIFI_MP: printf("Board: WIFI_MP (ADC=%dmV)\n", ADCVaule); break;
		case BOARD_ID_3G_DVT2: printf("Board: 3G_DVT2 (ADC=%dmV)\n", ADCVaule); break;
		case BOARD_ID_3G_PVT: printf("Board: 3G_PVT (ADC=%dmV)\n", ADCVaule); break;
		case BOARD_ID_3G_MP: printf("Board: 3G_MP (ADC=%dmV)\n", ADCVaule); break;
		default:            printf("Board: OMAP3621\n"); break;
	}
	/* LH_SWRD_CL1_Mervins@2011.06.28--> */
	return (0);
}
#endif
//&*&*&*SJ2_20101004, display board info.

//&*&*&*SJ1_20110520, Add check 3G modem exist GPIO status for EVT3. 
int check_3G_exist(void)
{
#if 0 
	gpio_t *gpio3_base = (gpio_t *)OMAP34XX_GPIO3_BASE;
	u32 value, reg;
	
	/** Get 3G-EXIST GPIO_93(EVT3) **/
	value = read_gpio_value((u32)&gpio3_base->datain, 29);
	reg = __raw_readl(0x48002108);	/* DSS_DATA23, GPIO_93 */
	reg &= ~(1<<20); /* Set PullDown selected. */
	__raw_writel(reg, 0x48002108);

	return (value);
#else
    return 1;
#endif
}
//&*&*&*SJ2_20110520, Add check 3G modem exist GPIO status for EVT3. 

//&*&*&*SJ1_20110520, Add check recovery flag. 
int check_recovery_flag(void)
{
#if 0
	char *pMem = (char *)0x82000000; 
	
	run_command ("mmc 1 read 82000000 2e00200 200", 0);
	if (strncmp(pMem, "CMD:recover", 11) == 0) {
		printf("%s\n", pMem);
		memset(pMem, 0x0, 512);
		run_command ("mmc 1 write 82000000 2e00200 200", 0);
		return 1;
	}
#else
	if (gRebootRecovery) {
		gRebootRecovery = 0;
		return 1;
	}
#endif	

	return 0;	
}
//&*&*&*SJ2_20110520, Add check recovery flag. 

/*****************************************
 * Routine: secure_unlock
 * Description: Setup security registers for access
 * (GP Device only)
 *****************************************/
void secure_unlock_mem(void)
{
	/* Permission values for registers -Full fledged permissions to all */
	#define UNLOCK_1 0xFFFFFFFF
	#define UNLOCK_2 0x00000000
	#define UNLOCK_3 0x0000FFFF

	/* Protection Module Register Target APE (PM_RT)*/
	__raw_writel(UNLOCK_1, RT_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_1, RT_READ_PERMISSION_0);
	__raw_writel(UNLOCK_1, RT_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, RT_ADDR_MATCH_1);

	__raw_writel(UNLOCK_3, GPMC_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, OCM_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, OCM_ADDR_MATCH_2);

	/* IVA Changes */
	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_1);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_1);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_2);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_2);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_2);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_3);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_3);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_3);

	__raw_writel(UNLOCK_1, SMS_RG_ATT0); /* SDRC region 0 public */
}


/**********************************************************
 * Routine: secureworld_exit()
 * Description: If chip is EMU and boot type is external
 *		configure secure registers and exit secure world
 *  general use.
 ***********************************************************/
void secureworld_exit(void)
{
	unsigned long i;

	/* configrue non-secure access control register */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 2":"=r" (i));
	/* enabling co-processor CP10 and CP11 accesses in NS world */
	__asm__ __volatile__("orr %0, %0, #0xC00":"=r"(i));
	/* allow allocation of locked TLBs and L2 lines in NS world */
	/* allow use of PLE registers in NS world also */
	__asm__ __volatile__("orr %0, %0, #0x70000":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 2":"=r" (i));

	/* Enable ASA and IBE in ACR register */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 1":"=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x50":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 1":"=r" (i));

	/* Exiting secure world */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 0":"=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x31":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 0":"=r" (i));
}

/**********************************************************
 * Routine: try_unlock_sram()
 * Description: If chip is GP/EMU(special) type, unlock the SRAM for
 *  general use.
 ***********************************************************/
void try_unlock_memory(void)
{
	int mode;
	int in_sdram = running_in_sdram();

	/* if GP device unlock device SRAM for general use */
	/* secure code breaks for Secure/Emulation device - HS/E/T*/
	mode = get_device_type();
	if (mode == GP_DEVICE) {
		secure_unlock_mem();
	}
	/* If device is EMU and boot is XIP external booting
	 * Unlock firewalls and disable L2 and put chip
	 * out of secure world
	 */
	/* Assuming memories are unlocked by the demon who put us in SDRAM */
	if ((mode <= EMU_DEVICE) && (get_boot_type() == 0x1F)
		&& (!in_sdram)) {
		secure_unlock_mem();
		secureworld_exit();
	}

	return;
}

/**********************************************************
 * Routine: s_init
 * Description: Does early system init of muxing and clocks.
 * - Called path is with SRAM stack.
 **********************************************************/
void s_init(void)
{
	int i;
	int external_boot = 0;
	int in_sdram = running_in_sdram();

	watchdog_init();

	external_boot = (get_boot_type() == 0x1F) ? 1 : 0;
	/* Right now flushing at low MPU speed. Need to move after clock init */
	v7_flush_dcache_all(get_device_type(), external_boot);

	try_unlock_memory();

	if (cpu_is_3410()) {
		/* Lock down 6-ways in L2 cache so that effective size of L2 is 64K */
		__asm__ __volatile__("mov %0, #0xFC":"=r" (i));
		__asm__ __volatile__("mcr p15, 1, %0, c9, c0, 0":"=r" (i));
	}

#ifndef CONFIG_ICACHE_OFF
	icache_enable();
#endif

#ifdef CONFIG_L2_OFF
	l2cache_disable();
#else
	l2cache_enable();
#endif
	set_muxconf_regs();
	delay(100);
	
	/* Writing to AuxCR in U-boot using SMI for GP/EMU DEV */
	/* Currently SMI in Kernel on ES2 devices seems to have an isse
	 * Once that is resolved, we can postpone this config to kernel
	 */
	setup_auxcr(get_device_type(), external_boot);

	prcm_init();

	per_clocks_enable();
}
//&*&*&*20110225_Peter ++
/*******************************************************
 * Routine: i2c_init_r
 * Description: Init omap i2c
 ********************************************************/
int i2c_init_r(void)
{
#ifdef CONFIG_DRIVER_OMAP34XX_I2C

	unsigned char data;

	i2c_init(CFG_I2C_SPEED, CFG_I2C_SLAVE);
	twl4030_power_reset_init();
	
#endif

	return (0);
}
//&*&*&*20110225_Peter --

/*******************************************************
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 ********************************************************/
int misc_init_r(void)
{
#ifdef CONFIG_DRIVER_OMAP34XX_I2C

	unsigned char data;

	i2c_init(CFG_I2C_SPEED, CFG_I2C_SLAVE);
	twl4030_power_reset_init();
	/* see if we need to activate the power button startup */
	char *s = getenv("pbboot");
	if (s) {
		/* figure out why we have booted */
		i2c_read(0x4b, 0x3a, 1, &data, 1);

		/* if status is non-zero, we didn't transition
		 * from WAIT_ON state
		 */
		if (data) {
			printf("Transitioning to Wait State (%x)\n", data);

			/* clear status */
			data = 0;
			i2c_write(0x4b, 0x3a, 1, &data, 1);

			/* put PM into WAIT_ON state */
			data = 0x01;
			i2c_write(0x4b, 0x46, 1, &data, 1);

			/* no return - wait for power shutdown */
			while (1) {;}
		}
		printf("Transitioning to Active State (%x)\n", data);

		/* turn on long pwr button press reset*/
		data = 0x40;
		i2c_write(0x4b, 0x46, 1, &data, 1);
		printf("Power Button Active\n");
	}
#endif
//&*&*&*20101201_Peter ++
	twl4030_usb_init();
	twl4030_keypad_init();
	twl4030_madc_init();
	twl4030_vadc_onoff(1);
//&*&*&*20101201_Peter --
	dieid_num_r();

	return (0);
}

/******************************************************
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 ******************************************************/
void wait_for_command_complete(unsigned int wd_base)
{
	int pending = 1;
	do {
		pending = __raw_readl(wd_base + WWPS);
	} while (pending);
}

/****************************************
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 *****************************************/
void watchdog_init(void)
{
	/* There are 3 watch dogs WD1=Secure, WD2=MPU, WD3=IVA. WD1 is
	 * either taken care of by ROM (HS/EMU) or not accessible (GP).
	 * We need to take care of WD2-MPU or take a PRCM reset.  WD3
	 * should not be running and does not generate a PRCM reset.
	 */

	sr32(CM_FCLKEN_WKUP, 5, 1, 1);
	sr32(CM_ICLKEN_WKUP, 5, 1, 1);
	wait_on_value(BIT5, 0x20, CM_IDLEST_WKUP, 5); /* some issue here */

	__raw_writel(WD_UNLOCK1, WD2_BASE + WSPR);
	wait_for_command_complete(WD2_BASE);
	__raw_writel(WD_UNLOCK2, WD2_BASE + WSPR);
}

/**********************************************
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 **********************************************/
int dram_init(void)
{
    #define NOT_EARLY 0
//20101215_Peter ++
//#define DEBUG
#if defined (CONFIG_EPXX_DDR_512MB)
	#define EARLY_INIT	1
#endif
//20101215_Peter --
    DECLARE_GLOBAL_DATA_PTR;
	unsigned int size0 = 0, size1 = 0;
	u32 mtype, btype;

	btype = get_board_type();
	mtype = get_mem_type();
#ifndef CONFIG_3430ZEBU
	/* fixme... dont know why this func is crashing in ZeBu */
	display_board_info(btype);
#endif
    /* If a second bank of DDR is attached to CS1 this is
     * where it can be started.  Early init code will init
     * memory on CS0.
     */
	if ((mtype == DDR_COMBO) || (mtype == DDR_STACKED)) {
//20101215_Peter ++
#if defined (CONFIG_EPXX_DDR_512MB)
		do_sdrc_init(SDRC_CS1_OSET, EARLY_INIT);
		make_cs1_contiguous();
#else
		do_sdrc_init(SDRC_CS1_OSET, NOT_EARLY);
#endif
//20101215_Peter --
	}

#ifdef DEBUG
 {
	unsigned int reg = 0;

	reg = __raw_readl(SDRC_MCFG_0);
	printf("SDRC_MCFG_0: %08x\n", reg);

	reg = __raw_readl(SDRC_MCFG_1);
	printf("SDRC_MCFG_1: %08x\n", reg);
	
	reg = __raw_readl(SDRC_ACTIM_CTRLA_0);
	__raw_writel(reg, SDRC_ACTIM_CTRLA_1);
	printf("SDRC_ACTIM_CTRLA_0: %08x\n", reg);
	
	reg = __raw_readl(SDRC_ACTIM_CTRLB_0);
	__raw_writel(reg, SDRC_ACTIM_CTRLB_1);
	printf("SDRC_ACTIM_CTRLB_0: %08x\n", reg);
	
	reg = __raw_readl(SDRC_ACTIM_CTRLA_1);
	printf("SDRC_ACTIM_CTRLA_1: %08x\n", reg);
	
	reg = __raw_readl(SDRC_ACTIM_CTRLB_1);
	printf("SDRC_ACTIM_CTRLB_1: %08x\n", reg);
	
	reg = __raw_readl(SDRC_MANUAL_0);
	printf("SDRC_MANUAL_0: %08x\n", reg);
	
	reg = __raw_readl(SDRC_MANUAL_1);
	printf("SDRC_MANUAL_1: %08x\n", reg);
	
	reg = __raw_readl(SDRC_MR_0);
	printf("SDRC_MR_0: %08x\n", reg);

	reg = __raw_readl(SDRC_MR_1);
	printf("SDRC_MR_1: %08x\n", reg);
	
	reg = __raw_readl(SDRC_RFR_CTRL_0);
	__raw_writel(reg, SDRC_RFR_CTRL_1);
	printf("SDRC_RFR_CTRL_0: %08x\n", reg);
	
	reg = __raw_readl(SDRC_RFR_CTRL_1);
	printf("SDRC_RFR_CTRL_1: %08x\n", reg);
	
	reg = __raw_readl(CONTROL_PROG_IO0);
	printf("CONTROL_PROG_IO0: %08x\n", reg);
	
	reg = __raw_readl(CONTROL_PROG_IO1);
	printf("CONTROL_PROG_IO1: %08x\n", reg);
	
	reg = __raw_readl(SDRC_DLLA_CTRL);
	printf("SDRC_DLLA_CTRL: %08x\n", reg);
	
 }
#endif

	size0 = get_sdr_cs_size(SDRC_CS0_OSET);
	size1 = get_sdr_cs_size(SDRC_CS1_OSET);

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = size0;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_1+size0;
	gd->bd->bi_dram[1].size = size1;

	return 0;
}

#define 	MUX_VAL(OFFSET,VALUE)\
		__raw_writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET));

#define		CP(x)	(CONTROL_PADCONF_##x)
/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */
#ifdef CONFIG_CL1_3G_VERSION /* <--LH_SWRD_CL1_Mervins@2011.09.01:for 3G version --> */
#define MUX_DEFAULT_ES2()\
	/*SDRC*/\
	MUX_VAL(CP(SDRC_D0),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D0*/\
	MUX_VAL(CP(SDRC_D1),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D1*/\
	MUX_VAL(CP(SDRC_D2),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D2*/\
	MUX_VAL(CP(SDRC_D3),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D3*/\
	MUX_VAL(CP(SDRC_D4),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D4*/\
	MUX_VAL(CP(SDRC_D5),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D5*/\
	MUX_VAL(CP(SDRC_D6),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D6*/\
	MUX_VAL(CP(SDRC_D7),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D7*/\
	MUX_VAL(CP(SDRC_D8),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D8*/\
	MUX_VAL(CP(SDRC_D9),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D9*/\
	MUX_VAL(CP(SDRC_D10),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D10*/\
	MUX_VAL(CP(SDRC_D11),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D11*/\
	MUX_VAL(CP(SDRC_D12),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D12*/\
	MUX_VAL(CP(SDRC_D13),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D13*/\
	MUX_VAL(CP(SDRC_D14),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D14*/\
	MUX_VAL(CP(SDRC_D15),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D15*/\
	MUX_VAL(CP(SDRC_D16),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D16*/\
	MUX_VAL(CP(SDRC_D17),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D17*/\
	MUX_VAL(CP(SDRC_D18),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D18*/\
	MUX_VAL(CP(SDRC_D19),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D19*/\
	MUX_VAL(CP(SDRC_D20),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D20*/\
	MUX_VAL(CP(SDRC_D21),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D21*/\
	MUX_VAL(CP(SDRC_D22),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D22*/\
	MUX_VAL(CP(SDRC_D23),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D23*/\
	MUX_VAL(CP(SDRC_D24),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D24*/\
	MUX_VAL(CP(SDRC_D25),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D25*/\
	MUX_VAL(CP(SDRC_D26),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D26*/\
	MUX_VAL(CP(SDRC_D27),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D27*/\
	MUX_VAL(CP(SDRC_D28),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D28*/\
	MUX_VAL(CP(SDRC_D29),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D29*/\
	MUX_VAL(CP(SDRC_D30),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D30*/\
	MUX_VAL(CP(SDRC_D31),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D31*/\
	MUX_VAL(CP(SDRC_CLK),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_CLK*/\
	MUX_VAL(CP(SDRC_DQS0),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_DQS0*/\
	MUX_VAL(CP(SDRC_DQS1),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_DQS1*/\
	MUX_VAL(CP(SDRC_DQS2),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_DQS2*/\
	MUX_VAL(CP(SDRC_DQS3),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*SDRC_DQS3*/\
	/* MUX_VAL(CP(SYS_NRESWARM),   (IDIS | PTD | DIS | M0)) */ /*SYS_NRESWARM*/\
	/*GPMC*/\
	MUX_VAL(CP(GPMC_A1),        (IDIS | PTD | EN | OFF_OUT_PD| M4)) /*GPMC_A1, GPIO_34, CCM_PWDN2*/\
	MUX_VAL(CP(GPMC_A2),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-35 */\
	MUX_VAL(CP(GPMC_A3),        (IDIS | PTD | DIS | OFF_OUT_PD | M4)) /*GPIO-36 */\
	MUX_VAL(CP(GPMC_A4),        (IDIS | PTD | DIS | OFF_OUT_PD | M4)) /*GPIO-37, LVDS SHTDN_N*/\
	MUX_VAL(CP(GPMC_A5),        (IDIS | PTD | EN  | OFF_OUT_PD | M4)) /*GPIO-38, LCD_BL_EN */\
	MUX_VAL(CP(GPMC_A6),        (IEN  | PTD | DIS  | OFF_IN_PD | M4)) /*BT_HOST_WAKEUP, GPIO-39 */\
	MUX_VAL(CP(GPMC_A7),        (IDIS | PTU | DIS | OFF_IN_PU | M4)) /*GPIO-40, BT_WAKEUP */\
	MUX_VAL(CP(GPMC_A8),        (IDIS | PTU | DIS | M4)) /*GPIO-41 WL_WAKEUP*/\
	MUX_VAL(CP(GPMC_A9),        (IDIS  | PTU | DIS | M4)) /*GPMC_A9, GPIO-42, MAX8677_CEN */\
	MUX_VAL(CP(GPMC_A10),       (IDIS  | PTD | DIS  | OFF_IN_PD | M4)) /*GPIO-43, 3G SYSTEM_RESET_N */\
	MUX_VAL(CP(GPMC_D0),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D0, NAND */\
	MUX_VAL(CP(GPMC_D1),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D1, NAND */\
	MUX_VAL(CP(GPMC_D2),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D2, NAND */\
	MUX_VAL(CP(GPMC_D3),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D3, NAND */\
	MUX_VAL(CP(GPMC_D4),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D4, NAND */\
	MUX_VAL(CP(GPMC_D5),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D5, NAND */\
	MUX_VAL(CP(GPMC_D6),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D6, NAND */\
	MUX_VAL(CP(GPMC_D7),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D7, NAND*/\
	MUX_VAL(CP(GPMC_D8),        (IEN | PTU | DIS | OFF_IN_PD | M4)) /*GPIO-44, CAMERA_OE */\
	MUX_VAL(CP(GPMC_D9),        (IDIS | PTD | EN | OFF_IN_PD | M4)) /*GPIO-45, CCM_RESET_N*/\
	MUX_VAL(CP(GPMC_D10),       (IDIS | PTU| EN | M4)) /*GPIO-46, UART1_SW/GPIO110*/\
/*	MUX_VAL(CP(GPMC_D10),       (IDIS | PTD | EN | OFF_IN_PD | M4)) *//*GPIO-46, UART1_SW/GPIO110*/\
	MUX_VAL(CP(GPMC_D11),       (IDIS | PTU | EN | OFF_IN_PU | M4)) /*GPIO-47, CAMERA_SEL*/\
	MUX_VAL(CP(GPMC_D12),       (IDIS | PTU | EN | OFF_IN_PU | M4)) /*GPIO-48, LCD_RST1*/\
	MUX_VAL(CP(GPMC_D13),       (IDIS | PTU | EN | OFF_IN_PU | M4)) /*GPIO-49, LCD_STB1 */\
	MUX_VAL(CP(GPMC_D14),       (IDIS | PTD | EN | OFF_IN_PD | M4)) /*GPIO-50, 3G_POWER_ON*/\
	MUX_VAL(CP(GPMC_D15),       (IEN | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-51, DETECT_DEVICE */\
	/*MUX_VAL(CP(GPMC_nCS0),    (IDIS | PTD | DIS | M0)) *//*GPMC_nCS0*/\
	/*MUX_VAL(CP(GPMC_nCS1),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-52*/\
	/*MUX_VAL(CP(GPMC_nCS2),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-53*/\
	/*MUX_VAL(CP(GPMC_nCS3),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-54*/\
	/*MUX_VAL(CP(GPMC_nCS4),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-55*/\
	/*MUX_VAL(CP(GPMC_nCS5),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-56*/\
	/*MUX_VAL(CP(GPMC_nCS6),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-57*/\
	MUX_VAL(CP(GPMC_nCS7),      (IDIS | PTD | EN | OFF_IN_PD | M3/*M0*/)) /*GPMC_nCS7, LCD backlight PWM*/\
	MUX_VAL(CP(GPMC_CLK),       (IDIS | PTD | DIS  | M4)) /*GPI0-59, LED_EN */\
	/*MUX_VAL(CP(GPMC_nADV_ALE),(IDIS | PTD | DIS | M0)) *//*GPMC_nADV_ALE*/\
	/*MUX_VAL(CP(GPMC_nOE),     (IDIS | PTD | DIS | M0)) *//*GPMC_nOE*/\
	/*MUX_VAL(CP(GPMC_nWE),     (IDIS | PTD | DIS | M0)) *//*GPMC_nWE*/\
	MUX_VAL(CP(GPMC_nBE0_CLE),  (IEN | PTU | DIS | OFF_IN_PD | M4)) /*GPI0-60, up_IOC_LEVEL */\
	MUX_VAL(CP(GPMC_nBE1),      (IDIS| PTD| EN | M4)) /*GPI0-61, LCD_POWER*/\	
	MUX_VAL(CP(GPMC_nWP),       (IDIS| PTD | EN | OFF_IN_PD | M4)) /*GPI0-62, Touch_LED*/\
	MUX_VAL(CP(GPMC_WAIT0),     (IEN | PTD | EN | OFF_IN_PD | M0)) /*GPMC_WAIT0*/\
	MUX_VAL(CP(GPMC_WAIT1),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPI0-63 Not balled*/\
	MUX_VAL(CP(GPMC_WAIT2),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPI0-64 Not balled*/\
	MUX_VAL(CP(GPMC_WAIT3),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPI0-65 Not balled*/\
	/*DSS*/\
	MUX_VAL(CP(DSS_PCLK),       (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_PCLK*/\
	MUX_VAL(CP(DSS_HSYNC),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_HSYNC*/\
	MUX_VAL(CP(DSS_VSYNC),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_VSYNC*/\
	MUX_VAL(CP(DSS_ACBIAS),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_ACBIAS*/\
	MUX_VAL(CP(DSS_DATA0),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA0 */\
	MUX_VAL(CP(DSS_DATA1),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA1 */\
	MUX_VAL(CP(DSS_DATA2),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA2 */\
	MUX_VAL(CP(DSS_DATA3),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA3 */\
	MUX_VAL(CP(DSS_DATA4),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA4 */\
	MUX_VAL(CP(DSS_DATA5),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA5 */\
	MUX_VAL(CP(DSS_DATA6),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA6 */\
	MUX_VAL(CP(DSS_DATA7),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA7 */\
	MUX_VAL(CP(DSS_DATA8),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA8*/\
	MUX_VAL(CP(DSS_DATA9),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA9*/\
	MUX_VAL(CP(DSS_DATA10),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT10*/\
	MUX_VAL(CP(DSS_DATA11),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT11*/\
	MUX_VAL(CP(DSS_DATA12),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT12*/\
	MUX_VAL(CP(DSS_DATA13),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT13*/\
	MUX_VAL(CP(DSS_DATA14),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT14*/\
	MUX_VAL(CP(DSS_DATA15),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA15, GPIO-85*/\
	MUX_VAL(CP(DSS_DATA16),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT16*/\
	MUX_VAL(CP(DSS_DATA17),     (IDIS | PTU | DIS | OFF_IN_PD | M0)) /*DSS_DATA17, GPIO-87*/\
	MUX_VAL(CP(DSS_DATA18),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT18*/\
	MUX_VAL(CP(DSS_DATA19),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT19*/\
	MUX_VAL(CP(DSS_DATA20),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA20, VOLT_INT */\
	MUX_VAL(CP(DSS_DATA21),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA21*/\
	MUX_VAL(CP(DSS_DATA22),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA22*/\
	MUX_VAL(CP(DSS_DATA23),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA23*/\
	/*McSPI1 */\
	MUX_VAL(CP(McSPI1_SIMO),    (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_171 */\
	MUX_VAL(CP(McSPI1_CLK),     (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_172*/\
	MUX_VAL(CP(McSPI1_CS0),     (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_173*/\
	MUX_VAL(CP(McSPI1_SOMI),    (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_174*/\
	MUX_VAL(CP(McSPI1_CS1),     (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_175*/\
	MUX_VAL(CP(McSPI1_CS2),     (IEN | PTD  | EN | M4)) /*GPIO_176*/\
	/*CAMERA*/\
	MUX_VAL(CP(CAM_HS),         (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-94 Not balled*/\
	MUX_VAL(CP(CAM_VS),         (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-95 Not balled*/\
	MUX_VAL(CP(CAM_XCLKA),      (IEN  | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-96, CCM_PWDN1*/\
	MUX_VAL(CP(CAM_PCLK),       (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-97 Not balled*/\
	MUX_VAL(CP(CAM_FLD),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-98 Not balled*/\
	MUX_VAL(CP(CAM_D0 ),        (IEN  | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-99 HPH_DETECT*/\
	MUX_VAL(CP(CAM_D1 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-100 G_Sen/GPIO_100/CAM_D1*/\
	MUX_VAL(CP(CAM_D2 ),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-101, GPS_RESET*/\
	MUX_VAL(CP(CAM_D3 ),        (IDIS | PTD | DIS | M4)) /*BT_RST_N, GPIO-102*/\
	MUX_VAL(CP(CAM_D4 ),        (IDIS | PTD | DIS | M4)) /*GPIO-103, AUDIO_REST*/\
	MUX_VAL(CP(CAM_D5 ),        (IDIS | PTD | DIS | M4)) /*GPIO-104, (CHARGE) DCM */\
	MUX_VAL(CP(CAM_D6 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-105*/\
	MUX_VAL(CP(CAM_D7 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-106*/\
	MUX_VAL(CP(CAM_D8 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-107*/\
	MUX_VAL(CP(CAM_D9 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-108*/\
	MUX_VAL(CP(CAM_D10),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-109, GPS_ON_OFF*/\
	MUX_VAL(CP(CAM_D11),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-110, LCD_EN*/\
	MUX_VAL(CP(CAM_XCLKB),      (IEN  | PTD | DIS | M0)) /*GPIO-111 -- sam */\
	MUX_VAL(CP(CAM_WEN),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-167*/\
	MUX_VAL(CP(CAM_STROBE),     (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-126*/\
	MUX_VAL(CP(CSI2_DX0),       (IEN  | PTD | DIS | M0)) /*GPIO-112, CCM_CLK_N -- sam */\
	MUX_VAL(CP(CSI2_DY0),       (IEN  | PTD | DIS | M0)) /*GPIO-113, CCM_CLK_P -- sam */\
	MUX_VAL(CP(CSI2_DX1),       (IEN  | PTD | DIS | M0)) /*GPIO-114, CCM_DATA_N -- sam */\
	MUX_VAL(CP(CSI2_DY1),       (IEN  | PTD | DIS | M0)) /*GPIO-115, CCM_DATA_P -- sam */\
	/*Audio Interface */\
	MUX_VAL(CP(McBSP2_FSX),     (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_FSX*/\
	MUX_VAL(CP(McBSP2_CLKX),    (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_CLKX*/\
	MUX_VAL(CP(McBSP2_DR),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_DR*/\
	MUX_VAL(CP(McBSP2_DX),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_DX*/\
	/*Expansion card  */\
	MUX_VAL(CP(MMC1_CLK),       (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_CLK */\
	MUX_VAL(CP(MMC1_CMD),       (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_CMD */\
	MUX_VAL(CP(MMC1_DAT0),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT0*/\
	MUX_VAL(CP(MMC1_DAT1),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT1*/\
	MUX_VAL(CP(MMC1_DAT2),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT2*/\
	MUX_VAL(CP(MMC1_DAT3),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT3*/\
	MUX_VAL(CP(MMC1_DAT4),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT4*/\
	MUX_VAL(CP(MMC1_DAT5),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT5*/\
	MUX_VAL(CP(MMC1_DAT6),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT6*/\
	MUX_VAL(CP(MMC1_DAT7),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT7*/\
	/* eMMC */\
	MUX_VAL(CP(MMC2_CLK),       (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_CLK */\
	MUX_VAL(CP(MMC2_CMD),       (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_CMD */\
	MUX_VAL(CP(MMC2_DAT0),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT0*/\
	MUX_VAL(CP(MMC2_DAT1),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT1*/\
	MUX_VAL(CP(MMC2_DAT2),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT2*/\
	MUX_VAL(CP(MMC2_DAT3),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT3*/\
	MUX_VAL(CP(MMC2_DAT4),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_DIR_DAT0*/\
	MUX_VAL(CP(MMC2_DAT5),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_DIR_DAT1*/\
	MUX_VAL(CP(MMC2_DAT6),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_DIR_CMD */\
	MUX_VAL(CP(MMC2_DAT7),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_CLKIN*/\
	/*Bluetooth*/\
	MUX_VAL(CP(McBSP3_DX),      (IDIS | PTD | DIS | OFF_IN_PD | M1)) /*McBSP3_DX, BT_UART2_CTS  */\
	MUX_VAL(CP(McBSP3_DR),      (IEN  | PTD | DIS | OFF_IN_PD | M1)) /*McBSP3_DR, BT_UART2_RTS */\
	MUX_VAL(CP(McBSP3_CLKX),    (IDIS | PTD | DIS | OFF_OUT_PD | M1)) /*McBSP3_CLKX, BT_UART2_TX*/\
	MUX_VAL(CP(McBSP3_FSX),     (IEN  | PTD | DIS | OFF_IN_PD | M1)) /*McBSP3_FSX, BT_UART2_RX */\
	/*UART2 & 3  Interface */\
	MUX_VAL(CP(UART2_CTS),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-144 Not balled*/\
	MUX_VAL(CP(UART2_RTS),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-145 Not balled*/\
	MUX_VAL(CP(UART2_TX),       (IEN | PTD | EN | OFF_IN_PD | M4)) /*UGPIO-146 Not balled*/\
	MUX_VAL(CP(UART2_RX),       (IEN | PTD | EN | OFF_IN_PU | M4)) /*GPIO-147 Not balled*/\
	MUX_VAL(CP(McBSP1_CLKX),    (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-162 Not balled*/\
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-163 Not balled*/\
	MUX_VAL(CP(UART3_RX_IRRX),  (IEN | PTD | EN | OFF_IN_PU | M4)) /*UGPIO-164 Not balled*/\
	MUX_VAL(CP(UART3_RTS_SD),   (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-165 Not balled*/\
	MUX_VAL(CP(UART3_TX_IRTX),  (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-165 Not balled*/\
	/*Modem Interface */\
	MUX_VAL(CP(UART1_TX),       (IDIS | PTD | DIS | OFF_OUT_PD | M0)) /*UART1_TX*/\
	MUX_VAL(CP(UART1_RTS),      (IEN| PTU | DIS | OFF_IN_PU | M0)) /*UART1_RTS, GPIO-149, UART_RTS*/\
	MUX_VAL(CP(UART1_CTS),      (IEN  | PTU | DIS | OFF_IN_PU | M0)) /*UART1_CTS, GPIO_150, UART_CTS*/\
	MUX_VAL(CP(UART1_RX),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*UART1_RX*/\
	MUX_VAL(CP(McBSP1_CLKR),    (IDIS | PTU | EN | OFF_IN_PD | M4)) /*GPIO-156,McBSP1_CLKR*/\
	MUX_VAL(CP(McBSP1_FSR),     (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO_157*/\
	MUX_VAL(CP(McBSP1_DX),      (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GIO-158, WL_RST_N*/\
	MUX_VAL(CP(McBSP1_DR),      (IDIS | PTD | EN | OFF_IN_PD | M4)) /*GIO-159, VGH_CTRL*/\
	MUX_VAL(CP(McBSP_CLKS),     (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GIO-160*/\
	MUX_VAL(CP(McBSP1_FSX),     (IEN  | PTD | DIS | OFF_IN_PU | M4)) /*GIO-161, G_sensor_Lock */\
	/*McBSP4 Interface */\
	MUX_VAL(CP(McBSP4_CLKX),    (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-152*/\
	MUX_VAL(CP(McBSP4_DR),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-153*/\
	MUX_VAL(CP(McBSP4_FSX),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-154*/\
	MUX_VAL(CP(McBSP4_DX),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-155*/\
	/*Serial Interface*/\
	MUX_VAL(CP(HSUSB0_CLK),     (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*HSUSB0_CLK*/\
	MUX_VAL(CP(HSUSB0_STP),     (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*HSUSB0_STP*/\
	MUX_VAL(CP(HSUSB0_DIR),     (IEN  | PTD | EN  | OFF_IN_PU | M0)) /*HSUSB0_DIR*/\
	MUX_VAL(CP(HSUSB0_NXT),     (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*HSUSB0_NXT*/\
	MUX_VAL(CP(HSUSB0_DATA0),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA0 */\
	MUX_VAL(CP(HSUSB0_DATA1),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA1 */\
	MUX_VAL(CP(HSUSB0_DATA2),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA2 */\
	MUX_VAL(CP(HSUSB0_DATA3),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA3 */\
	MUX_VAL(CP(HSUSB0_DATA4),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA4 */\
	MUX_VAL(CP(HSUSB0_DATA5),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA5 */\
	MUX_VAL(CP(HSUSB0_DATA6),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA6 */\
	MUX_VAL(CP(HSUSB0_DATA7),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA7 */\
	MUX_VAL(CP(I2C1_SCL),       (IEN  | PTU | EN  | M0)) /*I2C1_SCL*/\
	MUX_VAL(CP(I2C1_SDA),       (IEN  | PTU | EN  | M0)) /*I2C1_SDA*/\
	MUX_VAL(CP(I2C2_SCL),       (IEN  | PTU | EN  | M0)) /*I2C2_SCL*/\
	MUX_VAL(CP(I2C2_SDA),       (IEN  | PTU | EN  | M0)) /*I2C2_SDA*/\
	MUX_VAL(CP(I2C4_SCL),       (IEN  | PTU | EN  | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),       (IEN  | PTU | EN  | M0)) /*I2C4_SDA*/\
	MUX_VAL(CP(McSPI1_CS3),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D2*/\
	MUX_VAL(CP(McSPI2_CLK),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D7*/\
	MUX_VAL(CP(McSPI2_SIMO),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D4*/\
	MUX_VAL(CP(McSPI2_SOMI),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D5*/\
	MUX_VAL(CP(McSPI2_CS0),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D6*/\
	MUX_VAL(CP(McSPI2_CS1),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D3*/\
	/*Control and debug */\
	MUX_VAL(CP(SYS_32K),        (IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_CLKREQ),     (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SYS_CLKREQ*/\
	MUX_VAL(CP(SYS_nIRQ),       (WAKEUP_EN  | OFF_IN_PU  | IEN  | PTU | EN  | M0)) /*SYS_nIRQ*/\
	MUX_VAL(CP(SYS_BOOT0),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT0*/\
	MUX_VAL(CP(SYS_BOOT1),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT1*/\
	MUX_VAL(CP(SYS_BOOT2),      (IEN  | PTU | DIS | M4)) /*SYS_BOOT2*/\
	MUX_VAL(CP(SYS_BOOT3),      (IEN  | PTD | DIS | M4)) /*SYS_BOOT3*/\
	MUX_VAL(CP(SYS_BOOT4),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT4*/\
	MUX_VAL(CP(SYS_BOOT5),      (IEN  | PTD | DIS | M4)) /*SYS_BOOT5*/\
	MUX_VAL(CP(SYS_BOOT6),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT6*/\
	MUX_VAL(CP(SYS_OFF_MODE),   (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*SYS_OFF_MODE */\
	MUX_VAL(CP(SYS_CLKOUT2),    (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*SYS_CLKOUT2   */\
	MUX_VAL(CP(JTAG_nTRST),     (IEN  | PTD | DIS | M0)) /*JTAG_nTRST*/\
	MUX_VAL(CP(JTAG_TCK),       (IEN  | PTD | DIS | M0)) /*JTAG_TCK*/\
	MUX_VAL(CP(JTAG_TMS),       (IEN  | PTD | DIS | M0)) /*JTAG_TMS*/\
	MUX_VAL(CP(JTAG_TDI),       (IEN  | PTD | DIS | M0)) /*JTAG_TDI*/\
	MUX_VAL(CP(JTAG_TDO),  	    (IEN  | PTD | DIS | M0)) /*JTAG_TDO*/\
	MUX_VAL(CP(JTAG_RTCK),      (IEN  | PTD | DIS | M0)) /*JTAG_RTCK*/\
	MUX_VAL(CP(JTAG_EMU0),      (IDIS | PTU | EN | OFF_OUT_PU | M4)) /*GPIO-11  JTAG_EMU0*/\
	MUX_VAL(CP(JTAG_EMU1),      (IDIS | PTU | EN | OFF_IN_PD | M4 )) /*GPIO-31 GPIO_103/CAM_D4*/\
	MUX_VAL(CP(ETK_CTL_ES2),    (IEN  | PTD | DIS | OFF_IN_PD | M2)) /*MMC3_CMD */\
	MUX_VAL(CP(ETK_CLK_ES2),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_CLK  */\
	MUX_VAL(CP(ETK_D0_ES2 ),    (IEN  | PTD | DIS | OFF_IN_PD | M4)) /*GIPO-14 UP_IOC_DATA_GPIO(EVT1)*/\
	MUX_VAL(CP(ETK_D1_ES2 ),    (IEN | PTU | EN | M4)) /*GIO-15, CHG   */\
	MUX_VAL(CP(ETK_D2_ES2 ),    (IEN | PTD | EN | OFF_IN_PU | M4)) /*GIPO-16 ,WL_IRQ for OOB mode*/\
	MUX_VAL(CP(ETK_D4_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT0 */\
	MUX_VAL(CP(ETK_D5_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT1 */\
	MUX_VAL(CP(ETK_D6_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT2 */\
	MUX_VAL(CP(ETK_D3_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT3 */\
	MUX_VAL(CP(ETK_D7_ES2 ),    (IDIS | PTD | EN | OFF_OUT_PD | M4)) /*GIO-21    */\
	MUX_VAL(CP(ETK_D8_ES2 ),    (IDIS | PTD | EN | OFF_OUT_PD | M4)) /*VBAT_POWON, GIO-22*/\
	MUX_VAL(CP(ETK_D9_ES2 ),    (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GIO-23    */\
	MUX_VAL(CP(ETK_D10_ES2),    (IDIS | PTD | DIS | OFF_IN_PD | M3)) /*HSUSB2_CLK*/\
	MUX_VAL(CP(ETK_D11_ES2),    (IEN  | PTU | EN  | OFF_IN_PD | M3)) /*HSUSB2_STP*/\
	MUX_VAL(CP(ETK_D12_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_DIR*/\
	MUX_VAL(CP(ETK_D13_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_NXT*/\
	MUX_VAL(CP(ETK_D14_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D0 */\
	MUX_VAL(CP(ETK_D15_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D1 */\
	/*Die to Die */\
	MUX_VAL(CP(sdrc_cke0),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*sdrc_cke0 */\
	MUX_VAL(CP(sdrc_cke1),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*sdrc_cke1 not used*/\
	MUX_VAL(CP(d2d_mcad0),      (IEN  | PTD | EN  | M0)) /*d2d_mcad0*/\
	MUX_VAL(CP(d2d_mcad1),      (IEN  | PTD | EN  | M0)) /*d2d_mcad1*/\
	MUX_VAL(CP(d2d_mcad2),      (IEN  | PTD | EN  | M0)) /*d2d_mcad2*/\
	MUX_VAL(CP(d2d_mcad3),      (IEN  | PTD | EN  | M0)) /*d2d_mcad3*/\
	MUX_VAL(CP(d2d_mcad4),      (IEN  | PTD | EN  | M0)) /*d2d_mcad4*/\
	MUX_VAL(CP(d2d_mcad5),      (IEN  | PTD | EN  | M0)) /*d2d_mcad5*/\
	MUX_VAL(CP(d2d_mcad6),      (IEN  | PTD | EN  | M0)) /*d2d_mcad6*/\
	MUX_VAL(CP(d2d_mcad7),      (IEN  | PTD | EN  | M0)) /*d2d_mcad7*/\
	MUX_VAL(CP(d2d_mcad8),      (IEN  | PTD | EN  | M0)) /*d2d_mcad8*/\
	MUX_VAL(CP(d2d_mcad8),      (IEN  | PTD | EN  | M0)) /*d2d_mcad9*/\
	MUX_VAL(CP(d2d_mcad10),     (IEN  | PTD | EN  | M0)) /*d2d_mcad10*/\
	MUX_VAL(CP(d2d_mcad11),     (IEN  | PTD | EN  | M0)) /*d2d_mcad11*/\
	MUX_VAL(CP(d2d_mcad12),     (IEN  | PTD | EN  | M0)) /*d2d_mcad12*/\
	MUX_VAL(CP(d2d_mcad13),     (IEN  | PTD | EN  | M0)) /*d2d_mcad13*/\
	MUX_VAL(CP(d2d_mcad14),     (IEN  | PTD | EN  | M0)) /*d2d_mcad14*/\
	MUX_VAL(CP(d2d_mcad15),     (IEN  | PTD | EN  | M0)) /*d2d_mcad15*/\
	MUX_VAL(CP(d2d_mcad16),     (IEN  | PTD | EN  | M0)) /*d2d_mcad16*/\
	MUX_VAL(CP(d2d_mcad17),     (IEN  | PTD | EN  | M0)) /*d2d_mcad17*/\
	MUX_VAL(CP(d2d_mcad18),     (IEN  | PTD | EN  | M0)) /*d2d_mcad18*/\
	MUX_VAL(CP(d2d_mcad19),     (IEN  | PTD | EN  | M0)) /*d2d_mcad19*/\
	MUX_VAL(CP(d2d_mcad20),     (IEN  | PTD | EN  | M0)) /*d2d_mcad20*/\
	MUX_VAL(CP(d2d_mcad21),     (IEN  | PTD | EN  | M0)) /*d2d_mcad21*/\
	MUX_VAL(CP(d2d_mcad22),     (IEN  | PTD | EN  | M0)) /*d2d_mcad22*/\
	MUX_VAL(CP(d2d_mcad23),     (IEN  | PTD | EN  | M0)) /*d2d_mcad23*/\
	MUX_VAL(CP(d2d_mcad24),     (IEN  | PTD | EN  | M0)) /*d2d_mcad24*/\
	MUX_VAL(CP(d2d_mcad25),     (IEN  | PTD | EN  | M0)) /*d2d_mcad25*/\
	MUX_VAL(CP(d2d_mcad26),     (IEN  | PTD | EN  | M0)) /*d2d_mcad26*/\
	MUX_VAL(CP(d2d_mcad27),     (IEN  | PTD | EN  | M0)) /*d2d_mcad27*/\
	MUX_VAL(CP(d2d_mcad28),     (IEN  | PTD | EN  | M0)) /*d2d_mcad28*/\
	MUX_VAL(CP(d2d_mcad29),     (IEN  | PTD | EN  | M0)) /*d2d_mcad29*/\
	MUX_VAL(CP(d2d_mcad30),     (IEN  | PTD | EN  | M0)) /*d2d_mcad30*/\
	MUX_VAL(CP(d2d_mcad31),     (IEN  | PTD | EN  | M0)) /*d2d_mcad31*/\
	MUX_VAL(CP(d2d_mcad32),     (IEN  | PTD | EN  | M0)) /*d2d_mcad32*/\
	MUX_VAL(CP(d2d_mcad33),     (IEN  | PTD | EN  | M0)) /*d2d_mcad33*/\
	MUX_VAL(CP(d2d_mcad34),     (IEN  | PTD | EN  | M0)) /*d2d_mcad34*/\
	MUX_VAL(CP(d2d_mcad35),     (IEN  | PTD | EN  | M0)) /*d2d_mcad35*/\
	MUX_VAL(CP(d2d_mcad36),     (IEN  | PTD | EN  | M0)) /*d2d_mcad36*/\
	MUX_VAL(CP(d2d_clk26mi),    (IEN  | PTD | EN  | M0)) /*d2d_clk26msi*/\
	MUX_VAL(CP(d2d_nrespwron),  (IEN  | PTD | DIS | M0)) /*d2d_nrespwron*/\
	MUX_VAL(CP(d2d_nreswarm),   (IEN  | PTU | EN  | M0)) /*d2d_nreswarm*/\
	MUX_VAL(CP(d2d_arm9nirq),   (IEN  | PTD | DIS | M0)) /*d2d_arm9nirq*/\
	MUX_VAL(CP(d2d_uma2p6fiq),  (IEN  | PTD | DIS | M0)) /*d2d_uma2p6fi*/\
	MUX_VAL(CP(d2d_spint),      (IEN  | PTD | EN  | M0)) /*d2d_spint*/\
	MUX_VAL(CP(d2d_frint),      (IEN  | PTD | EN  | M0)) /*d2d_clk26msi*/\
	MUX_VAL(CP(d2d_dmareq0),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq0*/\
	MUX_VAL(CP(d2d_dmareq1),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq1*/\
	MUX_VAL(CP(d2d_dmareq2),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq2*/\
	MUX_VAL(CP(d2d_dmareq3),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq3*/\
	MUX_VAL(CP(d2d_n3gtrst),    (IEN  | PTD | EN  | M0)) /*d2d_n3gtrst*/\
	MUX_VAL(CP(d2d_n3gtdi),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtdi*/\
	MUX_VAL(CP(d2d_n3gtdo),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtdo*/\
	MUX_VAL(CP(d2d_n3gtms),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtms*/\
	MUX_VAL(CP(d2d_n3gtck),     (IEN  | PTD | DIS | M0)) /*d2d_n3gtck*/\
	MUX_VAL(CP(d2d_n3grtck),    (IEN  | PTD | EN  | M0)) /*d2d_rtck*/\
	MUX_VAL(CP(d2d_mstdby),     (IEN  | PTU | EN  | M0)) /*d2d_mstdby*/\
	MUX_VAL(CP(d2d_idlereq),    (IEN  | PTD | EN  | M0)) /*d2d_idlereq*/\
	MUX_VAL(CP(d2d_idleack),    (IEN  | PTU | DIS | M0)) /*d2d_idleack*/\
	MUX_VAL(CP(d2d_mwrite),     (IEN  | PTD | EN  | M0)) /*d2d_mwrite*/\
	MUX_VAL(CP(d2d_swrite),     (IEN  | PTD | EN  | M0)) /*d2d_swrite*/\
	MUX_VAL(CP(d2d_mread),      (IEN  | PTD | EN  | M0)) /*d2d_mread*/\
	MUX_VAL(CP(d2d_sread),      (IEN  | PTD | EN  | M0)) /*d2d_sread*/\
	MUX_VAL(CP(d2d_mbusflag),   (IEN  | PTD | EN  | M0)) /*d2d_mbusflag*/\
	MUX_VAL(CP(d2d_sbusflag),   (IEN  | PTD | EN  | M0)) /*d2d_sbusflag*/\
	/*I2C3 */\
	MUX_VAL(CP(I2C3_SCL),       (IEN | PTD | EN | M4)) /*GPIO_184 Not balled */\
	MUX_VAL(CP(I2C3_SDA),       (IEN | PTD | EN | M4)) /*GPIO_185 Not balled*/

#else
#define MUX_DEFAULT_ES2()\
	/*SDRC*/\
	MUX_VAL(CP(SDRC_D0),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D0*/\
	MUX_VAL(CP(SDRC_D1),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D1*/\
	MUX_VAL(CP(SDRC_D2),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D2*/\
	MUX_VAL(CP(SDRC_D3),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D3*/\
	MUX_VAL(CP(SDRC_D4),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D4*/\
	MUX_VAL(CP(SDRC_D5),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D5*/\
	MUX_VAL(CP(SDRC_D6),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D6*/\
	MUX_VAL(CP(SDRC_D7),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D7*/\
	MUX_VAL(CP(SDRC_D8),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D8*/\
	MUX_VAL(CP(SDRC_D9),        (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D9*/\
	MUX_VAL(CP(SDRC_D10),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D10*/\
	MUX_VAL(CP(SDRC_D11),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D11*/\
	MUX_VAL(CP(SDRC_D12),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D12*/\
	MUX_VAL(CP(SDRC_D13),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D13*/\
	MUX_VAL(CP(SDRC_D14),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D14*/\
	MUX_VAL(CP(SDRC_D15),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D15*/\
	MUX_VAL(CP(SDRC_D16),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D16*/\
	MUX_VAL(CP(SDRC_D17),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D17*/\
	MUX_VAL(CP(SDRC_D18),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D18*/\
	MUX_VAL(CP(SDRC_D19),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D19*/\
	MUX_VAL(CP(SDRC_D20),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D20*/\
	MUX_VAL(CP(SDRC_D21),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D21*/\
	MUX_VAL(CP(SDRC_D22),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D22*/\
	MUX_VAL(CP(SDRC_D23),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D23*/\
	MUX_VAL(CP(SDRC_D24),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D24*/\
	MUX_VAL(CP(SDRC_D25),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D25*/\
	MUX_VAL(CP(SDRC_D26),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D26*/\
	MUX_VAL(CP(SDRC_D27),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D27*/\
	MUX_VAL(CP(SDRC_D28),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D28*/\
	MUX_VAL(CP(SDRC_D29),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D29*/\
	MUX_VAL(CP(SDRC_D30),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D30*/\
	MUX_VAL(CP(SDRC_D31),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_D31*/\
	MUX_VAL(CP(SDRC_CLK),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_CLK*/\
	MUX_VAL(CP(SDRC_DQS0),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_DQS0*/\
	MUX_VAL(CP(SDRC_DQS1),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_DQS1*/\
	MUX_VAL(CP(SDRC_DQS2),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SDRC_DQS2*/\
	MUX_VAL(CP(SDRC_DQS3),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*SDRC_DQS3*/\
	/* MUX_VAL(CP(SYS_NRESWARM),   (IDIS | PTD | DIS | M0)) */ /*SYS_NRESWARM*/\
	/*GPMC*/\
	MUX_VAL(CP(GPMC_A1),        (IDIS | PTD | EN | OFF_OUT_PD| M4)) /*GPMC_A1, GPIO_34, CCM_PWDN2*/\
	MUX_VAL(CP(GPMC_A2),        (IEN  | PTD | EN  | OFF_IN_PD | M7)) /*GPIO-35 */\
	MUX_VAL(CP(GPMC_A3),        (IDIS | PTD | DIS | OFF_OUT_PD | M4)) /*GPIO-36 */\
	MUX_VAL(CP(GPMC_A4),        (IDIS | PTD | DIS | OFF_OUT_PD | M4)) /*GPIO-37, LVDS SHTDN_N*/\
	MUX_VAL(CP(GPMC_A5),        (IDIS | PTD | EN  | OFF_OUT_PD | M4)) /*GPIO-38, LCD_BL_EN */\
	MUX_VAL(CP(GPMC_A6),        (IEN  | PTD | DIS  | OFF_IN_PD | M4)) /*BT_HOST_WAKEUP, GPIO-39 */\
	MUX_VAL(CP(GPMC_A7),        (IDIS | PTU | DIS | OFF_IN_PU | M4)) /*GPIO-40, BT_WAKEUP */\
	MUX_VAL(CP(GPMC_A8),        (IDIS | PTU | DIS | M4)) /*GPIO-41 WL_WAKEUP*/\
	MUX_VAL(CP(GPMC_A9),        (IDIS  | PTU | DIS | M4)) /*GPMC_A9, GPIO-42, MAX8677_CEN */\
	MUX_VAL(CP(GPMC_A10),       (IDIS  | PTD | DIS  | OFF_IN_PD | M4)) /*GPIO-43, 3G SYSTEM_RESET_N */\
	MUX_VAL(CP(GPMC_D0),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D0, NAND */\
	MUX_VAL(CP(GPMC_D1),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D1, NAND */\
	MUX_VAL(CP(GPMC_D2),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D2, NAND */\
	MUX_VAL(CP(GPMC_D3),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D3, NAND */\
	MUX_VAL(CP(GPMC_D4),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D4, NAND */\
	MUX_VAL(CP(GPMC_D5),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D5, NAND */\
	MUX_VAL(CP(GPMC_D6),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D6, NAND */\
	MUX_VAL(CP(GPMC_D7),        (IEN  | PTD | DIS  | OFF_IN_PD | M0)) /*GPMC_D7, NAND*/\
	MUX_VAL(CP(GPMC_D8),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-44, CAMERA_OE */\
	MUX_VAL(CP(GPMC_D9),        (IDIS | PTD | EN | OFF_IN_PD | M4)) /*GPIO-45, CCM_RESET_N*/\
	MUX_VAL(CP(GPMC_D10),       (IDIS | PTU| EN | M4)) /*GPIO-46, UART1_SW/GPIO110*/\
/*	MUX_VAL(CP(GPMC_D10),       (IDIS | PTD | EN | OFF_IN_PD | M4)) *//*GPIO-46, UART1_SW/GPIO110*/\
	MUX_VAL(CP(GPMC_D11),       (IDIS | PTU | EN | OFF_IN_PU | M4)) /*GPIO-47, CAMERA_SEL*/\
	MUX_VAL(CP(GPMC_D12),       (IDIS | PTU | EN | OFF_IN_PU | M4)) /*GPIO-48, LCD_RST1*/\
	MUX_VAL(CP(GPMC_D13),       (IDIS | PTU | EN | OFF_IN_PU | M4)) /*GPIO-49, LCD_STB1 */\
	MUX_VAL(CP(GPMC_D14),       (IDIS | PTD | EN | OFF_IN_PD | M4)) /*GPIO-50, 3G_POWER_ON*/\
	MUX_VAL(CP(GPMC_D15),       (IEN | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-51, DETECT_DEVICE */\
	/*MUX_VAL(CP(GPMC_nCS0),    (IDIS | PTD | DIS | M0)) *//*GPMC_nCS0*/\
	/*MUX_VAL(CP(GPMC_nCS1),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-52*/\
	/*MUX_VAL(CP(GPMC_nCS2),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-53*/\
	/*MUX_VAL(CP(GPMC_nCS3),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-54*/\
	/*MUX_VAL(CP(GPMC_nCS4),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-55*/\
	/*MUX_VAL(CP(GPMC_nCS5),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-56*/\
	/*MUX_VAL(CP(GPMC_nCS6),      (IEN | PTD | EN | OFF_IN_PD | M4))*/ /*GPIO-57*/\
	MUX_VAL(CP(GPMC_nCS7),      (IDIS | PTD | EN | OFF_IN_PD | M3/*M0*/)) /*GPMC_nCS7, LCD backlight PWM*/\
	MUX_VAL(CP(GPMC_CLK),       (IDIS | PTD | DIS  | M4)) /*GPI0-59, LED_EN */\
	/*MUX_VAL(CP(GPMC_nADV_ALE),(IDIS | PTD | DIS | M0)) *//*GPMC_nADV_ALE*/\
	/*MUX_VAL(CP(GPMC_nOE),     (IDIS | PTD | DIS | M0)) *//*GPMC_nOE*/\
	/*MUX_VAL(CP(GPMC_nWE),     (IDIS | PTD | DIS | M0)) *//*GPMC_nWE*/\
	MUX_VAL(CP(GPMC_nBE0_CLE),  (IEN | PTU | DIS | OFF_IN_PD | M4)) /*GPI0-60, up_IOC_LEVEL */\
	MUX_VAL(CP(GPMC_nBE1),      (IDIS| PTD| EN | M4)) /*GPI0-61, LCD_POWER*/\	
	MUX_VAL(CP(GPMC_nWP),       (IDIS| PTD | EN | OFF_IN_PD | M4)) /*GPI0-62, Touch_LED*/\
	MUX_VAL(CP(GPMC_WAIT0),     (IEN | PTD | EN | OFF_IN_PD | M0)) /*GPMC_WAIT0*/\
	MUX_VAL(CP(GPMC_WAIT1),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPI0-63 Not balled*/\
	MUX_VAL(CP(GPMC_WAIT2),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPI0-64 Not balled*/\
	MUX_VAL(CP(GPMC_WAIT3),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPI0-65 Not balled*/\
	/*DSS*/\
	MUX_VAL(CP(DSS_PCLK),       (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_PCLK*/\
	MUX_VAL(CP(DSS_HSYNC),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_HSYNC*/\
	MUX_VAL(CP(DSS_VSYNC),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_VSYNC*/\
	MUX_VAL(CP(DSS_ACBIAS),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_ACBIAS*/\
	MUX_VAL(CP(DSS_DATA0),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA0 */\
	MUX_VAL(CP(DSS_DATA1),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA1 */\
	MUX_VAL(CP(DSS_DATA2),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA2 */\
	MUX_VAL(CP(DSS_DATA3),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA3 */\
	MUX_VAL(CP(DSS_DATA4),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA4 */\
	MUX_VAL(CP(DSS_DATA5),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA5 */\
	MUX_VAL(CP(DSS_DATA6),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA6 */\
	MUX_VAL(CP(DSS_DATA7),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA7 */\
	MUX_VAL(CP(DSS_DATA8),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA8*/\
	MUX_VAL(CP(DSS_DATA9),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA9*/\
	MUX_VAL(CP(DSS_DATA10),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT10*/\
	MUX_VAL(CP(DSS_DATA11),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT11*/\
	MUX_VAL(CP(DSS_DATA12),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT12*/\
	MUX_VAL(CP(DSS_DATA13),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT13*/\
	MUX_VAL(CP(DSS_DATA14),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT14*/\
	MUX_VAL(CP(DSS_DATA15),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA15, GPIO-85*/\
	MUX_VAL(CP(DSS_DATA16),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT16*/\
	MUX_VAL(CP(DSS_DATA17),     (IDIS | PTU | DIS | OFF_IN_PD | M0)) /*DSS_DATA17, GPIO-87*/\
	MUX_VAL(CP(DSS_DATA18),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT18*/\
	MUX_VAL(CP(DSS_DATA19),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DAT19*/\
	MUX_VAL(CP(DSS_DATA20),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA20, VOLT_INT */\
	MUX_VAL(CP(DSS_DATA21),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA21*/\
	MUX_VAL(CP(DSS_DATA22),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA22*/\
	MUX_VAL(CP(DSS_DATA23),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA23*/\
	/*McSPI1 */\
	MUX_VAL(CP(McSPI1_SIMO),    (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_171 */\
	MUX_VAL(CP(McSPI1_CLK),     (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_172*/\
	MUX_VAL(CP(McSPI1_CS0),     (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_173*/\
	MUX_VAL(CP(McSPI1_SOMI),    (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_174*/\
	MUX_VAL(CP(McSPI1_CS1),     (IEN | PTD  | EN | OFF_IN_PD | M4)) /*GPIO_175*/\
	MUX_VAL(CP(McSPI1_CS2),     (IEN | PTD  | EN | M4)) /*GPIO_176*/\
	/*CAMERA*/\
	MUX_VAL(CP(CAM_HS),         (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-94 Not balled*/\
	MUX_VAL(CP(CAM_VS),         (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-95 Not balled*/\
	MUX_VAL(CP(CAM_XCLKA),      (IEN  | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-96, CCM_PWDN1*/\
	MUX_VAL(CP(CAM_PCLK),       (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-97 Not balled*/\
	MUX_VAL(CP(CAM_FLD),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-98 Not balled*/\
	MUX_VAL(CP(CAM_D0 ),        (IEN  | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-99 HPH_DETECT*/\
	MUX_VAL(CP(CAM_D1 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-100 G_Sen/GPIO_100/CAM_D1*/\
	MUX_VAL(CP(CAM_D2 ),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-101, GPS_RESET*/\
	MUX_VAL(CP(CAM_D3 ),        (IDIS | PTD | DIS | M4)) /*BT_RST_N, GPIO-102*/\
	MUX_VAL(CP(CAM_D4 ),        (IDIS | PTD | DIS | M4)) /*GPIO-103, AUDIO_REST*/\
	MUX_VAL(CP(CAM_D5 ),        (IDIS | PTD | DIS | M4)) /*GPIO-104, (CHARGE) DCM */\
	MUX_VAL(CP(CAM_D6 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-105*/\
	MUX_VAL(CP(CAM_D7 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-106*/\
	MUX_VAL(CP(CAM_D8 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-107*/\
	MUX_VAL(CP(CAM_D9 ),        (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GPIO-108*/\
	MUX_VAL(CP(CAM_D10),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-109, GPS_ON_OFF*/\
	MUX_VAL(CP(CAM_D11),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-110, LCD_EN*/\
	MUX_VAL(CP(CAM_XCLKB),      (IEN  | PTD | DIS | M0)) /*GPIO-111 -- sam */\
	MUX_VAL(CP(CAM_WEN),        (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-167*/\
	MUX_VAL(CP(CAM_STROBE),     (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO-126*/\
	MUX_VAL(CP(CSI2_DX0),       (IEN  | PTD | DIS | M0)) /*GPIO-112, CCM_CLK_N -- sam */\
	MUX_VAL(CP(CSI2_DY0),       (IEN  | PTD | DIS | M0)) /*GPIO-113, CCM_CLK_P -- sam */\
	MUX_VAL(CP(CSI2_DX1),       (IEN  | PTD | DIS | M0)) /*GPIO-114, CCM_DATA_N -- sam */\
	MUX_VAL(CP(CSI2_DY1),       (IEN  | PTD | DIS | M0)) /*GPIO-115, CCM_DATA_P -- sam */\
	/*Audio Interface */\
	MUX_VAL(CP(McBSP2_FSX),     (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_FSX*/\
	MUX_VAL(CP(McBSP2_CLKX),    (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_CLKX*/\
	MUX_VAL(CP(McBSP2_DR),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_DR*/\
	MUX_VAL(CP(McBSP2_DX),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*McBSP2_DX*/\
	/*Expansion card  */\
	MUX_VAL(CP(MMC1_CLK),       (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_CLK */\
	MUX_VAL(CP(MMC1_CMD),       (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_CMD */\
	MUX_VAL(CP(MMC1_DAT0),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT0*/\
	MUX_VAL(CP(MMC1_DAT1),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT1*/\
	MUX_VAL(CP(MMC1_DAT2),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT2*/\
	MUX_VAL(CP(MMC1_DAT3),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT3*/\
	MUX_VAL(CP(MMC1_DAT4),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT4*/\
	MUX_VAL(CP(MMC1_DAT5),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT5*/\
	MUX_VAL(CP(MMC1_DAT6),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT6*/\
	MUX_VAL(CP(MMC1_DAT7),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC1_DAT7*/\
	/* eMMC */\
	MUX_VAL(CP(MMC2_CLK),       (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_CLK */\
	MUX_VAL(CP(MMC2_CMD),       (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_CMD */\
	MUX_VAL(CP(MMC2_DAT0),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT0*/\
	MUX_VAL(CP(MMC2_DAT1),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT1*/\
	MUX_VAL(CP(MMC2_DAT2),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT2*/\
	MUX_VAL(CP(MMC2_DAT3),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_DAT3*/\
	MUX_VAL(CP(MMC2_DAT4),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_DIR_DAT0*/\
	MUX_VAL(CP(MMC2_DAT5),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_DIR_DAT1*/\
	MUX_VAL(CP(MMC2_DAT6),      (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*MMC2_DIR_CMD */\
	MUX_VAL(CP(MMC2_DAT7),      (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*MMC2_CLKIN*/\
	/*Bluetooth*/\
	MUX_VAL(CP(McBSP3_DX),      (IDIS | PTD | DIS | OFF_IN_PD | M1)) /*McBSP3_DX, BT_UART2_CTS  */\
	MUX_VAL(CP(McBSP3_DR),      (IEN  | PTD | DIS | OFF_IN_PD | M1)) /*McBSP3_DR, BT_UART2_RTS */\
	MUX_VAL(CP(McBSP3_CLKX),    (IDIS | PTD | DIS | OFF_OUT_PD | M1)) /*McBSP3_CLKX, BT_UART2_TX*/\
	MUX_VAL(CP(McBSP3_FSX),     (IEN  | PTD | DIS | OFF_IN_PD | M1)) /*McBSP3_FSX, BT_UART2_RX */\
	/*UART2 & 3  Interface */\
	MUX_VAL(CP(UART2_CTS),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-144 Not balled*/\
	MUX_VAL(CP(UART2_RTS),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-145 Not balled*/\
	MUX_VAL(CP(UART2_TX),       (IEN | PTD | EN | OFF_IN_PD | M4)) /*UGPIO-146 Not balled*/\
	MUX_VAL(CP(UART2_RX),       (IEN | PTD | EN | OFF_IN_PU | M4)) /*GPIO-147 Not balled*/\
	MUX_VAL(CP(McBSP1_CLKX),    (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-162 Not balled*/\
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-163 Not balled*/\
	MUX_VAL(CP(UART3_RX_IRRX),  (IEN | PTD | EN | OFF_IN_PU | M4)) /*UGPIO-164 Not balled*/\
	MUX_VAL(CP(UART3_RTS_SD),   (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-165 Not balled*/\
	MUX_VAL(CP(UART3_TX_IRTX),  (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-165 Not balled*/\
	/*Modem Interface */\
	MUX_VAL(CP(UART1_TX),       (IDIS | PTD | DIS | OFF_OUT_PD | M0)) /*UART1_TX*/\
	MUX_VAL(CP(UART1_RTS),      (IEN| PTU | DIS | OFF_IN_PU | M0)) /*UART1_RTS, GPIO-149, UART_RTS*/\
	MUX_VAL(CP(UART1_CTS),      (IEN  | PTU | DIS | OFF_IN_PU | M0)) /*UART1_CTS, GPIO_150, UART_CTS*/\
	MUX_VAL(CP(UART1_RX),       (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*UART1_RX*/\
	MUX_VAL(CP(McBSP1_CLKR),    (IDIS | PTU | EN | OFF_IN_PD | M4)) /*GPIO-156,McBSP1_CLKR*/\
	MUX_VAL(CP(McBSP1_FSR),     (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GPIO_157*/\
	MUX_VAL(CP(McBSP1_DX),      (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GIO-158, WL_RST_N*/\
	MUX_VAL(CP(McBSP1_DR),      (IDIS | PTD | EN | OFF_IN_PD | M4)) /*GIO-159, VGH_CTRL*/\
	MUX_VAL(CP(McBSP_CLKS),     (IEN  | PTD | EN  | OFF_IN_PD | M4)) /*GIO-160*/\
	MUX_VAL(CP(McBSP1_FSX),     (IEN  | PTD | DIS | OFF_IN_PU | M4)) /*GIO-161, G_sensor_Lock */\
	/*McBSP4 Interface */\
	MUX_VAL(CP(McBSP4_CLKX),    (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-152*/\
	MUX_VAL(CP(McBSP4_DR),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-153*/\
	MUX_VAL(CP(McBSP4_FSX),     (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-154*/\
	MUX_VAL(CP(McBSP4_DX),      (IEN | PTD | EN | OFF_IN_PD | M4)) /*GPIO-155*/\
	/*Serial Interface*/\
	MUX_VAL(CP(HSUSB0_CLK),     (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*HSUSB0_CLK*/\
	MUX_VAL(CP(HSUSB0_STP),     (IEN  | PTU | EN  | OFF_IN_PD | M0)) /*HSUSB0_STP*/\
	MUX_VAL(CP(HSUSB0_DIR),     (IEN  | PTD | EN  | OFF_IN_PU | M0)) /*HSUSB0_DIR*/\
	MUX_VAL(CP(HSUSB0_NXT),     (IEN  | PTD | EN  | OFF_IN_PD | M0)) /*HSUSB0_NXT*/\
	MUX_VAL(CP(HSUSB0_DATA0),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA0 */\
	MUX_VAL(CP(HSUSB0_DATA1),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA1 */\
	MUX_VAL(CP(HSUSB0_DATA2),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA2 */\
	MUX_VAL(CP(HSUSB0_DATA3),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA3 */\
	MUX_VAL(CP(HSUSB0_DATA4),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA4 */\
	MUX_VAL(CP(HSUSB0_DATA5),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA5 */\
	MUX_VAL(CP(HSUSB0_DATA6),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA6 */\
	MUX_VAL(CP(HSUSB0_DATA7),   (IEN  | PTD | EN  | M0)) /*HSUSB0_DATA7 */\
	MUX_VAL(CP(I2C1_SCL),       (IEN  | PTU | EN  | M0)) /*I2C1_SCL*/\
	MUX_VAL(CP(I2C1_SDA),       (IEN  | PTU | EN  | M0)) /*I2C1_SDA*/\
	MUX_VAL(CP(I2C2_SCL),       (IEN  | PTU | EN  | M0)) /*I2C2_SCL*/\
	MUX_VAL(CP(I2C2_SDA),       (IEN  | PTU | EN  | M0)) /*I2C2_SDA*/\
	MUX_VAL(CP(I2C4_SCL),       (IEN  | PTU | EN  | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),       (IEN  | PTU | EN  | M0)) /*I2C4_SDA*/\
	MUX_VAL(CP(McSPI1_CS3),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D2*/\
	MUX_VAL(CP(McSPI2_CLK),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D7*/\
	MUX_VAL(CP(McSPI2_SIMO),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D4*/\
	MUX_VAL(CP(McSPI2_SOMI),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D5*/\
	MUX_VAL(CP(McSPI2_CS0),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D6*/\
	MUX_VAL(CP(McSPI2_CS1),     (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D3*/\
	/*Control and debug */\
	MUX_VAL(CP(SYS_32K),        (IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_CLKREQ),     (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*SYS_CLKREQ*/\
	MUX_VAL(CP(SYS_nIRQ),       (WAKEUP_EN  | OFF_IN_PU  | IEN  | PTU | EN  | M0)) /*SYS_nIRQ*/\
	MUX_VAL(CP(SYS_BOOT0),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT0*/\
	MUX_VAL(CP(SYS_BOOT1),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT1*/\
	MUX_VAL(CP(SYS_BOOT2),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT2*/\
	MUX_VAL(CP(SYS_BOOT3),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT3*/\
	MUX_VAL(CP(SYS_BOOT4),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT4*/\
	MUX_VAL(CP(SYS_BOOT5),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT5*/\
	MUX_VAL(CP(SYS_BOOT6),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT6*/\
	MUX_VAL(CP(SYS_OFF_MODE),   (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*SYS_OFF_MODE */\
	MUX_VAL(CP(SYS_CLKOUT2),    (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*SYS_CLKOUT2   */\
	MUX_VAL(CP(JTAG_nTRST),     (IEN  | PTD | DIS | M0)) /*JTAG_nTRST*/\
	MUX_VAL(CP(JTAG_TCK),       (IEN  | PTD | DIS | M0)) /*JTAG_TCK*/\
	MUX_VAL(CP(JTAG_TMS),       (IEN  | PTD | DIS | M0)) /*JTAG_TMS*/\
	MUX_VAL(CP(JTAG_TDI),       (IEN  | PTD | DIS | M0)) /*JTAG_TDI*/\
	MUX_VAL(CP(JTAG_TDO),  	    (IEN  | PTD | DIS | M0)) /*JTAG_TDO*/\
	MUX_VAL(CP(JTAG_RTCK),      (IEN  | PTD | DIS | M0)) /*JTAG_RTCK*/\
	MUX_VAL(CP(JTAG_EMU0),      (IDIS | PTU | EN | OFF_OUT_PU | M4)) /*GPIO-11  JTAG_EMU0*/\
	MUX_VAL(CP(JTAG_EMU1),      (IDIS | PTU | EN | OFF_IN_PD | M4 )) /*GPIO-31 GPIO_103/CAM_D4*/\
	MUX_VAL(CP(ETK_CTL_ES2),    (IEN  | PTD | DIS | OFF_IN_PD | M2)) /*MMC3_CMD */\
	MUX_VAL(CP(ETK_CLK_ES2),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_CLK  */\
	MUX_VAL(CP(ETK_D0_ES2 ),    (IEN  | PTD | DIS | OFF_IN_PD | M4)) /*GIPO-14 UP_IOC_DATA_GPIO(EVT1)*/\
	MUX_VAL(CP(ETK_D1_ES2 ),    (IEN | PTU | EN | M4)) /*GIO-15, CHG   */\
	MUX_VAL(CP(ETK_D2_ES2 ),    (IEN | PTD | EN | OFF_IN_PU | M4)) /*GIPO-16 ,WL_IRQ for OOB mode*/\
	MUX_VAL(CP(ETK_D4_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT0 */\
	MUX_VAL(CP(ETK_D5_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT1 */\
	MUX_VAL(CP(ETK_D6_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT2 */\
	MUX_VAL(CP(ETK_D3_ES2 ),    (IEN  | PTU | EN  | OFF_IN_PD | M2)) /*MMC3_DAT3 */\
	MUX_VAL(CP(ETK_D7_ES2 ),    (IDIS | PTD | EN | OFF_OUT_PD | M4)) /*GIO-21    */\
	MUX_VAL(CP(ETK_D8_ES2 ),    (IDIS | PTD | EN | OFF_OUT_PD | M4)) /*VBAT_POWON, GIO-22*/\
	MUX_VAL(CP(ETK_D9_ES2 ),    (IDIS | PTD | DIS | OFF_IN_PD | M4)) /*GIO-23    */\
	MUX_VAL(CP(ETK_D10_ES2),    (IDIS | PTD | DIS | OFF_IN_PD | M3)) /*HSUSB2_CLK*/\
	MUX_VAL(CP(ETK_D11_ES2),    (IEN  | PTU | EN  | OFF_IN_PD | M3)) /*HSUSB2_STP*/\
	MUX_VAL(CP(ETK_D12_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_DIR*/\
	MUX_VAL(CP(ETK_D13_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_NXT*/\
	MUX_VAL(CP(ETK_D14_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D0 */\
	MUX_VAL(CP(ETK_D15_ES2),    (IEN  | PTD | EN  | OFF_IN_PD | M3)) /*HSUSB2_D1 */\
	/*Die to Die */\
	MUX_VAL(CP(sdrc_cke0),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*sdrc_cke0 */\
	MUX_VAL(CP(sdrc_cke1),      (IEN  | PTD | DIS | OFF_IN_PD | M0)) /*sdrc_cke1 not used*/\
	MUX_VAL(CP(d2d_mcad0),      (IEN  | PTD | EN  | M0)) /*d2d_mcad0*/\
	MUX_VAL(CP(d2d_mcad1),      (IEN  | PTD | EN  | M0)) /*d2d_mcad1*/\
	MUX_VAL(CP(d2d_mcad2),      (IEN  | PTD | EN  | M0)) /*d2d_mcad2*/\
	MUX_VAL(CP(d2d_mcad3),      (IEN  | PTD | EN  | M0)) /*d2d_mcad3*/\
	MUX_VAL(CP(d2d_mcad4),      (IEN  | PTD | EN  | M0)) /*d2d_mcad4*/\
	MUX_VAL(CP(d2d_mcad5),      (IEN  | PTD | EN  | M0)) /*d2d_mcad5*/\
	MUX_VAL(CP(d2d_mcad6),      (IEN  | PTD | EN  | M0)) /*d2d_mcad6*/\
	MUX_VAL(CP(d2d_mcad7),      (IEN  | PTD | EN  | M0)) /*d2d_mcad7*/\
	MUX_VAL(CP(d2d_mcad8),      (IEN  | PTD | EN  | M0)) /*d2d_mcad8*/\
	MUX_VAL(CP(d2d_mcad8),      (IEN  | PTD | EN  | M0)) /*d2d_mcad9*/\
	MUX_VAL(CP(d2d_mcad10),     (IEN  | PTD | EN  | M0)) /*d2d_mcad10*/\
	MUX_VAL(CP(d2d_mcad11),     (IEN  | PTD | EN  | M0)) /*d2d_mcad11*/\
	MUX_VAL(CP(d2d_mcad12),     (IEN  | PTD | EN  | M0)) /*d2d_mcad12*/\
	MUX_VAL(CP(d2d_mcad13),     (IEN  | PTD | EN  | M0)) /*d2d_mcad13*/\
	MUX_VAL(CP(d2d_mcad14),     (IEN  | PTD | EN  | M0)) /*d2d_mcad14*/\
	MUX_VAL(CP(d2d_mcad15),     (IEN  | PTD | EN  | M0)) /*d2d_mcad15*/\
	MUX_VAL(CP(d2d_mcad16),     (IEN  | PTD | EN  | M0)) /*d2d_mcad16*/\
	MUX_VAL(CP(d2d_mcad17),     (IEN  | PTD | EN  | M0)) /*d2d_mcad17*/\
	MUX_VAL(CP(d2d_mcad18),     (IEN  | PTD | EN  | M0)) /*d2d_mcad18*/\
	MUX_VAL(CP(d2d_mcad19),     (IEN  | PTD | EN  | M0)) /*d2d_mcad19*/\
	MUX_VAL(CP(d2d_mcad20),     (IEN  | PTD | EN  | M0)) /*d2d_mcad20*/\
	MUX_VAL(CP(d2d_mcad21),     (IEN  | PTD | EN  | M0)) /*d2d_mcad21*/\
	MUX_VAL(CP(d2d_mcad22),     (IEN  | PTD | EN  | M0)) /*d2d_mcad22*/\
	MUX_VAL(CP(d2d_mcad23),     (IEN  | PTD | EN  | M0)) /*d2d_mcad23*/\
	MUX_VAL(CP(d2d_mcad24),     (IEN  | PTD | EN  | M0)) /*d2d_mcad24*/\
	MUX_VAL(CP(d2d_mcad25),     (IEN  | PTD | EN  | M0)) /*d2d_mcad25*/\
	MUX_VAL(CP(d2d_mcad26),     (IEN  | PTD | EN  | M0)) /*d2d_mcad26*/\
	MUX_VAL(CP(d2d_mcad27),     (IEN  | PTD | EN  | M0)) /*d2d_mcad27*/\
	MUX_VAL(CP(d2d_mcad28),     (IEN  | PTD | EN  | M0)) /*d2d_mcad28*/\
	MUX_VAL(CP(d2d_mcad29),     (IEN  | PTD | EN  | M0)) /*d2d_mcad29*/\
	MUX_VAL(CP(d2d_mcad30),     (IEN  | PTD | EN  | M0)) /*d2d_mcad30*/\
	MUX_VAL(CP(d2d_mcad31),     (IEN  | PTD | EN  | M0)) /*d2d_mcad31*/\
	MUX_VAL(CP(d2d_mcad32),     (IEN  | PTD | EN  | M0)) /*d2d_mcad32*/\
	MUX_VAL(CP(d2d_mcad33),     (IEN  | PTD | EN  | M0)) /*d2d_mcad33*/\
	MUX_VAL(CP(d2d_mcad34),     (IEN  | PTD | EN  | M0)) /*d2d_mcad34*/\
	MUX_VAL(CP(d2d_mcad35),     (IEN  | PTD | EN  | M0)) /*d2d_mcad35*/\
	MUX_VAL(CP(d2d_mcad36),     (IEN  | PTD | EN  | M0)) /*d2d_mcad36*/\
	MUX_VAL(CP(d2d_clk26mi),    (IEN  | PTD | EN  | M0)) /*d2d_clk26msi*/\
	MUX_VAL(CP(d2d_nrespwron),  (IEN  | PTD | DIS | M0)) /*d2d_nrespwron*/\
	MUX_VAL(CP(d2d_nreswarm),   (IEN  | PTU | EN  | M0)) /*d2d_nreswarm*/\
	MUX_VAL(CP(d2d_arm9nirq),   (IEN  | PTD | DIS | M0)) /*d2d_arm9nirq*/\
	MUX_VAL(CP(d2d_uma2p6fiq),  (IEN  | PTD | DIS | M0)) /*d2d_uma2p6fi*/\
	MUX_VAL(CP(d2d_spint),      (IEN  | PTD | EN  | M0)) /*d2d_spint*/\
	MUX_VAL(CP(d2d_frint),      (IEN  | PTD | EN  | M0)) /*d2d_clk26msi*/\
	MUX_VAL(CP(d2d_dmareq0),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq0*/\
	MUX_VAL(CP(d2d_dmareq1),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq1*/\
	MUX_VAL(CP(d2d_dmareq2),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq2*/\
	MUX_VAL(CP(d2d_dmareq3),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq3*/\
	MUX_VAL(CP(d2d_n3gtrst),    (IEN  | PTD | EN  | M0)) /*d2d_n3gtrst*/\
	MUX_VAL(CP(d2d_n3gtdi),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtdi*/\
	MUX_VAL(CP(d2d_n3gtdo),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtdo*/\
	MUX_VAL(CP(d2d_n3gtms),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtms*/\
	MUX_VAL(CP(d2d_n3gtck),     (IEN  | PTD | DIS | M0)) /*d2d_n3gtck*/\
	MUX_VAL(CP(d2d_n3grtck),    (IEN  | PTD | EN  | M0)) /*d2d_rtck*/\
	MUX_VAL(CP(d2d_mstdby),     (IEN  | PTU | EN  | M0)) /*d2d_mstdby*/\
	MUX_VAL(CP(d2d_idlereq),    (IEN  | PTD | EN  | M0)) /*d2d_idlereq*/\
	MUX_VAL(CP(d2d_idleack),    (IEN  | PTU | DIS | M0)) /*d2d_idleack*/\
	MUX_VAL(CP(d2d_mwrite),     (IEN  | PTD | EN  | M0)) /*d2d_mwrite*/\
	MUX_VAL(CP(d2d_swrite),     (IEN  | PTD | EN  | M0)) /*d2d_swrite*/\
	MUX_VAL(CP(d2d_mread),      (IEN  | PTD | EN  | M0)) /*d2d_mread*/\
	MUX_VAL(CP(d2d_sread),      (IEN  | PTD | EN  | M0)) /*d2d_sread*/\
	MUX_VAL(CP(d2d_mbusflag),   (IEN  | PTD | EN  | M0)) /*d2d_mbusflag*/\
	MUX_VAL(CP(d2d_sbusflag),   (IEN  | PTD | EN  | M0)) /*d2d_sbusflag*/\
	/*I2C3 */\
	MUX_VAL(CP(I2C3_SCL),       (IEN | PTD | EN | M4)) /*GPIO_184 Not balled */\
	MUX_VAL(CP(I2C3_SDA),       (IEN | PTD | EN | M4)) /*GPIO_185 Not balled*/

#endif
#define MUX_TFT_ES2()\
	/*DSS*/\
	MUX_VAL(CP(DSS_PCLK),       (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_PCLK  */\
	MUX_VAL(CP(DSS_HSYNC),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_HSYNC */\
	MUX_VAL(CP(DSS_VSYNC),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_VSYNC */\
	MUX_VAL(CP(DSS_ACBIAS),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_ACBIAS*/\
	MUX_VAL(CP(DSS_DATA0),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA0 */\
	MUX_VAL(CP(DSS_DATA1),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA1 */\
	MUX_VAL(CP(DSS_DATA2),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA2 */\
	MUX_VAL(CP(DSS_DATA3),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA3 */\
	MUX_VAL(CP(DSS_DATA4),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA4 */\
	MUX_VAL(CP(DSS_DATA5),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA5 */\
	MUX_VAL(CP(DSS_DATA6),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA6 */\
	MUX_VAL(CP(DSS_DATA7),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA7 */\
	MUX_VAL(CP(DSS_DATA8),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA8 */\
	MUX_VAL(CP(DSS_DATA9),      (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA9 */\
	MUX_VAL(CP(DSS_DATA10),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA10*/\
	MUX_VAL(CP(DSS_DATA11),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA11*/\
	MUX_VAL(CP(DSS_DATA12),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA12*/\
	MUX_VAL(CP(DSS_DATA13),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA13*/\
	MUX_VAL(CP(DSS_DATA14),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA14*/\
	MUX_VAL(CP(DSS_DATA15),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA15*/\
	MUX_VAL(CP(DSS_DATA16),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA16*/\
	MUX_VAL(CP(DSS_DATA17),     (IDIS | PTD | DIS | OFF_IN_PD | M0)) /*DSS_DATA17*/\
	MUX_VAL(CP(DSS_DATA18),     (IDIS | PTD | DIS | OFF_IN_PD | M0/*M0*/)) /*DSS_DATA18*/\
	MUX_VAL(CP(DSS_DATA19),     (IDIS | PTD | DIS | OFF_IN_PD | M0/*M0*/)) /*DSS_DATA19*/\
	MUX_VAL(CP(DSS_DATA20),     (IDIS | PTD | DIS | OFF_IN_PD | M0/*M0*/)) /*DSS_DATA20*/\
	MUX_VAL(CP(DSS_DATA21),     (IDIS | PTD | DIS | OFF_IN_PD | M0/*M0*/)) /*DSS_DATA21*/\
	MUX_VAL(CP(DSS_DATA22),     (IDIS | PTD | DIS | OFF_IN_PD | M0/*M0*/)) /*DSS_DATA22*/\
	MUX_VAL(CP(DSS_DATA23),     (IDIS | PTD | DIS | OFF_IN_PD | M0/*M0*/)) /*DSS_DATA23*/\
	MUX_VAL(CP(I2C2_SCL),       (IEN  | PTU | EN  | M0)) /*I2C2_SCL  */\
	MUX_VAL(CP(I2C2_SDA),       (IEN  | PTU | EN  | M0)) /*I2C2_SDA  */\
/*	MUX_VAL(CP(McSPI2_CLK),     (IEN  | PTD | EN  | OFF_IN_PD | M0))*/ /*hsusb2_d7 lab*/\
/*	MUX_VAL(CP(McSPI2_SIMO),    (IEN  | PTD | EN  | OFF_IN_PD | M0))*/ /*hsusb2_d4 lab*/\
/*	MUX_VAL(CP(McSPI2_SOMI),    (IEN  | PTD | EN  | OFF_IN_PD | M0))*/ /*hsusb2_d5 lab*/\
/*	MUX_VAL(CP(McSPI2_CS0),     (IEN  | PTD | EN  | OFF_IN_PD | M0))*/ /*hsusb2_d6 lab*/\
/*	MUX_VAL(CP(McSPI2_CS1),     (IEN  | PTD | EN  | OFF_IN_PD | M0))*/ /*hsusb2_d3 lab*/\
/*	MUX_VAL(CP(GPMC_nCS4),      (IDIS | PTD | DIS | OFF_IN_PD | M4))*/ /*GIO-55 */\
	MUX_VAL(CP(CAM_XCLKA),      (IDIS  | PTD | EN | OFF_IN_PD | M4)) /*GIO-96,CCM_PWDN1*/\
/*	MUX_VAL(CP(UART3_CTS_RCTX), (IDIS | PTD | DIS | OFF_IN_PD | M4))*/ /*GPIO-163 Not balled*/\
	/*MUX_VAL(CP(GPMC_A6),        (IEN  | PTU | EN  | OFF_IN_PU | M4))*/ /*GPIO-39 SynapticsIRQ*/\
	/*MUX_VAL(CP(GPMC_A9),        (IEN  | PTD | EN  | OFF_IN_PD | M4))*/ /*GPIO-42*/\
	/*MUX_VAL(CP(GPMC_nCS7),      (IEN  | PTD | EN  | OFF_IN_PD | M4))*/ /*GPI0-58*/\


/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_DEFAULT_ES2();
#ifdef CFG_TFT_DISPLAY
	MUX_TFT_ES2();
#endif
}

/******************************************************************************
 * Routine: update_mux()
 * Description:Update balls which are different between boards.  All should be
 *             updated to match functionality.  However, I'm only updating ones
 *             which I'll be using for now.  When power comes into play they
 *             all need updating.
 *****************************************************************************/
void update_mux(u32 btype, u32 mtype)
{
	/* NOTHING as of now... */
}

int board_late_init(void)
{
  return determine_boot_type();
}

