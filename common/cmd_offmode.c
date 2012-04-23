#include <common.h>
#include <command.h>
#include <i2c.h>
#include <twl4030.h>
#include <off_mode.h>
#include <asm/io.h>


extern void ds2786_battery_info();
//extern void twl4030_poweroff();

/* <--LH_SWRD_CL1_TGG@2011.11.15--> start*/

#ifdef CONFIG_CL1_3G_VERSION
void voltagesocchange(int *soc)
{
	int tmp;
	tmp = *soc;
	if(tmp <16)
		*soc = tmp/3;
	else
		*soc = (950*(tmp-15)/85+5)/10+5;	
}
#else
void voltagesocchange(int *soc)
{
;
}
#endif
/* <--LH_SWRD_CL1_TGG@2011.11.15--> end*/

/* <--LH_SWRD_CL1_TGG@2011.07.18--> start*/
static void display_draw_charge();
static void charge_logo_init();
void change_cpu_freq(unsigned char mode);
void change_mem_freq(unsigned char mode);

#define mdelay(n) ({ unsigned long msec = (n); while (msec--) udelay(1000); })

#define CM_CLKEN_PLL 0x48004d00
#define PLL_FAST_RELOCK_BYPASS 6
#define BIT0 (1<<0)
#define CM_IDLEST_CKGEN 0x48004d20
#define LDELAY 12000000
#define CM_CLKSEL1_PLL 0x48004d40
#define PLL_LOCK 7
#define CM_CLKSEL1_PLL_MPU 0x48004940
#define CM_CLKEN_PLL_MPU 0x48004904
#define CM_IDLEST_PLL_MPU 0x48004924



volatile unsigned int g_panle_cnt = 0;
int g_hundred_bit = 0;
int g_ten_bit = 0;
int g_low_bit = 0;

int g_hundred_bit_tmp = 0;
int g_ten_bit_tmp = 0;
int g_low_bit_tmp = 0;
/* <--LH_SWRD_CL1_TGG@2011.07.18--> end*/

#if CONFIG_MAX17043_GAUGE
#define POWER_ON_VOLTAGE 360000
#elif CONFIG_TWL4030_MADC_VBAT
#define POWER_ON_VOLTAGE 3600
#elif CONFIG_BQ27541_VBAT
#define POWER_ON_VOLTAGE 3600 /* <--LH_SWRD_CL1_Mervins@2011.05.10--> */
#define POWER_ON_CAPACITY 5  /* <--LH_SWRD_CL1_Mervins@2011.06.03--> */
#endif

#define mdelay(n) ({ unsigned long msec = (n); while (msec--) udelay(1000); })

int g_bootup_reason = BOOTUP_UNKNOWN;

// handle the special case, press power key 8s to shutdon device 
#define BOOTUP_8S_OFF	(BOOTUP_BATTERY_DETECT|BOOTUP_POWER_KEY)
#define BOOTUP_8S_OFF_CABLE	(BOOTUP_BATTERY_DETECT|BOOTUP_POWER_KEY|BOOTUP_CABLE_IN)

//&*&*&*HC1_20110503, Adjust long press time (ref. bsp spec v0.6)
//&*&*&*HC1_20110428, enable pwr key long press function 
#if CONFIG_PWRKEY_LONGPRESS
#define PWRKEY_PRESS_TIME_STARTUP 1
#define PWRKEY_PRESS_TIME_CHARGING 2
//&*&*&*HC2_20110503, Adjust long press time (ref. bsp spec v0.6)

// power key long press check
int pwrkey_press_check(u32 sec, int boot_reason)
{
	int ret = 1;
	int i;
	int step = 250;
	int time_unit = 1000;

	//printf("S pwrkey_press_check(%d)...\n", sec);

	// limit the long press check time		
	if (sec < 0 || sec > 10)
		return 0;

	if (boot_reason == BOOTUP_POWER_KEY)
	{
		
		if (sec%2)
			time_unit = 500;			
		else
			sec/=2;	

		step = 500;
	}	

	for (i=0; i<sec*time_unit; i+=step)
	{
		mdelay(step);

		// verify the STS_PWON bit (active high with pwr key press)	
		if ((twl4030_hw_sts_get() & 0x1))
			continue;
		else
		{
			ret = 0;
			break;
		}	
	}
	if(ret==1){
		/* <--LH_SWRD_CL1_Mervins@2011.06.20 Add vibrator EN*/		
		gpio_t *gpio5_base = (gpio_t *)OMAP34XX_GPIO5_BASE;
		gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
		sr32((u32)&gpio2_base->res2[0], 6, 1, 0);
		mdelay(1000);
		/** Enable vibrator_EN  GPIO_156 **/ 
		sr32((u32)&gpio5_base->oe, 28, 1, 0);
		sr32((u32)&gpio5_base->setdataout, 28, 1, 1);
		mdelay(1000);
		sr32((u32)&gpio5_base->cleardataout, 28, 1, 1);
		/* LH_SWRD_CL1_Mervins@2011.06.20--> */
		}
	printf("E pwrkey_press_check..., ret=%d, step=%d\n", ret, step);
	
	return ret;	
}
#endif // CONFIG_PWRKEY_LONGPRESS
//&*&*&*HC2_20110428, enable pwr key long press function 


void off_mode_charging( void )
{
	int CableInType = CABLE_USB;
	int bootup_reason = BOOTUP_UNKNOWN;
	int led_flash = 0;
	/* <--LH_SWRD_CL1_TGG@2011.07.20--> */
	gpio_t *g_gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
	int keyflag=0;	
	int charge_in_ac = 0;
	/* <--LH_SWRD_CL1_TGG@2011.07.20--> */
	unsigned int key_press_cnt = 0;
	int panel_flag = 0;
#if CONFIG_MAX17043_GAUGE
	int reset_gauge = 0;
	union power_supply_propval BatteryVol = {0,};
	union power_supply_propval BatteryCap = {0,};
#elif CONFIG_TWL4030_MADC_VBAT
	int BatteryCap = 0, BatteryVol = 0;
#elif CONFIG_BQ27541_VBAT
	int reset_gauge = 0;
	int BatteryCap = 0;
	int BatteryVol = 0;
#endif

#if CONFIG_MAX17043_GAUGE
	select_bus(1, 100);	
	ds2786_gauge_reset();
	mdelay(1000);
#elif CONFIG_TWL4030_MADC_VBAT
	//twl4030_vadc_onoff(1);
	//twl4030_madc_init();
#endif

	bootup_reason = g_bootup_reason;
	
	/* <--LH_SWRD_CL1_TGG@2011.07.18--> */
	charge_logo_init();
	run_command("logo_black", 0);
	sr32((u32)&g_gpio2_base->oe, 6, 1, 0);
	select_bus(1, 100);
	ds278x_battery_get_property(POWER_SUPPLY_PROP_CAPACITY, &BatteryCap);
	printf("old soc---->%3d\n", BatteryCap);
	voltagesocchange(&BatteryCap);
	printf("new soc---->%3d\n", BatteryCap);
	g_hundred_bit = BatteryCap / 100;
	g_ten_bit = (BatteryCap % 100)/10;
	g_low_bit = (BatteryCap % 100)%10;
	if(dc_ok_detect() == 0) {
		//charge_logo_init();
		display_draw_charge();
		if(panel_flag==0){
			run_command("panel",0);
			panel_flag=1;
		}
	}
	/* <--LH_SWRD_CL1_TGG@2011.07.018--> */

	
	while (1)
	{
#if CONFIG_MAX17043_GAUGE
		// select I2C2
		select_bus(1, 100);	
		ds278x_battery_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW, &BatteryVol);
		ds278x_battery_get_property(POWER_SUPPLY_PROP_CAPACITY, &BatteryCap);
		printf("**** Off Mode Charging(from MAX17043 Gauge) *****\n");
		printf("****      VOLTAGE: %d    ********\n", BatteryVol.intval);
		printf("****      CAPACITY: %d        *******\n", BatteryCap.intval);
		printf("****  End Of Off Mode Charging  ****\n");
		//ds2786_battery_info();
#elif CONFIG_TWL4030_MADC_VBAT
		BatteryVol = twl4030_get_battery_voltage();
		printf("**** Off Mode Charging(from MADC) *****\n");
		printf("****      VOLTAGE: %d   ********\n", BatteryVol);
		printf("****  End Of Off Mode Charging  ****\n");
/* <--LH_SWRD_CL1_Mervins@2011.05.10 */		
#elif CONFIG_BQ27541_VBAT
		select_bus(1, 100);
		ds278x_battery_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW, &BatteryVol);
		if(BatteryVol==1){
			printf("read battery Voltage error!******************************************\n");
			//continue;
		}
		ds278x_battery_get_property(POWER_SUPPLY_PROP_CAPACITY, &BatteryCap);
		voltagesocchange(&BatteryCap);
		if(BatteryCap==1){
			printf("read battery BatteryCap error!***************************************\n");
			//continue;
		}
		printf("**** Off Mode Charging Battery  detect by BQ27541 *****\n");
		printf("****             VOLTAGE:%d,                      ********\n", BatteryVol);
		printf("****             CAPACITY:%d%,                    ********\n", BatteryCap);
		printf("****        End Of Off Mode Charging              ****\n");
/* LH_SWRD_CL1_Mervins@2011.05.10--> */		
#endif
		/* <--LH_SWRD_CL1_TGG@2011.07.18--> */
		
		g_hundred_bit_tmp = g_hundred_bit;
		g_ten_bit_tmp = g_ten_bit;
		g_low_bit_tmp = g_low_bit;
		g_hundred_bit = BatteryCap / 100;
		g_ten_bit = (BatteryCap % 100)/10;
		g_low_bit = (BatteryCap % 100)%10;
		g_panle_cnt = g_panle_cnt + 1;
		if(g_panle_cnt == 9)
			g_panle_cnt = 15;
		if(g_low_bit != g_low_bit_tmp)
			display_draw_charge();
		
		if(dc_ok_detect() == 0)
		{	
			if(g_panle_cnt > 10) {
				//sr32((u32)&g_gpio2_base->res2[0], 6, 1, 0);
				if(panel_flag==1){
				
						run_command("paneloff", 0);
						change_cpu_freq(0);
						//change_mem_freq(0);
						
						panel_flag=0;
					}
			}
			else {
				if(panel_flag==0){
					//sr32((u32)&g_gpio2_base->setdataout, 6, 1, 1);
					//change_mem_freq(1);
					change_cpu_freq(1);
					mdelay(50);
					run_command("logo_black", 0);
					mdelay(50);
					display_draw_charge();
					run_command("panel", 0);
					panel_flag=1;
				}
			}
		}
		
		printf("#############g_panle_cnt = %4d ##############\n", g_panle_cnt);
		/* <--LH_SWRD_CL1_TGG@2011.07.18--> */
		
		mdelay(1000);
		//upadte_display_area(517, 287, 27, 25, ICON_LIGHT_NOING_ADDR);
		// select I2C1
		select_bus(0, 100);
		
		if((dc_ok_detect() == 0)&&(twl4030_bootup_reason_get() & BOOTUP_POWER_KEY))
		{
//			mdelay(1000);
			printf("======***here I am !***=====\n");
			if(g_panle_cnt == 1)
				g_panle_cnt = 1;
			else if(g_panle_cnt == 21)
				g_panle_cnt = 21;
			else if(g_panle_cnt < 10)
				g_panle_cnt = 20;
			else
				g_panle_cnt = 0;
	
			keyflag = 1;
		}
		
		if ((keyflag == 1) || (bootup_reason & BOOTUP_POWER_KEY))
		{
			keyflag=0;
//&*&*&*HC1_20110503, Adjust long press time (ref. bsp spec v0.6)
//&*&*&*HC1_20110428, enable pwr key long press function 
#if CONFIG_PWRKEY_LONGPRESS
			if ((bootup_reason != BOOTUP_POWER_KEY) && (!pwrkey_press_check(PWRKEY_PRESS_TIME_CHARGING, bootup_reason)))
			{
				bootup_reason = BOOTUP_UNKNOWN;
				continue;		
			}
#endif // CONFIG_PWRKEY_LONGPRESS			
//&*&*&*HC2_20110428, enable pwr key long press function 
//&*&*&*HC2_20110503, Adjust long press time (ref. bsp spec v0.6)

#if (defined(CONFIG_TWL4030_MADC_VBAT) || defined(CONFIG_MAX17043_GAUGE) || CONFIG_BQ27541_VBAT) /* <--LH_SWRD_CL1_Mervins@2011.05.10--> */

#if CONFIG_MAX17043_GAUGE
			if(BatteryVol.intval < POWER_ON_VOLTAGE)
#elif CONFIG_TWL4030_MADC_VBAT
			if(BatteryVol < POWER_ON_VOLTAGE)
#elif CONFIG_BQ27541_VBAT
			if(dc_ok_detect() == 1)
				charge_in_ac = 0;
			else 
			{
				if(twl4030_get_cable_type() == USB_EVENT_VBUS)
					charge_in_ac = 0;
				else
					charge_in_ac = 1;

			}
			if((charge_in_ac == 0)&&(BatteryCap < POWER_ON_CAPACITY)) /* <--LH_SWRD_CL1_Mervins@2011.05.10--> */
#endif
			{
				bootup_reason = BOOTUP_UNKNOWN;
				printf("Battery capacity < 5%, can't boot up !!!\n");
			/* <--LH_SWRD_CL1_Mervins@2011.06.03--> */
#ifdef CONFIG_BOOTUP_LOGO
#ifdef CONFIG_SHOW_BATTERY_STATE

			

				if(BatteryCap < POWER_ON_CAPACITY)
				{
					run_command("logo_black",0);//
					run_command("display_battery_low",0);//
					run_command("panel",0);//
					mdelay(6000);//
					run_command("paneloff",0);
					run_command("logo_black",0);//
					display_draw_charge();
				}
#endif
#endif 
			/* <--LH_SWRD_CL1_Mervins@2011.06.03--> */			
				
			}	
			else 

				break;
#else
			printf("No battery voltage checking !!!\n");
			break;
#endif
//&*&*&*HC2_20110103, enable the battery check
//&*&*&*HC2_20101230, disable the battery check
		}
		if(dc_ok_detect() == 1)
		{
			printf("Not a vaild long press !!! Please press and hold pwr key more than 1 sec.\n");

			twl4030_poweroff();
			break;
		}
		else
		{
			if(twl4030_get_cable_type() == USB_EVENT_VBUS)
				CableInType = CABLE_USB;
			else
				CableInType = CABLE_AC;
			if(CableInType == CABLE_AC)
			{
				printf("AC: ");
				charging_mode(CHARGING_START_1000MA);
			}
			else
			{
				printf("USB: ");
				charging_mode(CHARGING_START_500MA);
			}
		}

		// control charging LED
		if(dc_ok_detect() == 1)
		{
			chg_led_set(0);
			chg_led_set_green(0);
		}
		else if (BatteryCap < 95)
		{	//chg_led_set(0);
			chg_led_set(1);
			chg_led_set_green(0);
		}
		else if(BatteryCap >= 95)
		{
			chg_led_set(0);
			chg_led_set_green(1);			
		}
/* <--LH_SWRD_CL1_Mervins@2011.06.09:modify charge full power off.--> */
#if 0 // temporarily use HW terminal mechanism 
		if (BatteryCap > 95)
		{
			printf("Charging Complete... \n");
			chg_led_set(0);
			chg_led_set_green(1);
//			twl4030_poweroff();
		}		
#endif
/* <--LH_SWRD_CL1_Mervins@2011.06.09--> */
		mdelay(2000);
		if(BatteryCap < 11)	
		   chg_led_set(0);
		//if(g_hundred_bit != 1)
		//	upadte_display_area(517, 287, 27, 25, ICON_NO_LIGHT_NOING_ADDR);
#if CONFIG_MAX17043_GAUGE 	/* <--LH_SWRD_CL1_Mervins@2011.05.07--> */
		if(reset_gauge == 0)
		{	
			printf("max17043 gauge reset !!!\n");
			select_bus(1, 100);	
			ds2786_gauge_reset();
			mdelay(1000);
			reset_gauge = 1;
		}
#endif
	}	
}

int off_mode_start( void )
{
	int ret = 0;
	u8 value;
	int i = 0;
	int bootup_reason = BOOTUP_UNKNOWN;
	int BatteryVol = 0;
	int icheck_cnt  = 0;
	gpio_t *g_gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
	printf(" +++ off_mode_start +++\n");

	/** <--LH_SWRD_CL1_Tanggang@2011.07.07, Add check battery Model*/
	select_bus(1, 100);
	ds278x_battery_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW, &BatteryVol);
	while(BatteryVol < 3200)
	{
		mdelay(1000);
		chg_led_set(1);
		select_bus(1, 100);
		printf("Low BAT!BatteryVol = %8d\n", BatteryVol);
		if(dc_ok_detect())
		{
			select_bus(0, 100);
			twl4030_poweroff();
		}
		mdelay(2000);
		chg_led_set(0);
		ds278x_battery_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW, &BatteryVol);	
		}	
	/** <--LH_SWRD_CL1_Tanggang@2011.07.07, Add check battery Model*/
	
	g_bootup_reason = twl4030_bootup_reason_get();

	if (g_bootup_reason == BOOTUP_8S_OFF)
	{
		printf("[pwr key 8s]:  ");
		bootup_reason = BOOTUP_POWER_KEY;
		g_bootup_reason = BOOTUP_POWER_KEY;
	}
	else if (g_bootup_reason == BOOTUP_8S_OFF_CABLE)
	{
		printf("[pwr key 8s + cable]:  ");
		bootup_reason = BOOTUP_CABLE_IN;
		g_bootup_reason = BOOTUP_CABLE_IN;		
	}
	else
		bootup_reason = g_bootup_reason;

	// select I2C0
	select_bus(0, 100);
	/** <--LH_SWRD_CL1_Tanggang@2011.08.15, start*/
	unsigned int *pRestFlag = (unsigned int *)(0x9ff00000);
	unsigned int *pRebootFlag = (unsigned int *)(0x48307258);
	printf("PRM reboot flag:%x\n",pRebootFlag[0]);
	printf("rest flag:%x\n",pRestFlag[0]);
	
	if (0x87654321 == pRestFlag[0]){
		printf("@@@ Boot reason = restart!!!  @@@\n");
	}/** <--LH_SWRD_CL1_Tanggang@2011.08.15, end*/
	else if (bootup_reason & BOOTUP_POWER_KEY)
	{
		printf("power key detect...\n");		
		
//&*&*&*HC1_20110503, Adjust long press time (ref. bsp spec v0.6)		
//&*&*&*HC1_20110428, enable pwr key long press function 
#if CONFIG_PWRKEY_LONGPRESS
		if (pwrkey_press_check(PWRKEY_PRESS_TIME_STARTUP, g_bootup_reason))
			g_bootup_reason = BOOTUP_POWER_KEY;
			else
			g_bootup_reason = BOOTUP_UNKNOWN;
#endif // #if CONFIG_PWRKEY_LONGPRESS		
//&*&*&*HC2_20110428, enable pwr key long press function 
//&*&*&*HC2_20110503, Adjust long press time (ref. bsp spec v0.6)

		off_mode_charging();
	}
	else if (bootup_reason &  BOOTUP_CABLE_IN)
	{
		printf("cable detect...\n");
		off_mode_charging();
	}
	else if (bootup_reason & BOOTUP_BATTERY_DETECT)
	{
		printf("battery detect...\n");
		twl4030_poweroff();
	}
	else
	{
//&*&*&*HC1_20110221, let reboot pass the check 	
		//printf("unknown ...\n");
		//twl4030_poweroff();
		//unsigned int *pRestFlag = (unsigned int *)(0x9ff00000);
		printf("rest flag:%x\n",pRestFlag[0]);
		if (0x12345678 == pRestFlag[0])
		{
			off_mode_charging();
		}
		printf("reboot ...\n");		
//&*&*&*HC2_20110221, let reboot pass the check 			
	}
	//32((u32)&g_gpio2_base->res2[0], 6, 1, 0);
	run_command("paneloff",0);
	return g_bootup_reason;

}

int do_offmode (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	if (argc == 2 && strcmp (argv[1], "start") == 0)
	off_mode_start();
			
	return 0;
	
}

U_BOOT_CMD(
	offmode, CFG_MAXARGS, 1,	do_offmode,
	"offmode battery- get battery information\n",
);

/* <--LH_SWRD_CL1_TGG@2011.07.20--> start*/
void display_draw_charge()
{
	printf("######Now, display_draw_charge! ######\n");
	if(g_hundred_bit == 1)
		upadte_display_area(425, 212, 170, 171, ICON_BTY_CHARGE_FULL_ADDR);
	else
	{
		upadte_display_area(425, 212, 170, 171, ICON_BTY_CHARGE_BKGD_ADDR);
		
		switch(g_ten_bit)
			{
				case 0 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_NONE_ADDR); break;
				case 1 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_ONE_ADDR); break;
				case 2 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_TWO_ADDR); break;
				case 3 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_THREE_ADDR); break;
				case 4 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_FOUR_ADDR); break;
				case 5 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_FIVE_ADDR); break;
				case 6 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_SIX_ADDR); break;
				case 7 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_SEVEN_ADDR); break;
				case 8 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_EIGHT_ADDR); break;
				case 9 : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_NINE_ADDR); break;
				default : upadte_display_area(482, 313, 28, 18, ICON_NUM_HIGH_NONE_ADDR); break;
			}

			if(g_ten_bit==0 && g_hundred_bit==0 && g_low_bit==0)
				g_low_bit = 1;

			switch(g_low_bit)
			{
				case 0 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_ZERO_ADDR); break;
				case 1 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_ONE_ADDR); break;
				case 2 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_TWO_ADDR); break;
				case 3 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_THREE_ADDR); break;
				case 4 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_FOUR_ADDR); break;
				case 5 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_FIVE_ADDR); break;
				case 6 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_SIX_ADDR); break;
				case 7 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_SEVEN_ADDR); break;
				case 8 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_EIGHT_ADDR); break;
				case 9 : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_NINE_ADDR); break;
				default : upadte_display_area(482, 295, 28, 18, ICON_NUM_LOW_NONE_ADDR); break;
			}
		}
}

void charge_logo_init()
{
	char cmd[64];
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1700000 8000", ICON_NUM_LOW_ZERO_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1700000 8000", ICON_NUM_LOW_ZERO_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1708000 8000", ICON_NUM_LOW_ONE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1710000 8000", ICON_NUM_LOW_TWO_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1718000 8000", ICON_NUM_LOW_THREE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1720000 8000", ICON_NUM_LOW_FOUR_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1728000 8000", ICON_NUM_LOW_FIVE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1730000 8000", ICON_NUM_LOW_SIX_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1738000 8000", ICON_NUM_LOW_SEVEN_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1740000 8000", ICON_NUM_LOW_EIGHT_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1748000 8000", ICON_NUM_LOW_NINE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1750000 8000", ICON_NUM_LOW_NONE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1758000 8000", ICON_NUM_HIGH_ZERO_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1760000 8000", ICON_NUM_HIGH_ONE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1768000 8000", ICON_NUM_HIGH_TWO_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1770000 8000", ICON_NUM_HIGH_THREE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1778000 8000", ICON_NUM_HIGH_FOUR_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1780000 8000", ICON_NUM_HIGH_FIVE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1788000 8000", ICON_NUM_HIGH_SIX_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1790000 8000", ICON_NUM_HIGH_SEVEN_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x1798000 8000", ICON_NUM_HIGH_EIGHT_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x17a0000 8000", ICON_NUM_HIGH_NINE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x17a8000 8000", ICON_NUM_HIGH_NONE_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x17b0000 20000", ICON_BTY_CHARGE_BKGD_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x17d0000 20000", ICON_BTY_CHARGE_FULL_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x17f0000 8000", ICON_LIGHT_NOING_ADDR);	
	run_command(cmd,0);
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 0x17f8000 8000", ICON_NO_LIGHT_NOING_ADDR);	
	run_command(cmd,0);
}

void change_mem_freq(unsigned char mode)
{
	sr32(CM_CLKEN_PLL, 0, 3, PLL_FAST_RELOCK_BYPASS);
	wait_on_value(BIT0, 0, CM_IDLEST_CKGEN, LDELAY);

	mdelay(500);

	if (mode)
	{
		printf("change mem freq high...\n");
		// 200 MHz
		sr32(CM_CLKSEL1_PLL, 16, 11, 200);	/* Set M */
	}
	else
	{
		printf("change mem freq low...\n");
		// 133 MHz
		sr32(CM_CLKSEL1_PLL, 16, 11, 133);	/* Set M */
	}

	sr32(CM_CLKEN_PLL, 0, 3, PLL_LOCK);		/* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_CKGEN, LDELAY);

	mdelay(500);

	printf("CM_CLKSEL1_PLL = 0x%x\n", readl(CM_CLKSEL1_PLL));
}

void change_cpu_freq(unsigned char mode)
{

	if (mode)
	{
		printf("change cpu freq high...\n");		
		// 600 MHz
		sr32(CM_CLKSEL1_PLL_MPU, 8, 11, 300);	/* Set M */
	}
	else
	{
		printf("change cpu freq low...\n");		
		// 26 MHz
		sr32(CM_CLKSEL1_PLL_MPU, 8, 11, 13);	/* Set M */
	}

	sr32(CM_CLKEN_PLL_MPU, 0, 3, PLL_LOCK); /* lock mode */
	wait_on_value(BIT0, 1, CM_IDLEST_PLL_MPU, LDELAY);	

	printf("CM_CLKSEL1_PLL_MPU = 0x%x\n", readl(CM_CLKSEL1_PLL_MPU));
}

void do_luboot()
{
	run_command("mmcinit 0",0);
	mdelay(1000);
	run_command("fatload mmc 0 82000000 u-boot.bin",0);
	mdelay(1000);
	run_command("mmcinit 1",0);
	mdelay(1000);
	run_command("mmc 1 write 82000000 80000 100000",0);
	mdelay(1000);
}

void do_lkernel()
{
	run_command("mmcinit 0",0);
	mdelay(1000);
	run_command("fatload mmc 0 82000000 uImage",0);
	mdelay(1000);
	run_command("mmcinit 1",0);
	mdelay(1000);
	run_command("mmc 1 write 82000000 300000 500000",0);
	mdelay(1000);

}


U_BOOT_CMD(
	luboot,	1,	1,	do_luboot,
	"",
	""
);

U_BOOT_CMD(
	lkernel,	1,	1,	do_lkernel,
	"",
	""
);

/* <--LH_SWRD_CL1_TGG@2011.07.20--> end*/

