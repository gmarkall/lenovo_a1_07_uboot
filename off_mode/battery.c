/*
 * (C) Copyright 2010 FOXCONN
 * Author : aimar liu <aimar.ts.liu@foxconn.com>
 * 
 */

#include <common.h>
#include <exports.h>
#include <asm/types.h>
#include <off_mode.h>
#include <command.h>
#include <malloc.h>
#include <devices.h>
#include <asm/io.h>
#include <asm-arm/arch-omap3/mux.h>

typedef unsigned long  uint32;
typedef long           int32;
typedef unsigned short uint16;
typedef short          int16;
typedef unsigned char  uint8;
typedef signed char    int8;

static int ds278x_read(u8 reg, int *rt_value, int b_single)
{
	int ret;
	int alen = 1;
	int lineBytes = 2;
	if(b_single)
		lineBytes = 1;
	ret = i2c_read(BQ27541_CHIP_SLAVE_ADDRESS, reg, alen, (uchar *)rt_value, lineBytes);  /* <--LH_SWRD_CL1_Mervins@2011.05.10--> */
	if(ret)
			puts ("Error reading the chip.\n");
	//*rt_value = be16_to_cpu(*rt_value);

	return ret;
}


static int ds278x_write(u8 reg, unsigned int *rt_value, int b_single)
{
	int ret;
	int alen = 1;
	int lineBytes = 2;
	if(b_single)
		lineBytes = 1;
	ret = i2c_write(DS2786_CHIP_SLAVE_ADDRESS, reg, alen, (uchar *)rt_value, lineBytes);
	if(ret)
			puts ("Error write the chip.\n");
	//*rt_value = be16_to_cpu(*rt_value);

	return ret;
}

static int ds278x_read_multi(u8 reg, uchar *rt_value, int lineBytes)
{
	int ret;
	int alen = 1;
	ret = i2c_read(DS2786_CHIP_SLAVE_ADDRESS, reg, alen, (uchar *)rt_value, lineBytes);
	if(ret)
			puts ("Error reading the chip.\n");
	//*rt_value = be16_to_cpu(*rt_value);

	return ret;
}


static int ds278x_write_multi(u8 reg, uchar *rt_value, int lineBytes)
{
	int ret;
	int alen = 1;
	ret = i2c_write(DS2786_CHIP_SLAVE_ADDRESS, reg, alen, (uchar *)rt_value, lineBytes);
	if(ret)
		puts ("Error write the chip.\n");

	return ret;
}

int temperature_value_change(int reg_value)
{
	uint8 value_high_bit;
	uint8 value_low_bit;
	int16 value;
	int temp;
	
	value_high_bit = reg_value;
	value_low_bit = reg_value>>8;
	value = ((value_high_bit<<8) |value_low_bit);

	temp = (value/32) * 125 / 1000;

	return temp;
}

static int ds278x_battery_temperature()
{
	int ret;
	int temp = 0;

	ret = ds278x_read(DS278x_REG_TEMP_MSB, &temp, 0);
	if (ret) {
		printf("error reading temperature, ret=%d\n", ret);
		return ret;
	}

	return temperature_value_change(temp);
}

int voltage_value_change(int reg_value)
{
	int value_high_bit = 0;
	int value_low_bit = 0;
	int value = 0;
	
	value_high_bit = (reg_value & 0xff)<<4;
	value_low_bit = (reg_value & 0xff00)>>12;
	value = (value_high_bit + value_low_bit);
	value = value * 125;

	return value;
}

static int ds2786_battery_voltage()
{
	int ret;
	int volt = 0;

	ret = ds278x_read(BQ27541_REG_VOLT, &volt, 0);/* <--LH_SWRD_CL1_Mervins@2011.05.10--> */
	if (ret) {
		printf("error reading voltage, ret=%d\n", ret);
		return ret;
	}

	return volt;//voltage_value_change(volt);/* <--LH_SWRD_CL1_Mervins@2011.05.10--> */
}

int current_value_change(int reg_value)
{

	uint8 value_high_bit;
	uint8 value_low_bit;
	int16 value;
	int curr;

	value_high_bit = reg_value;
	value_low_bit = reg_value>>8;
	value = ((value_high_bit<<8) |value_low_bit);

	curr = (value/16)*(25*1000)/10;

	return curr;

}

static int ds2786_battery_current()
{
	int ret;
	int curr = 0;

	ret = ds278x_read(DS278x_REG_CURRENT_MSB, &curr, 0);
	if (ret) {
		printf("error reading current, ret=%d\n", ret);
		return ret;
	}

	return current_value_change(curr);
}

static int ds2786_battery_rsoc()
{
	int ret;
	int rsoc = 0;

	ret = ds278x_read(BQ27541_REG_RSOC, &rsoc, 0);/* <--LH_SWRD_CL1_Mervins@2011.05.10--> */
	if (ret) {
		printf("error reading relative State-of-Charge, ret=%d\n", ret);
		return ret;
	}

	printf("[off mode]max17043 real soc : %d\n", rsoc);

	if(rsoc > 100)
		rsoc = 100;

	return rsoc; 
}

int ds278x_battery_get_property(enum power_supply_property psp,
										union power_supply_propval *val)
{
		switch (psp) {
			case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			case POWER_SUPPLY_PROP_PRESENT:
					val->intval = ds2786_battery_voltage();
					if (psp == POWER_SUPPLY_PROP_PRESENT)
							val->intval = val->intval <= 1 ? 0 : 1;
					break;
			case POWER_SUPPLY_PROP_CURRENT_NOW:
					val->intval = ds2786_battery_current();
					break;
			case POWER_SUPPLY_PROP_CAPACITY:
					val->intval = ds2786_battery_rsoc();
					break;
			case POWER_SUPPLY_PROP_TEMP:
					val->intval = ds278x_battery_temperature();
					break;
			default:
					return -1;
		}

		return 0;
}

void ds2786_battery_info()
{
		union power_supply_propval val = {0,};
			
		printf("**** [Max17043]off mode charging *****\n");
		//ds278x_battery_get_property(POWER_SUPPLY_PROP_CURRENT_NOW, &val);
		//printf("***     CURRENT: %d   ********\n", val.intval);
		ds278x_battery_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
		printf("***     VOLTAGE: %d  ********\n", val.intval);
		ds278x_battery_get_property(POWER_SUPPLY_PROP_CAPACITY, &val);
		printf("***     CAPACITY: %d      ********\n", val.intval);
		printf("**** end of off mode charging ****\n");
		
		return;		
}

void charging_gpio_set(int dcm, int iusb, int usus, int cen)
{

#if 0 // there is no connection
	gpio_t *gpio1_base = (gpio_t *)OMAP34XX_GPIO1_BASE;
#endif
	gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
	gpio_t *gpio4_base = (gpio_t *)OMAP34XX_GPIO4_BASE;

#if 0 // there is no connection
	sr32((u32)&gpio1_base->oe, 11, 1, 0); // USUS: GPIO-11
#endif
	sr32((u32)&gpio2_base->oe, 10, 1, 0); // CEN: GPIO-42
	sr32((u32)&gpio2_base->oe, 29, 1, 0); // IUSB: GPIO-61
	sr32((u32)&gpio4_base->oe, 8, 1, 0); // DCM: GPIO-104	

	if (dcm)
		sr32((u32)&gpio4_base->setdataout, 8, 1, 1); 
	else
		//sr32((u32)&gpio4_base->cleardataout, 8, 1, 1); 
		sr32((u32)&gpio4_base->res2[0], 8, 1, 0); 

	//if (!iusb)
	//{
	//	sr32((u32)&gpio2_base->setdataout, 29, 1, 1); 
		//sr32((SCM_REGISTER_BASE+CONTROL_PADCONF_GPMC_nBE1), 3, 2, 3);
	//}	
	//else
	//{
		//sr32((u32)&gpio2_base->cleardataout, 29, 1, 1); 
	//	sr32((u32)&gpio2_base->res2[0], 29, 1, 0); 
		//sr32((SCM_REGISTER_BASE+CONTROL_PADCONF_GPMC_nBE1), 3, 2, 1);
	//}	

#if 0 // there is no connection
	if (usus)
		sr32((u32)&gpio1_base->setdataout, 11, 1, 1); 
	else
		//sr32((u32)&gpio1_base->cleardataout, 11, 1, 1); 
		sr32((u32)&gpio1_base->res2[0], 11, 1, 0); 	
#endif

	if (cen)
		sr32((u32)&gpio2_base->setdataout, 10, 1, 1); 
	else
		//sr32((u32)&gpio2_base->cleardataout, 10, 1, 1); 	
		sr32((u32)&gpio2_base->res2[0], 10, 1, 0);		


}

void charging_mode(enum charging_operation co)
{

	int dcm = 0;
	int iusb = 0;
	int usus = 0;
	int cen = 0;

	if (co > CHARGING_STOP || co < CHARGING_START_100MA)
		return;	

	/*----------------------------------------------------------------------
				DCM(GPIO-104)	IUSB(GPIO- 61)	USUS(GPIO-11)	CEN(GPIO-42)
		1000mA	H				X				X				L
		500mA	L				H				L				L
		100mA	L				L				L				L
	-----------------------------------------------------------------------*/	


	switch (co)
	{
		case CHARGING_START_100MA:
			printf("CHARGING_START_100MA...\n");
			break;

		case CHARGING_START_500MA:
			iusb = 1;
			printf("CHARGING_START_500MA...\n");			
			break;	

		case CHARGING_START_1000MA:
			dcm = 1;
			printf("CHARGING_START_1000MA...\n");			
			break;		
			
		case CHARGING_STOP:
			cen = 1;
			printf("CHARGING_STOP...\n");			
			break;
		default:
			cen = 1;
			printf("UNKNOWN ERROR !!!\n");			
			break;
	}

	charging_gpio_set(dcm, iusb, usus, cen);
	
}

int dc_ok_detect(void)
{
	int value;
	gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;

	// get DETECT_DEVICE pin (gpio-51)
	value =  read_gpio_value((u32)&gpio2_base->datain, 19);
	printf("[off mode charging]gpio2_base->datain[19] is %d\n", value);
	return value;
}

int chg_status_detect(void)
{
	int value;
//	gpio_t *gpio5_base = (gpio_t *)OMAP34XX_GPIO5_BASE;
	gpio_t *gpio1_base = (gpio_t *)OMAP34XX_GPIO1_BASE;


	// get CHG pin (gpio-149)
//	value =  read_gpio_value((u32)&gpio5_base->datain, 21);
	value =  read_gpio_value((u32)&gpio1_base->datain, 15);

	printf("[off mode charging]gpio5_base->datain[21] is %d\n", value);
	return value;
}

void chg_led_set(int on)
{
	int value;
	gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;

	// set LED_EN pin (gpio-59) 
	sr32((u32)&gpio2_base->oe, 27, 1, 0);

	if (on)
		sr32((u32)&gpio2_base->setdataout, 27, 1, 1); 	
	else
		//sr32((u32)&gpio4_base->cleardataout, 8, 1, 1); 
		sr32((u32)&gpio2_base->res2[0], 27, 1, 0);

	printf("[off mode charging]chg LED is %s\n", on==1? "ON":"OFF");
}

void chg_led_set_green(int on)
{
	int value;
	gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;

	// set LED_EN pin (gpio-60) 
	sr32((u32)&gpio2_base->oe, 28, 1, 0);

	if (on)
		sr32((u32)&gpio2_base->setdataout, 28, 1, 1); 	
	else
		//sr32((u32)&gpio4_base->cleardataout, 8, 1, 1); 
		sr32((u32)&gpio2_base->res2[0], 28, 1, 0);

	printf("[off mode charging]charge full LED is %s\n", on==1? "ON":"OFF");
}

void ds2786_gauge_reset(void)
{
	uchar reset_msb = 0x54; 
	uchar reset_lsb = 0x00;
	ds278x_write(MAX17043_REG_COMMAND, &reset_msb, 1);
	ds278x_write(MAX17043_REG_COMMAND+1, &reset_lsb, 1);
}


