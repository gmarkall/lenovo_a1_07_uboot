
#ifndef __OFF_MODE_H__
#define __OFF_MODE_H__

/***
 ** Edian - big and little
***/
#define __force	__attribute__((force))
#define __bitwise __attribute__((bitwise))
typedef __u16 __bitwise __be16;
#define __swab16(x) ((__u16)(				\
	(((__u16)(x) & (__u16)0x00ffU) << 8) |			\
	(((__u16)(x) & (__u16)0xff00U) >> 8)))
#define cpu_to_be16(val)	((__force __be16)__swab16((__u16)(val)))
#define be16_to_cpu(val)	(__swab16((__force __u16)(__be16)(val)))

//* Error number
#define	EINVAL		22	/* Invalid argument */

/***
 ** Battery Control
***/

// Slave Address
#define DS2786_CHIP_SLAVE_ADDRESS 0x36

#define BQ27541_CHIP_SLAVE_ADDRESS 0x55 /* <--LH_SWRD_CL1_Mervins@2011.05.10--> */
//BQ27541
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_RSOC		0x2c
// Battery Register
#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A

#define MAX17043_REG_VCELL		0x02
#define MAX17043_REG_SOC		  0x04
#define MAX17043_REG_MODE		  0x06 /* Relative State-of-Charge */
#define MAX17043_REG_VERSION	0x08
#define MAX17043_REG_CONFIG		0x0C
#define MAX17043_REG_COMMAND	0xFE

// DS278x
#define DS278x_REG_VOLT_MSB	0x0C
#define DS278x_REG_TEMP_MSB	0x0A
#define DS278x_REG_CURRENT_MSB	0x0E
#define DS2786_REG_RARC		0x02	/* Remaining active relative capacity */

/* <--LH_SWRD_CL1_TGG@2011.07.12--> start*/
#define	ICON_NUM_LOW_ZERO_ADDR			(LCD_LOGO_PHY_ADDR - 0x100000)
#define	ICON_NUM_LOW_ONE_ADDR			(ICON_NUM_LOW_ZERO_ADDR - 0x8000)
#define	ICON_NUM_LOW_TWO_ADDR			(ICON_NUM_LOW_ONE_ADDR - 0x8000)
#define	ICON_NUM_LOW_THREE_ADDR			(ICON_NUM_LOW_TWO_ADDR - 0x8000)
#define	ICON_NUM_LOW_FOUR_ADDR			(ICON_NUM_LOW_THREE_ADDR - 0x8000)
#define	ICON_NUM_LOW_FIVE_ADDR			(ICON_NUM_LOW_FOUR_ADDR - 0x8000)
#define	ICON_NUM_LOW_SIX_ADDR			(ICON_NUM_LOW_FIVE_ADDR - 0x8000)
#define	ICON_NUM_LOW_SEVEN_ADDR			(ICON_NUM_LOW_SIX_ADDR - 0x8000)
#define	ICON_NUM_LOW_EIGHT_ADDR			(ICON_NUM_LOW_SEVEN_ADDR - 0x8000)
#define	ICON_NUM_LOW_NINE_ADDR			(ICON_NUM_LOW_EIGHT_ADDR - 0x8000)
#define	ICON_NUM_LOW_NONE_ADDR			(ICON_NUM_LOW_NINE_ADDR - 0x8000)

#define	ICON_NUM_HIGH_ZERO_ADDR			(ICON_NUM_LOW_NONE_ADDR - 0x8000)
#define	ICON_NUM_HIGH_ONE_ADDR			(ICON_NUM_HIGH_ZERO_ADDR - 0x8000)
#define	ICON_NUM_HIGH_TWO_ADDR			(ICON_NUM_HIGH_ONE_ADDR - 0x8000)
#define	ICON_NUM_HIGH_THREE_ADDR		(ICON_NUM_HIGH_TWO_ADDR - 0x8000)
#define	ICON_NUM_HIGH_FOUR_ADDR			(ICON_NUM_HIGH_THREE_ADDR - 0x8000)
#define	ICON_NUM_HIGH_FIVE_ADDR			(ICON_NUM_HIGH_FOUR_ADDR - 0x8000)
#define	ICON_NUM_HIGH_SIX_ADDR			(ICON_NUM_HIGH_FIVE_ADDR - 0x8000)
#define	ICON_NUM_HIGH_SEVEN_ADDR		(ICON_NUM_HIGH_SIX_ADDR - 0x8000)
#define	ICON_NUM_HIGH_EIGHT_ADDR		(ICON_NUM_HIGH_SEVEN_ADDR - 0x8000)
#define	ICON_NUM_HIGH_NINE_ADDR			(ICON_NUM_HIGH_EIGHT_ADDR - 0x8000)
#define	ICON_NUM_HIGH_NONE_ADDR			(ICON_NUM_HIGH_NINE_ADDR - 0x8000)

#define	ICON_BTY_CHARGE_BKGD_ADDR		(ICON_NUM_HIGH_NONE_ADDR - 0x20000)
#define	ICON_BTY_CHARGE_FULL_ADDR		(ICON_BTY_CHARGE_BKGD_ADDR - 0x20000)
#define	ICON_LIGHT_NOING_ADDR			(ICON_BTY_CHARGE_FULL_ADDR - 0x8000)
#define	ICON_NO_LIGHT_NOING_ADDR		(ICON_LIGHT_NOING_ADDR - 0x8000)
/* <--LH_SWRD_CL1_TGG@2011.07.12--> end*/

enum charging_supply {
	CABLE_UNKNOW,
	CABLE_AC,
	CABLE_USB,
	
};
enum charging_operation {
	CHARGING_START_100MA,
	CHARGING_START_500MA,
	CHARGING_START_1000MA,
	CHARGING_STOP,
};

enum bootup_reason {
	BOOTUP_UNKNOWN = 0x00,
	BOOTUP_BATTERY_DETECT = 0x40,
	BOOTUP_CABLE_IN = 0x04,
	BOOTUP_POWER_KEY = 0x01,
};

enum power_supply_property {
	/* Properties of type `int' */
	POWER_SUPPLY_PROP_PRESENT = 0,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY, /* in percents! */
	POWER_SUPPLY_PROP_TEMP,
};
union power_supply_propval {
	int intval;
	const char *strval;
};
// Export function
extern int ds2786_battery_get_property(enum power_supply_property psp,union power_supply_propval *val);
extern void ds2786_battery_info();
extern void ds2786_gauge_reset(void);
extern int dc_ok_detect(void);
extern int chg_status_detect(void);
extern void chg_led_set(int on);
extern void chg_led_set_green(int on);


#endif // __OFF_MODE_H__