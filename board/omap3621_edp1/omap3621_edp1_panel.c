/*
 * Encore panel support 
 *
 * (C) Copyright 2010 Barnes & Noble
 * Author: David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>
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

#include <linux/types.h>
#include <config.h>
#include <common.h>
#include <lcd.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/mux.h>

typedef enum {
    GPIO_OUTPUT = 0,
    GPIO_INPUT  = 1
} gpio_dir_t;

typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1
} gpio_level_t;

extern int gpio_pin_init(u32, gpio_dir_t, gpio_level_t);
extern int gpio_pin_write(u32, gpio_level_t);

vidinfo_t panel_info = {
    .vl_col     = 1024,
    .vl_row     = 600,
    .vl_width   = 155,
    .vl_height  = 91,

    .vl_clkp    = CFG_HIGH,
    .vl_oep     = CFG_HIGH,
    .vl_hsp     = CFG_HIGH,
    .vl_vsp     = CFG_HIGH,
    .vl_dp      = CFG_HIGH,
    .vl_bpix    = LCD_COLOR16,
    .vl_lbw     = 1,
    .vl_splt    = 0,
    .vl_clor    = 1,
    .vl_tft     = 1,

    /* Values from cmd_tft */
    .dss_panel_config = {
        .timing_h   = ((0xA0 << 20) | (0x18<<8) | (0x88)), 
        .timing_v   = ((0x1D<<20) | (0x03<<8) | (0x06)),
        .pol_freq   = 0x3000,
        .divisor    = 0x00010002,
        .lcd_size   = (599 << 16) | (1023),
        .panel_type = 0x1,
        .data_lines = 0x3,
        .load_mode  = 0x2,
    },
};

int lcd_line_length;
int lcd_color_fg = 0;
int lcd_color_bg = 0;

void *lcd_base;
void *lcd_console_address;

static int lcd_disabled;
static void *lcd_set_base;

short console_col;
short console_row;

#define FCK_DSS_ON 0x00000007 /* dss1+dss2+tv */
#define ICK_DSS_ON 0x00000001 

#define     MUX_VAL(OFFSET,VALUE) __raw_writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET));
#define     CP(x)   (CONTROL_PADCONF_##x)
     
void lcd_ctrl_init(void *lcdbase)
{
   sr32(CM_FCLKEN_DSS, 0, 32, FCK_DSS_ON);
   sr32(CM_ICLKEN_DSS, 0, 32, ICK_DSS_ON);
    
   udelay(200);
  
   omap3_dss_panel_config(&panel_info.dss_panel_config);
   omap3_dss_mem_config(&panel_info.dss_panel_config, lcdbase);
   lcd_set_base = lcdbase;
   lcd_base = lcdbase;
}

void lcd_setcolreg (ushort regno, ushort red, ushort green, ushort blue)
{
}

static inline u32 read_gpt8_reg(u32 reg)
{
    return __raw_readl(OMAP34XX_GPT8 + reg);
}

static inline void write_gpt8_reg(u32 reg, u32 val)
{
    __raw_writel(val, OMAP34XX_GPT8 + reg); 
}

void lcd_adjust_brightness(int level);


// autoreload, overflow & cmp trigger, compare enable
#define GPT8_ST		(1<<0)
#define GPT8_AR		(1<<1)
#define GPT8_CE		(1<<6)
#define GPT8_SCPWM	(1<<7)
#define GPT8_PT 	(1<<12)
#define GPT8_GPOCFG	(1<<14)
    
#define GPT8_PWM_EN (GPT8_AR | (2 << 10) | GPT8_CE | GPT8_ST | GPT8_PT)

 void enable_backlight(void) 
{
    // FIXME: Populate with correct stuff if required.
    //DECLARE_GLOBAL_DATA_PTR;
    //u32 l;
   
  
    //sr32(CM_CLKSEL_PER, 6, 1, 0x0); /* CLKSEL = 32Khz */
    //sr32(CM_FCLKEN_PER, 9, 1, 0x1); /* FCLKEN GPT8 */
    //sr32(CM_ICLKEN_PER, 9, 1, 0x1); /* ICLKEN GPT8 */
   
    //udelay(200);

    /*start with a random brightness which consumes less than 500mA such that we will gain
      current into battery even when display is showing UI image*/
    //lcd_adjust_brightness(40); 
    
   	//gpio_pin_init(GPIO_BACKLIGHT_EN_EVT2, GPIO_OUTPUT, 0);
  
}

void disable_backlight(void)
{
    // FIXME: Populate with correct stuff if required.
    //DECLARE_GLOBAL_DATA_PTR;
    //u32 l;

    //l = read_gpt8_reg(TCLR);
    //l &= ~GPT8_PWM_EN;
    //write_gpt8_reg(TLDR, 0x0);
    //write_gpt8_reg(TTGR, 0x0);
    //write_gpt8_reg(TMAR, 0x0);

    //sr32(CM_FCLKEN_PER, 9, 1, 0x0); /* FCLKEN GPT8 */
    //sr32(CM_ICLKEN_PER, 9, 1, 0x0); /* ICLKEN GPT8 */

    //gpio_pin_write( GPIO_BACKLIGHT_EN_EVT2, 1 );
}
void lcd_adjust_brightness(int level)
{
    // FIXME: Populate with correct stuff if required.
//	 u32 value=0;
  //  u32 v_tldr=0xfffffff0;
    
   /*stop timer first*/
   // write_gpt8_reg(TCLR, 0x0);
    
     /*set the match value*/
    //value =  v_tldr  + ((0xFFFFFFFE - v_tldr) * level/100);
  
  //	 write_gpt8_reg(TMAR, value);  
  	  	 
    /*set the carrier freq to based on TLDR value*/
    //write_gpt8_reg(TLDR, v_tldr);
  
  	 /*set the counter to compare */
    //write_gpt8_reg(TCRR, v_tldr);

   
    /*start the PWM*/
    //value=GPT8_PWM_EN;
    //write_gpt8_reg(TCLR, value);
	
}

// FIXME: Do I need something equivalent?
//void boxer_disable_panel(void)
//{
//	 lcd_spi_send( 0x0, 0x00);
//}
//void boxer_init_panel(void)
//{
//   lcd_spi_send( 0x0, 0x00);
//	lcd_spi_send(   0, 0xad);
//	lcd_spi_send(   1, 0x30);
//	lcd_spi_send(   2, 0x40);
//	lcd_spi_send( 0xe, 0x5f);
//	lcd_spi_send( 0xf, 0xa4);
//	lcd_spi_send( 0xd, 0x00);
//	lcd_spi_send( 0x2, 0x43);
//	lcd_spi_send( 0xa, 0x28);
//	lcd_spi_send( 0x10, 0x41);
//}

// Copied from cmd_tft.c
static void display_config(void)
{
    omap3_dss_cm_config();
//       omap3_dss_venc_config(&venc_config_std_tv); 
    omap3_dss_panel_config(&dvid_cfg);
    omap3_dss_set_background_col(DVI_BEAGLE_ORANGE_COL);
    omap3_dss_gfx_config();
    udelay(1000);
    omap3_dss_enable();
}

void lcd_enable(void)
{
    // FIXME: Populate with correct items from disp_hanstar_init
    if (lcd_disabled) {
        lcd_ctrl_init(lcd_set_base);
    }
    
    // Taken from disp_hanstar_init
    gpio_t *gpio1_base = (gpio_t *)OMAP34XX_GPIO1_BASE;
    gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
    gpio_t *gpio4_base = (gpio_t *)OMAP34XX_GPIO4_BASE;
    gpio_t *gpio5_base = (gpio_t *)OMAP34XX_GPIO5_BASE;


    /* for CL1 */
    /* <--LH_SWRD_CL1_Mervins@2011.05.17 */
    /** /SHTDN High GPIO_37 **/

    sr32((u32)&gpio2_base->oe, 5, 1, 0);
    //sr32((u32)&gpio2_base->cleardataout, 5, 1, 1);
    sr32((u32)&gpio2_base->res2[0], 5, 1, 0);

    /* GPI0_49 LCD_STB1 */
    sr32((u32)&gpio2_base->oe, 17, 1, 0);
    //sr32((u32)&gpio2_base->cleardataout, 17, 1, 1);
    sr32((u32)&gpio2_base->res2[0], 17, 1, 0);

    /* GPIO_110 LCD_EN H:enable L:disable*/
    sr32((u32)&gpio4_base->oe, 14, 1, 0);
    sr32((u32)&gpio4_base->setdataout, 14, 1, 1); /* LCD_EN H:power enable */
    udelay(80000);


            /* GPIO_48 LCD_RST1 */
    sr32((u32)&gpio2_base->oe, 16, 1, 0);
    //sr32((u32)&gpio2_base->cleardataout, 16, 1, 1);
    sr32((u32)&gpio2_base->res2[0], 16, 1, 0);
    udelay(150000);
    /* GPIO_61 LCD_POWER H:power enable L:power disable*/
    sr32((u32)&gpio2_base->oe, 29, 1, 0);
    sr32((u32)&gpio2_base->setdataout, 29, 1, 1); /* LCD_POWER H:power enable */

    udelay(100000);
    /* GPIO_159 VGH_CTRL */
    sr32((u32)&gpio5_base->oe, 31, 1, 0);
    sr32((u32)&gpio5_base->setdataout, 31, 1, 1);

    udelay(100000);
    /** /SHTDN High GPIO_37 **/
    sr32((u32)&gpio2_base->oe, 5, 1, 0);
    sr32((u32)&gpio2_base->setdataout, 5, 1, 1);
    /* LH_SWRD_CL1_Mervins@2011.05.17--> */


    udelay(500);
    display_config();

    zoom_pwm_init();
    zoom_pwm_config(220);
    zoom_pwm_enable(1);
    udelay(300000);

    /** Enable LCD_BL_EN GPIO_38 **/
    sr32((u32)&gpio2_base->oe, 6, 1, 0);
    sr32((u32)&gpio2_base->setdataout, 6, 1, 1);

    udelay(10000);
    zoom_pwm_config(180);
    udelay(10000);
    zoom_pwm_config(140);
    udelay(10000);
    zoom_pwm_config(100);
    udelay(10000);
    zoom_pwm_config(30);
    // End of code copied from disp_hanstar_init

    omap3_dss_enable();
    lcd_disabled = 0;
}


void lcd_disable(void)
{
    // FIXME: Populate with correct items from disp_hanstar_off   
    
    // Copied from disp_hanstar_off
    gpio_t *gpio1_base = (gpio_t *)OMAP34XX_GPIO1_BASE;
    gpio_t *gpio2_base = (gpio_t *)OMAP34XX_GPIO2_BASE;
    gpio_t *gpio4_base = (gpio_t *)OMAP34XX_GPIO4_BASE;
    gpio_t *gpio5_base = (gpio_t *)OMAP34XX_GPIO5_BASE;

    /* for CL1 */
    /** Disable LCD_BL_EN GPIO_38 **/
    //sr32((u32)&gpio2_base->cleardataout, 6, 1, 1);
    sr32((u32)&gpio2_base->res2[0], 6, 1, 0);
    udelay(30000);
    /* GPI0_49 LCD_STB1 */
    sr32((u32)&gpio2_base->setdataout, 16, 1, 1);
    udelay(10000);
    sr32((u32)&gpio2_base->setdataout, 17, 1, 1);

    udelay(10000);
    /** /SHTDN High GPIO_37 **/
    //sr32((u32)&gpio2_base->cleardataout, 5, 1, 1);
    sr32((u32)&gpio2_base->res2[0], 5, 1, 0);

    udelay(20000);
    /* GPIO_159 VGH_CTRL */
    sr32((u32)&gpio5_base->cleardataout, 31, 1, 1);
    udelay(40000);
    /* GPIO_61 LCD_POWER H:power enable L:power disable*/
    //sr32((u32)&gpio2_base->cleardataout, 29, 1, 1); /* LCD_POWER H:power enable */
    sr32((u32)&gpio2_base->res2[0], 29, 1, 0);
    udelay(40000);

    /* GPIO_110 LCD_EN H:enable L:disable*/
    sr32((u32)&gpio4_base->cleardataout, 14, 1, 1); /* LCD_EN H:power enable */
    udelay(40000);
    //zoom_pwm_enable(0);   
    omap3_dss_cm_config_off();
    // End of stuff copied from disp_hanstar_off
   
    lcd_disabled = 1;
}

void lcd_panel_disable(void)
{
}

ulong calc_fbsize(void)
{
    ulong size;
    int line_length = (panel_info.vl_col * NBITS (panel_info.vl_bpix)) / 8;

    size = line_length * panel_info.vl_row;
    size += PAGE_SIZE;

    return size;
}

