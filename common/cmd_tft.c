/*From: Syed Mohammed Khasim <kha...@ti.com>                                                           
Date: Fri, 8 Jan 2010 21:01:44 +0530                                                                 
Subject: [PATCH] Minimal Display driver for OMAP3                                                    
                                                                                                     
Supports dynamic configuration of Panel and Video Encoder                                            
Supports Background color on DVID                                                                    
Supports Color bar on S-Video                                                                        
                                                                                                     
Signed-off-by: Syed Mohammed Khasim <kha...@ti.com>                    
*/

#include <common.h> 
#include <command.h>
#include <asm/io.h>  
#include <asm/arch/dss.h> 

#include <asm/byteorder.h>

#include <asm/arch/cpu.h>
#include <asm/arch/sys_proto.h>

#include <cmd_tft.h>
int gTFT_Initialized=0;

/*                                                                                                  
 * Display Configuration                                                                            
 */                                                                                                 
#define DVI_BEAGLE_ORANGE_COL          0xFFFFFFFF//0x00FF8000

/*
 * Configure VENC in DSS for Beagle to generate Color Bar                                           
 *                                                                                                  
 * Kindly refer to OMAP TRM for definition of these values.                                         
 */
static const struct venc_config venc_config_std_tv = {
       .status                                 = 0x0000001B,
       .f_control                              = 0x00000040,
       .vidout_ctrl                            = 0x00000000, 
       .sync_ctrl                              = 0x00008000,
       .llen                                   = 0x00008359,
       .flens                                  = 0x0000020C,
       .hfltr_ctrl                             = 0x00000000,
       .cc_carr_wss_carr                       = 0x043F2631,
       .c_phase                                = 0x00000024,
       .gain_u                                 = 0x00000130,
       .gain_v                                 = 0x00000198,
       .gain_y                                 = 0x000001C0,
       .black_level                            = 0x0000006A,
       .blank_level                            = 0x0000005C,
       .x_color                                = 0x00000000,
       .m_control                              = 0x00000001,
       .bstamp_wss_data                        = 0x0000003F,
       .s_carr                                 = 0x21F07C1F,
       .line21                                 = 0x00000000,
       .ln_sel                                 = 0x00000015,
       .l21__wc_ctl                            = 0x00001400,
       .htrigger_vtrigger                      = 0x00000000, 
       .savid__eavid                           = 0x069300F4, 
       .flen__fal                              = 0x0016020C, 
       .lal__phase_reset                       = 0x00060107,
       .hs_int_start_stop_x                    = 0x008D034E,
       .hs_ext_start_stop_x                    = 0x000F0359,
       .vs_int_start_x                         = 0x01A00000,
       .vs_int_stop_x__vs_int_start_y          = 0x020501A0, 
       .vs_int_stop_y__vs_ext_start_x          = 0x01AC0024,
       .vs_ext_stop_x__vs_ext_start_y          = 0x020D01AC,
       .vs_ext_stop_y                          = 0x00000006,
       .avid_start_stop_x                      = 0x03480079,
       .avid_start_stop_y                      = 0x02040024, 
       .fid_int_start_x__fid_int_start_y       = 0x0001008A,
       .fid_int_offset_y__fid_ext_start_x      = 0x01AC0106,
       .fid_ext_start_y__fid_ext_offset_y      = 0x01060006,
       .tvdetgp_int_start_stop_x               = 0x00140001,
       .tvdetgp_int_start_stop_y               = 0x00010001, 
       .gen_ctrl                               = 0x00FF0000, 
       .output_control                         = 0x0000000D, 
       .dac_b__dac_c                           = 0x00000000,
       .height_width                           = 0x00ef027f
};

/*
 * Configure Timings for DVI D 
 */

/*
*const u32 htiming = ((0x67 << 20) | (0x09<<8) | (0x60));  //((BP << 20) | (FP<<8) | (SW));
*const u32 vtiming = ((0x0A<<20) | (0x01<<8) | (0x03));  //((BP << 20) | (FP<<8) | (SW));
*
*   if want to change Pixel clock
*   you should modify these register 
*   CM_CLKSEL2_PLL (0x4800 4D44)  -> don't change now, for some error
*   CM_CLKSEL_DSS (0x4800 4E40)
*   CM_CLKEN_PLL    (0x4800 4D00)
*   DISPC_DIVISOR  (0x4805 0470)
*/

/* <--LH_SWRD_CL1_Mervins@2011.05.17 */
static const struct panel_config dvid_cfg = { 
       .timing_h       = ((0xA0 << 20) | (0x18<<8) | (0x88)),//((0x5B << 20) | (0x09<<8) | (0x55))/*0x06700960*/, /* Horizantal timing */
       .timing_v       = ((0x1D<<20) | (0x03<<8) | (0x06))/*0x00a00103*/, /* Vertical timing */
       .pol_freq       = 0x00003000,//0x00007028, /* Pol Freq */ 
       .divisor        = 0x00010002, /* 29Mhz Pixel Clock */ 
       .lcd_size       = 0x025703ff, /* 1024x600 */
       .panel_type     = 0x01, /* TFT */ 
       .data_lines     = 0x03, /*18 Bit RGB*/     //0x03, /* 24 Bit RGB */ 
       .load_mode      = 0x02 /* Frame Mode */
};
/* LH_SWRD_CL1_Mervins@2011.05.17--> */
#define CM_FCLKEN					0x0000
#define CM_CLKSEL					0x0040
#define CM_ICLKEN					0x0010
#define PER_CM_MOD				0x800
#define GPTIMER8					0x4903E000

#define OMAP3430_CLKSEL_GPT8_MASK			(1 << 6)
#define OMAP3430_CLKSEL_GPT8_SHIFT		6
#define OMAP3430_EN_GPT8							(1 << 9)
#define OMAP3430_EN_GPT8_SHIFT				9
#define OMAP3430_CM_BASE							0x48004800

/* Read-modify-write a register in a CM module. Caller must lock */
static int cm_rmw_mod_reg_bits(u32 mask, u32 bits, s16 module, s16 idx)
{
	u32 v;

	v = __raw_readl(OMAP3430_CM_BASE + module + idx);
	v &= ~mask;
	v |= bits;
	__raw_writel(v, OMAP3430_CM_BASE+ module + idx);

	return 0;
}

static int zoom_pwm_init(void)
{
	cm_rmw_mod_reg_bits(OMAP3430_CLKSEL_GPT8_MASK, 0x1 << OMAP3430_CLKSEL_GPT8_SHIFT, PER_CM_MOD, CM_CLKSEL);
	cm_rmw_mod_reg_bits(OMAP3430_EN_GPT8, 0x1 << OMAP3430_EN_GPT8_SHIFT, PER_CM_MOD, CM_FCLKEN);
	cm_rmw_mod_reg_bits(OMAP3430_EN_GPT8, 0x1 << OMAP3430_EN_GPT8_SHIFT, PER_CM_MOD, CM_ICLKEN);
	
	__raw_writel(0x4, GPTIMER8+ 0x040);

	__raw_writel(0xFFFFFFFE, GPTIMER8+ 0x038);
	__raw_writel(0xFFFFFAEC, GPTIMER8+ 0x02C);
	__raw_writel(0x00000001, GPTIMER8+ 0x030);
	__raw_writel(0xFFFF0000, GPTIMER8+ 0x028);

	__raw_writel(0x00001847, GPTIMER8+ 0x024);
	
	return 0;		
}

static void zoom_pwm_enable(int enable)
{
	if(enable){
			__raw_writel(0x00001847, GPTIMER8+ 0x024);
	}else{
			__raw_writel(0x00001846, GPTIMER8+ 0x024);
			//cm_rmw_mod_reg_bits(OMAP3430_EN_GPT8, 0x0 << OMAP3430_EN_GPT8_SHIFT, PER_CM_MOD, CM_FCLKEN);
			//cm_rmw_mod_reg_bits(OMAP3430_EN_GPT8, 0x0 << OMAP3430_EN_GPT8_SHIFT, PER_CM_MOD, CM_ICLKEN);
	}
}

static void zoom_pwm_config(u8 brightness)
{
	u32 omap_pwm;
/*20kHz*/
	omap_pwm = 0xFFFFFAEC + (brightness*0x5);
/*128Hz*/
//	omap_pwm = 0xFFFCE68B + (brightness*793);

	__raw_writel(omap_pwm, GPTIMER8+ 0x038); 
}	

static void display_config(void)
{
	u32 data;

	omap3_dss_cm_config();
//       omap3_dss_venc_config(&venc_config_std_tv); 
  omap3_dss_panel_config(&dvid_cfg); 
  omap3_dss_set_background_col(DVI_BEAGLE_ORANGE_COL); 
	omap3_dss_gfx_config();
	udelay(1000);
	omap3_dss_enable();
}

static void disp_hanstar_init(void)
{
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

	gTFT_Initialized = 1;
}

/* <--LH_SWRD_CL1_Mervins@2011.06.03 */
static void disp_hanstar_off(void)
{
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
	gTFT_Initialized = 0;

}
/* LH_SWRD_CL1_Mervins@2011.06.03--> */

/* FIXME: Just a hack. could be wrong. */
static void lcd_disable()
{
  disp_hanstar_off();
}


static void dispc_clr_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
       u32 l = 0;
		unsigned long pwr0, pwr1, pwr2;	
		pwr0 = simple_strtoul(argv[1], NULL, 10);
		pwr1 = simple_strtoul(argv[2], NULL, 10);
		pwr2 = simple_strtoul(argv[3], NULL, 10);
	
		printf("dispc_clr_cmd\n");
       l = dss_read_reg(pwr0);
		printf("register 0x%x = 0x%x \n",pwr0, l);
       l &= (~(pwr1<<pwr2));
		printf("register 0x%x = 0x%x \n",pwr0, l);

    dss_write_reg(pwr0, l);
}

static void dispc_set_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
       u32 l = 0;
		unsigned long pwr0, pwr1, pwr2;	
		pwr0 = simple_strtoul(argv[1], NULL, 10);
		pwr1 = simple_strtoul(argv[2], NULL, 10);
		pwr2 = simple_strtoul(argv[3], NULL, 10);
	
		printf("dispc_set_cmd\n");
       l = dss_read_reg(pwr0);
		printf("register 0x%x = 0x%x \n",pwr0, l);
       l |= (pwr1<<pwr2);
		printf("register 0x%x = 0x%x \n",pwr0, l);
		
    dss_write_reg(pwr0, l);
}

void display_draw_logo_red(void)
{
	register unsigned int *pfb = (unsigned int *)LCD_FB_PHY_ADDR;
	register unsigned int i=((1024*600*2)/sizeof(unsigned int));
	do {
		*pfb++ = 0xF800F800;
	} while(--i);
	printf("display_draw_logo_red\n");
}

void display_draw_logo_green(void)
{
	register unsigned int *pfb = (unsigned int *)LCD_FB_PHY_ADDR;
	register unsigned int i=((1024*600*2)/sizeof(unsigned int));
	do {
		*pfb++ = 0x07E007E0;
	} while(--i);
	printf("display_draw_logo_green\n");
}

void display_draw_logo_blue(void)
{
	register unsigned int *pfb = (unsigned int *)LCD_FB_PHY_ADDR;
	register unsigned int i=((1024*600*2)/sizeof(unsigned int));
	do {
		*pfb++ = 0x001F001F;
	} while(--i);
	printf("display_draw_logo_blue\n");
}

void display_draw_logo_white(void)
{
	register unsigned int *pfb = (unsigned int *)LCD_FB_PHY_ADDR;
	register unsigned int i=((1024*600*2)/sizeof(unsigned int));
	do {
		*pfb++ = 0xFFFFFFFF;
	} while(--i);
	printf("display_draw_logo_white\n");
}

void display_draw_logo_black(void)
{
	register unsigned int *pfb = (unsigned int *)LCD_FB_PHY_ADDR;
	register unsigned int i=((1024*600*2)/sizeof(unsigned int));
	do {
		*pfb++ = 0x00000000;	
	} while(--i);
}
/* <--LH_SWRD_CL1_Mervins@2011.05.30 */

void display_draw_logo_area(void)
{
	run_command("logo_black", 0);
	upadte_display_area(462,145,100,310,LCD_LOGO_PHY_ADDR);
}

void disp_battery_low(void)
{
	char cmd[64];
	sprintf(cmd, "mmcinit 1;mmc 1 read %x 1640000 20000", LCD_FB_CHARGING_MESSAGE_WARNING_ADDR);	
	run_command(cmd,0);
	upadte_display_area(422,200,180,199,LCD_FB_CHARGING_MESSAGE_WARNING_ADDR);
}

U_BOOT_CMD(
	uparea,1,1,display_draw_logo_area,
	"",
	""
);

U_BOOT_CMD(
	paneloff,	1,	1,	disp_hanstar_off,
	"",
	""
);

U_BOOT_CMD(
	display_battery_low,	1,	1,	disp_battery_low,
	"",
	""
);
/* LH_SWRD_CL1_Mervins@2011.05.30--> */
void disp_panel_off(void)
{
	zoom_pwm_enable(0);	
}

void disp_panel_on(void)
{
	zoom_pwm_enable(1);	
}

U_BOOT_CMD(
	panel,	1,	1,	disp_hanstar_init,
	"",
	""
);



U_BOOT_CMD(
	logo_red,	1,	1,	display_draw_logo_red,
	"",
	""
);
U_BOOT_CMD(
	logo_green,	1,	1,	display_draw_logo_green,
	"",
	""
);

U_BOOT_CMD(
	logo_blue,	1,	1,	display_draw_logo_blue,
	"",
	""
);

U_BOOT_CMD(
	logo_white,	1,	1,	display_draw_logo_white,
	"",
	""
);

U_BOOT_CMD(
	logo_black,	1,	1,	display_draw_logo_black,
	"",
	""
);

U_BOOT_CMD(	
	dispc_clr, 6, 0,	dispc_clr_cmd,	"",);

U_BOOT_CMD(	
	dispc_set, 6, 0,	dispc_set_cmd,	"",);
	
/* ****************************************************** 
* Function:	upadte_display_area 
* Description:	epd draw area funtion on u-boot
* Prameter:	left -- x1
* 					top -- 	y1
* 					width -- w
* 					height -- h
* 					buffer -- image buffer
*
*******************************************************/
void upadte_display_area(unsigned int left, unsigned int top, unsigned int width, unsigned int height, unsigned short *buffer)
{	
	unsigned short *pImage;
	unsigned short *pfb; 
	unsigned int i, j;

	pImage = buffer;
	pfb    = (unsigned short *)(LCD_FB_PHY_ADDR+(left*2)+((top*LCD_WIDTH)*2)); 
	i=height;
	do 
	{
		j = width;
		do {
			*pfb++ = *pImage++;
		} while (--j);
		pfb += (LCD_WIDTH-width);
	} while(--i);
}
