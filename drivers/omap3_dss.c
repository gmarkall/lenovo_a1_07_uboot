/*                                                                                                  
 * (C) Copyright 2010                                                                               
 * Texas Instruments, <www.ti.com>                                                                  
 * Syed Mohammed Khasim <kha...@ti.com>                                                             
 *                                                                                                  
 * Referred to Linux DSS driver files for OMAP3                                                     
 *                                                                                                  
 * See file CREDITS for list of people who contributed to this                                      
 * project.                                                                                         
 *                                                                                                  
 * This program is free software; you can redistribute it and/or                                    
 * modify it under the terms of the GNU General Public License as                                   
 * published by the Free Software Foundation's version 2 of                                         
 * the License.                                                                                     
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
#include <asm/arch/dss.h> 

/*----------------------------------------*/

/* VENC Register address */
#define VENC_REV_ID                            0x48050C00
#define VENC_STATUS                            0x48050C04
#define VENC_F_CONTROL                         0x48050C08
#define VENC_VIDOUT_CTRL                       0x48050C10
#define VENC_SYNC_CTRL                         0x48050C14
#define VENC_LLEN                              0x48050C1C
#define VENC_FLENS                             0x48050C20
#define VENC_HFLTR_CTRL                        0x48050C24
#define VENC_CC_CARR_WSS_CARR                  0x48050C28
#define VENC_C_PHASE                           0x48050C2C
#define VENC_GAIN_U                            0x48050C30
#define VENC_GAIN_V                            0x48050C34
#define VENC_GAIN_Y                            0x48050C38
#define VENC_BLACK_LEVEL                       0x48050C3C
#define VENC_BLANK_LEVEL                       0x48050C40
#define VENC_X_COLOR                           0x48050C44
#define VENC_M_CONTROL                         0x48050C48
#define VENC_BSTAMP_WSS_DATA                   0x48050C4C
#define VENC_S_CARR                            0x48050C50
#define VENC_LINE21                            0x48050C54
#define VENC_LN_SEL                            0x48050C58
#define VENC_L21__WC_CTL                       0x48050C5C
#define VENC_HTRIGGER_VTRIGGER                 0x48050C60
#define VENC_SAVID__EAVID                      0x48050C64
#define VENC_FLEN__FAL                         0x48050C68
#define VENC_LAL__PHASE_RESET                  0x48050C6C
#define VENC_HS_INT_START_STOP_X               0x48050C70
#define VENC_HS_EXT_START_STOP_X               0x48050C74
#define VENC_VS_INT_START_X                    0x48050C78
#define VENC_VS_INT_STOP_X__VS_INT_START_Y     0x48050C7C
#define VENC_VS_INT_STOP_Y__VS_EXT_START_X     0x48050C80
#define VENC_VS_EXT_STOP_X__VS_EXT_START_Y     0x48050C84
#define VENC_VS_EXT_STOP_Y                     0x48050C88
#define VENC_AVID_START_STOP_X                 0x48050C90
#define VENC_AVID_START_STOP_Y                 0x48050C94
#define VENC_FID_INT_START_X__FID_INT_START_Y  0x48050CA0
#define VENC_FID_INT_OFFSET_Y__FID_EXT_START_X 0x48050CA4
#define VENC_FID_EXT_START_Y__FID_EXT_OFFSET_Y 0x48050CA8
#define VENC_TVDETGP_INT_START_STOP_X          0x48050CB0
#define VENC_TVDETGP_INT_START_STOP_Y          0x48050CB4
#define VENC_GEN_CTRL                          0x48050CB8
#define VENC_OUTPUT_CONTROL                    0x48050CC4
#define VENC_DAC_B__DAC_C                      0x48050CC8

/* DSS register addresses */
#define DSS_SYSCONFIG                          0x48050010
#define DSS_CONTROL                            0x48050040

/* DISPC register addresses */
#define DISPC_SYSCONFIG                        0x48050410
#define DISPC_SYSSTATUS                        0x48050414
#define DISPC_CONTROL                          0x48050440
#define DISPC_CONFIG                           0x48050444 
#define DISPC_DEFAULT_COLOR0                   0x4805044c 
#define DISPC_DEFAULT_COLOR1                   0x48050450 
#define DISPC_TRANS_COLOR0                     0x48050454
#define DISPC_TRANS_COLOR1                     0x48050458 
#define DISPC_TIMING_H                         0x48050464
#define DISPC_TIMING_V                         0x48050468
#define DISPC_POL_FREQ                         0x4805046c
#define DISPC_DIVISOR                          0x48050470
#define DISPC_SIZE_DIG                         0x48050478
#define DISPC_SIZE_LCD                         0x4805047c 

/* Few Register Offsets */
#define FRAME_MODE_OFFSET                      1
#define TFTSTN_OFFSET                          3
#define DATALINES_OFFSET                       8 
 
/* Enabling Display controller */
#define LCD_ENABLE                             1
#define DIG_ENABLE                             (1 << 1)
#define GO_LCD                                 (1 << 5)
#define GO_DIG                                 (1 << 6)
#define GP_OUT0                                (1 << 15)
#define GP_OUT1                                (1 << 16)

#define DISPC_ENABLE                           (LCD_ENABLE | \ 
                                                DIG_ENABLE | \ 
                                                GO_LCD | \ 
                                                GO_DIG | \
                                                GP_OUT0| \
                                                GP_OUT1)
/* Configure VENC DSS Params */                                                                     
#define VENC_CLK_ENABLE                        (1 << 3)
#define DAC_DEMEN                              (1 << 4)
#define DAC_POWERDN                            (1 << 5)
#define VENC_OUT_SEL                           (1 << 6)

#define VENC_DSS_CONFIG                         (VENC_CLK_ENABLE | \ 
                                                DAC_DEMEN | \
                                                DAC_POWERDN | \
                                                VENC_OUT_SEL)

/*---------------------------------------*/

#define OMAP3621_GPIP_OE_1 0x48310034  // GPIP_OE --> GPIO1
#define OMAP3621_GPIO_SETDATAOUT_1 0x48310094  //GPIO_SETDATAOUT --> GPIO1
#define OMAP3621_CM_CLKSEL2_PLL 0x48004D44  // CM_CLKSEL2_PLL
#define OMAP3621_CM_CLKSEL_DSS 0x48004E40  //CM_CLKSEL_DSS
#define OMAP3621_CM_CLKEN_PLL 0x48004D00  //CM_CLKEN_PLL
#define OMAP3621_VENC_REV_ID 0x48050C00  // VENC_REV_ID --> read only
#define OMAP3621_DSS_SYSCONFIG 0x48050010  // ->  DSS_SYSCONFIG  default = 0x01
#define OMAP3621_DISPC_SYSCONFIG 0x48050410 // -> DISPC_SYSCONFIG  default = 0x01
#define OMAP3621_DISPC_SYSSTATUS 0x48050414  //DISPC_SYSSTATUS --> read only	
#define OMAP3621_DISPC_TRANS_COLOR_m 0x48050454 //DISPC_TRANS_COLOR_m  default = 0x0
#define OMAP3621_DISPC_GFX_BA0 0x48050480  //DISPC_GFX_BAj -> DISPC_GFX_BA0 default = 0x0
#define OMAP3621_DISPC_GFX_BA1 0x48050484  //DISPC_GFX_BAj -> DISPC_GFX_BA1
#define OMAP3621_DISPC_GFX_POSITION 0x48050488  // DISPC_GFX_POSITION default =0x0
#define OMAP3621_DISPC_GFX_SIZE 0x4805048c  //DISPC_GFX_SIZE -> 719x1279
#define OMAP3621_DISPC_GFX_ATTRIBUTES 0x480504a0  //DISPC_GFX_ATTRIBUTES -> GFXENABLE:0x1, GFXFORMAT: RGB16, GFXREPLICATION: 0x0 disable, GFXBURSTSIZE: 0x2 16x32bit bursts
#define OMAP3621_DISPC_GFX_FIFO_THRESHOLD 0x480504a4  //DISPC_GFX_FIFO_THRESHOLD default = 0x03FF 03C0
#define OMAP3621_DISPC_GFX_FIFO_SIZE_STATUS 0x480504a8  // DISPC_GFX_FIFO_SIZE_STATUS --> read only
#define OMAP3621_DISPC_GFX_ROW_INC 0x480504ac  //DISPC_GFX_ROW_INC default= 0x01
#define OMAP3621_DISPC_GFX_PIXEL_INC 0x480504b0  //DISPC_GFX_PIXEL_INC  default = 0x01
#define OMAP3621_DISPC_GFX_WINDOW_SKIP 0x480504b4  //DISPC_GFX_WINDOW_SKIP  default = 0x00
#define OMAP3621_DISPC_GFX_TABLE_BA 0x480504b8 //DISPC_GFX_TABLE_BA  default = 0x00

/*
*  CM CLK configuration
*/
void omap3_dss_cm_config(void)
{

//		dss_write_reg(0x48310034, 0xfefffedf);
//		dss_write_reg(0x48310094, 0x01000120);

		dss_write_reg(OMAP3621_CM_CLKSEL2_PLL, 0x0001b00c);  // divide the source clock*M/N
		dss_write_reg(OMAP3621_CM_CLKSEL_DSS, 0x00001008/*0x00001006*/); // TV/16, DSS/5   TFT use 0x00001005, EPD 9" use 0x00001006
		dss_write_reg(OMAP3621_CM_CLKEN_PLL, 0x00070007/*0x00370037*/); // enable PLL3 and PLL4, power-up DSS clock

		dss_write_reg(0x48004A48, 0x0000003F); //CM_CLKSTCREL_CORE, D2D L3 L4 clock, automatic transition are enabled.
		dss_write_reg(0x48004E10, 0x00000001); //CM_ICLKEN_DSS, DSS_L3_ICLK and DSS_L4_ICLK are enabled, interface clock
		dss_write_reg(0x48004E00, 0x00000007); //CM_FCLKEN_DSS, DSS1, DSS2, TV are enabled. 

		
}

void omap3_dss_cm_config_off(void)
{

		dss_write_reg(OMAP3621_CM_CLKEN_PLL, 0x00000000/*0x00370037*/); // enable PLL3 and PLL4, power-up DSS clock

		dss_write_reg(0x48004A48, 0x00000000); //CM_CLKSTCREL_CORE, D2D L3 L4 clock, automatic transition are enabled.
		dss_write_reg(0x48004E10, 0x00000000); //CM_ICLKEN_DSS, DSS_L3_ICLK and DSS_L4_ICLK are enabled, interface clock
		dss_write_reg(0x48004E00, 0x00000000); //CM_FCLKEN_DSS, DSS1, DSS2, TV are enabled. 

		
}

/*                                                                                                  
 * VENC configuration                                                                               
 */
void omap3_dss_venc_config(const struct venc_config *venc_cfg)
{
       dss_write_reg(VENC_F_CONTROL, venc_cfg->f_control);
       dss_write_reg(VENC_VIDOUT_CTRL, venc_cfg->vidout_ctrl);
       dss_write_reg(VENC_SYNC_CTRL, venc_cfg->sync_ctrl); 
       dss_write_reg(VENC_LLEN, venc_cfg->llen); 
       dss_write_reg(VENC_FLENS, venc_cfg->flens); 
       dss_write_reg(VENC_HFLTR_CTRL, venc_cfg->hfltr_ctrl); 
       dss_write_reg(VENC_CC_CARR_WSS_CARR, venc_cfg->cc_carr_wss_carr);
       dss_write_reg(VENC_C_PHASE, venc_cfg->c_phase);
       dss_write_reg(VENC_GAIN_U, venc_cfg->gain_u);
       dss_write_reg(VENC_GAIN_V, venc_cfg->gain_v);
       dss_write_reg(VENC_GAIN_Y, venc_cfg->gain_y); 
       dss_write_reg(VENC_BLACK_LEVEL, venc_cfg->black_level);
       dss_write_reg(VENC_BLANK_LEVEL, venc_cfg->blank_level); 
       dss_write_reg(VENC_X_COLOR, venc_cfg->x_color); 
       dss_write_reg(VENC_M_CONTROL, venc_cfg->m_control); 
       dss_write_reg(VENC_BSTAMP_WSS_DATA, venc_cfg->bstamp_wss_data); 
       dss_write_reg(VENC_S_CARR, venc_cfg->s_carr); 
       dss_write_reg(VENC_LINE21, venc_cfg->line21);
       dss_write_reg(VENC_LN_SEL, venc_cfg->ln_sel);
       dss_write_reg(VENC_L21__WC_CTL, venc_cfg->l21__wc_ctl);
       dss_write_reg(VENC_HTRIGGER_VTRIGGER, venc_cfg->htrigger_vtrigger);
       dss_write_reg(VENC_SAVID__EAVID, venc_cfg->savid__eavid);
       dss_write_reg(VENC_FLEN__FAL, venc_cfg->flen__fal);
       dss_write_reg(VENC_LAL__PHASE_RESET, venc_cfg->lal__phase_reset);
       dss_write_reg(VENC_HS_INT_START_STOP_X, 
                               venc_cfg->hs_int_start_stop_x);
       dss_write_reg(VENC_HS_EXT_START_STOP_X,
                               venc_cfg->hs_ext_start_stop_x);
       dss_write_reg(VENC_VS_INT_START_X, venc_cfg->vs_int_start_x); 
       dss_write_reg(VENC_VS_INT_STOP_X__VS_INT_START_Y, 
                       venc_cfg->vs_int_stop_x__vs_int_start_y);
       dss_write_reg(VENC_VS_INT_STOP_Y__VS_EXT_START_X, 
                       venc_cfg->vs_int_stop_y__vs_ext_start_x);
       dss_write_reg(VENC_VS_EXT_STOP_X__VS_EXT_START_Y,
                       venc_cfg->vs_ext_stop_x__vs_ext_start_y);
       dss_write_reg(VENC_VS_EXT_STOP_Y, venc_cfg->vs_ext_stop_y);
       dss_write_reg(VENC_AVID_START_STOP_X, venc_cfg->avid_start_stop_x);
       dss_write_reg(VENC_AVID_START_STOP_Y, venc_cfg->avid_start_stop_y);
       dss_write_reg(VENC_FID_INT_START_X__FID_INT_START_Y,
                               venc_cfg->fid_int_start_x__fid_int_start_y);
       dss_write_reg(VENC_FID_INT_OFFSET_Y__FID_EXT_START_X,
                               venc_cfg->fid_int_offset_y__fid_ext_start_x);
       dss_write_reg(VENC_FID_EXT_START_Y__FID_EXT_OFFSET_Y,
                               venc_cfg->fid_ext_start_y__fid_ext_offset_y);
       dss_write_reg(VENC_TVDETGP_INT_START_STOP_X,
                               venc_cfg->tvdetgp_int_start_stop_x);
       dss_write_reg(VENC_TVDETGP_INT_START_STOP_Y,
                               venc_cfg->tvdetgp_int_start_stop_y);
       dss_write_reg(VENC_GEN_CTRL, venc_cfg->gen_ctrl);
       dss_write_reg(VENC_OUTPUT_CONTROL, venc_cfg->output_control);
       dss_write_reg(VENC_DAC_B__DAC_C, venc_cfg->dac_b__dac_c);

       dss_write_reg(DISPC_SIZE_DIG, venc_cfg->height_width);
       dss_write_reg(DSS_CONTROL, VENC_DSS_CONFIG);
		dss_write_reg(OMAP3621_DISPC_SYSCONFIG, 0x00002015); // -> DISPC_SYSCONFIG  default = 0x01			 
}

/*                                                                                                  
 * Configure Panel Specific parameters                                                              
 */
void omap3_dss_panel_config(const struct panel_config *panel_cfg)
{
       dss_write_reg(DISPC_TIMING_H, panel_cfg->timing_h);
       dss_write_reg(DISPC_TIMING_V, panel_cfg->timing_v);
       dss_write_reg(DISPC_POL_FREQ, panel_cfg->pol_freq);
       dss_write_reg(DISPC_DIVISOR, panel_cfg->divisor); 
       dss_write_reg(DISPC_SIZE_LCD, panel_cfg->lcd_size);
       dss_write_reg(DISPC_CONFIG,
               (panel_cfg->load_mode << FRAME_MODE_OFFSET));
       dss_write_reg(DISPC_CONTROL, 
               ((panel_cfg->panel_type << TFTSTN_OFFSET) | 
               (panel_cfg->data_lines << DATALINES_OFFSET)));

       dss_write_reg(DSS_CONTROL, VENC_DSS_CONFIG);
	dss_write_reg(OMAP3621_DISPC_SYSCONFIG, 0x00002015); // -> DISPC_SYSCONFIG  default = 0x01			 

}

void omap3_dss_gfx_config(void)
{
       dss_write_reg(OMAP3621_DISPC_GFX_BA0, LCD_FB_PHY_ADDR /* 0x805EC000 */);   //DISPC_GFX_BAj -> DISPC_GFX_BA0 default = 0x0
       dss_write_reg(OMAP3621_DISPC_GFX_BA1, LCD_FB_PHY_ADDR /* 0x805EC000 */);   //DISPC_GFX_BAj -> DISPC_GFX_BA1
       dss_write_reg(OMAP3621_DISPC_GFX_POSITION, 0x00000000);   // DISPC_GFX_POSITION default =0x0
       dss_write_reg(OMAP3621_DISPC_GFX_SIZE, 0x025703ff);   //DISPC_GFX_SIZE -> 599x1023   /* <--LH_SWRD_CL1_Mervins@2011.05.17--> */
//       dss_write_reg(OMAP3621_DISPC_GFX_SIZE, 0x0338012B);   //DISPC_GFX_SIZE -> 825x1200   EPD 9"
       dss_write_reg(OMAP3621_DISPC_GFX_ATTRIBUTES, 0x0000008d);   //DISPC_GFX_ATTRIBUTES -> GFXENABLE:0x1, GFXFORMAT: RGB16, GFXREPLICATION: 0x0 disable, GFXBURSTSIZE: 0x2 16x32bit bursts
       dss_write_reg(OMAP3621_DISPC_GFX_FIFO_THRESHOLD, 0x03fc03bc);   //DISPC_GFX_FIFO_THRESHOLD default = 0x03FF 03C0
       dss_write_reg(OMAP3621_DISPC_GFX_FIFO_SIZE_STATUS, 0x00000400);   // DISPC_GFX_FIFO_SIZE_STATUS --> read only
       dss_write_reg(OMAP3621_DISPC_GFX_ROW_INC, 0x00000001);   //DISPC_GFX_ROW_INC default= 0x01
       dss_write_reg(OMAP3621_DISPC_GFX_PIXEL_INC, 0x00000001);   //DISPC_GFX_PIXEL_INC  default = 0x01
       dss_write_reg(OMAP3621_DISPC_GFX_WINDOW_SKIP, 0x00000000);   //DISPC_GFX_WINDOW_SKIP  default = 0x00
       dss_write_reg(OMAP3621_DISPC_GFX_TABLE_BA, 0x807ff000);  //DISPC_GFX_TABLE_BA  default = 0x00
	
}

/* 
 * Enable LCD and DIGITAL OUT in DSS                                                                
 */
void omap3_dss_enable(void)
{
       u32 l = 0;

       l = dss_read_reg(DISPC_CONTROL);
       l |= DISPC_ENABLE;

       dss_write_reg(DISPC_CONTROL, l);
}

/*
 * Set Background Color in DISPC                                                                    
 */
void omap3_dss_set_background_col(u32 color)
{

       dss_write_reg(DISPC_DEFAULT_COLOR0, color);

		//for epd 9"
		dss_write_reg(0x48050454, color);
}


