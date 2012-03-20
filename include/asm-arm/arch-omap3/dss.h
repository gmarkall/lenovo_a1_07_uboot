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

#ifndef DSS_H 
#define DSS_H

/* VENC Register address */
#define VENC_REV_ID                            0x48050C00
#define VENC_STATUS                            0x48050C04
#define VENC_F_CONTROL                         0x48050C08
#define VENC_VIDOUT_CTRL                       0x48050C10
#define VENC_SYNC_CTRL                         0x48050C14
#define VENC_LLEN                              0x48050C1C
#define VENC_FLENS                             0x48050C20
#define VENC_HFLTR_CTRL                                0x48050C24
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
#define        DSS_SYSCONFIG                           0x48050010
#define DSS_CONTROL                            0x48050040

/* DISPC register addresses */
#define DISPC_SYSCONFIG                                0x48050410
#define DISPC_SYSSTATUS                                0x48050414
#define DISPC_CONTROL                          0x48050440
#define DISPC_CONFIG                           0x48050444 
#define DISPC_DEFAULT_COLOR0                   0x4805044c 
#define DISPC_DEFAULT_COLOR1                   0x48050450 
#define DISPC_TRANS_COLOR0                     0x48050454
#define DISPC_TRANS_COLOR1                     0x48050458 
#define DISPC_TIMING_H                                 0x48050464
#define DISPC_TIMING_V                                 0x48050468
#define DISPC_POL_FREQ                                 0x4805046c
#define DISPC_DIVISOR                          0x48050470
#define DISPC_SIZE_DIG                                 0x48050478
#define DISPC_SIZE_LCD                                 0x4805047c 

/* Few Register Offsets */
#define FRAME_MODE_OFFSET                      1
#define TFTSTN_OFFSET                          3
#define DATALINES_OFFSET                       8 
 
/* Enabling Display controller */
#define LCD_ENABLE                             1
#define DIG_ENABLE                             (1 << 1)
#define GO_LCD                                 (1 << 5)
#define GO_DIG                                 (1 << 6)
#define GP_OUT0                                        (1 << 15)
#define GP_OUT1                                        (1 << 16)

#define DISPC_ENABLE                           (LCD_ENABLE | \ 
                                                DIG_ENABLE | \ 
                                                GO_LCD | \ 
                                                GO_DIG | \
                                                GP_OUT0| \
                                                GP_OUT1)
/* Configure VENC DSS Params */                                                                     
#define VENC_CLK_ENABLE                                (1 << 3)
#define DAC_DEMEN                              (1 << 4)
#define DAC_POWERDN                            (1 << 5)
#define VENC_OUT_SEL                           (1 << 6)

#define VENC_DSS_CONFIG                                (VENC_CLK_ENABLE | \ 
                                                DAC_DEMEN | \
                                                DAC_POWERDN | \
                                                VENC_OUT_SEL)

struct venc_config {
       u32 status;
       u32 f_control;
       u32 vidout_ctrl;
       u32 sync_ctrl;
       u32 llen;
       u32 flens;
       u32 hfltr_ctrl;
       u32 cc_carr_wss_carr;
       u32 c_phase;
       u32 gain_u;
       u32 gain_v;
       u32 gain_y;
       u32 black_level;
       u32 blank_level;
       u32 x_color;
       u32 m_control;
       u32 bstamp_wss_data;
       u32 s_carr;
       u32 line21; 
       u32 ln_sel; 
       u32 l21__wc_ctl;
       u32 htrigger_vtrigger;
       u32 savid__eavid;
       u32 flen__fal;
       u32 lal__phase_reset;
       u32 hs_int_start_stop_x;
       u32 hs_ext_start_stop_x;
       u32 vs_int_start_x;
       u32 vs_int_stop_x__vs_int_start_y;
       u32 vs_int_stop_y__vs_ext_start_x;
       u32 vs_ext_stop_x__vs_ext_start_y;
       u32 vs_ext_stop_y; 
       u32 avid_start_stop_x; 
       u32 avid_start_stop_y;
       u32 fid_int_start_x__fid_int_start_y;
       u32 fid_int_offset_y__fid_ext_start_x;
       u32 fid_ext_start_y__fid_ext_offset_y;
       u32 tvdetgp_int_start_stop_x;
       u32 tvdetgp_int_start_stop_y;
       u32 gen_ctrl;
       u32 output_control;
       u32 dac_b__dac_c;
       u32 height_width;
};

struct panel_config {
       u32 timing_h;
       u32 timing_v;
       u32 pol_freq;
       u32 divisor;
       u32 lcd_size;
       u32 panel_type;
       u32 data_lines;
       u32 load_mode;
};

static inline void dss_write_reg(int reg, u32 val) 
{
       __raw_writel(val, reg);
}

static inline u32 dss_read_reg(int reg) 
{
       u32 l = __raw_readl(reg); 
       return l;  
}

extern void omap3_dss_cm_config(void);
extern void omap3_dss_venc_config(const struct venc_config *venc_cfg);
extern void omap3_dss_panel_config(const struct panel_config *panel_cfg);
extern void omap3_dss_gfx_config(void);
extern void omap3_dss_enable(void);
extern void omap3_dss_set_background_col(u32 color);
                                                                                                    
#endif /* DSS_H */     