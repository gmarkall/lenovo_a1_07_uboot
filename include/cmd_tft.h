#ifndef __CMD_TFT_H
#define __CMD_TFT_H         

#define	LCD_WIDTH	1024
#define LCD_HEIGHT	768

extern void disp_panel_on(void);
extern void disp_panel_off(void);
extern void upadte_display_area(unsigned int left, unsigned int top, unsigned int width, unsigned int height, unsigned short *buffer);
extern void display_draw_logo_red(void);
extern void display_draw_logo_green(void);
extern void display_draw_logo_blue(void);
extern void display_draw_logo_white(void);
extern void display_draw_logo_black(void);
extern int gTFT_Initialized;

#endif /* End of __CMD_TFT_H */
