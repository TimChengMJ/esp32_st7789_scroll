#ifndef __ST7789_DRIVER_H__
#define __ST7789_DRIVER_H__

#include "driver/spi_master.h"

#define CONFIG_WIDTH 			240
#define CONFIG_HEIGHT 			320
#define CONFIG_OFFSETX 			0
#define CONFIG_OFFSETY 			0
#define CONFIG_MOSI_GPIO 		13
#define CONFIG_SCLK_GPIO 		14
#define CONFIG_CS_GPIO 			15
#define CONFIG_DC_GPIO 			2
#define CONFIG_RESET_GPIO 		4
#define CONFIG_BL_GPIO 			5

#define LCD_COMMAND_MODE		0
#define LCD_DATA_MODE			1

#define LCD_DIRECTION0			0
#define LCD_DIRECTION90			1
#define LCD_DIRECTION180		2
#define LCD_DIRECTION270		3

#define CONFIG_LCD_DIRECTION	LCD_DIRECTION0

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40
#define BRRED 			 0XFC07
#define GRAY  			 0X8430

int st7789_lcd_init(int width, int height);
int lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
int lcd_draw_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
int lcd_dispaly_control(uint8_t enable);
int lcd_fill_screen(uint16_t color);
int lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
int lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
int lcd_draw_rect_angle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color);
int lcd_set_scroll_area(uint16_t start, uint16_t scroll_end, uint16_t hor);
int lcd_set_scroll_start_address(uint16_t start_addr);

#endif 

