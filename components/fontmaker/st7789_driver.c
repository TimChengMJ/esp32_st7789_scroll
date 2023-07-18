#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "st7789_driver.h"

#ifndef BLOG_INFO
#define BLOG_TAG					"ST7789"
#define BLOG_INFO(fmt, ...)			printf("[%s][I]"fmt"\r\n", BLOG_TAG, ##__VA_ARGS__)
#define BLOG_ERR(fmt, ...)			printf("[%s][I]"fmt"\r\n", BLOG_TAG, ##__VA_ARGS__)
#endif

#define HOST_ID SPI2_HOST

// static const int SPI_Frequency = SPI_MASTER_FREQ_20M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_26M;
// static const int SPI_Frequency = SPI_MASTER_FREQ_40M;
static const int SPI_Frequency = SPI_MASTER_FREQ_80M;
static spi_device_handle_t lcd_7789_handle;

void lcd_delay_ms(int ms)
{
	vTaskDelay(ms);
}

static int spi_master_init(int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS, int16_t GPIO_DC, int16_t GPIO_RESET, int16_t GPIO_BL)
{
	esp_err_t ret;

	BLOG_INFO("GPIO_CS=%d",GPIO_CS);
	if ( GPIO_CS >= 0 ) {
		//gpio_pad_select_gpio( GPIO_CS );
		gpio_reset_pin( GPIO_CS );
		gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_CS, 0 );
	}

	BLOG_INFO("GPIO_DC=%d",GPIO_DC);
	//gpio_pad_select_gpio( GPIO_DC );
	gpio_reset_pin( GPIO_DC );
	gpio_set_direction( GPIO_DC, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_DC, 0 );

	BLOG_INFO("GPIO_RESET=%d",GPIO_RESET);
	if ( GPIO_RESET >= 0 ) {
		//gpio_pad_select_gpio( GPIO_RESET );
		gpio_reset_pin( GPIO_RESET );
		gpio_set_direction( GPIO_RESET, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_RESET, 1 );
		lcd_delay_ms(100);
		gpio_set_level( GPIO_RESET, 0 );
		lcd_delay_ms(100);
		gpio_set_level( GPIO_RESET, 1 );
		lcd_delay_ms(100);
	}

	BLOG_INFO("GPIO_BL=%d",GPIO_BL);
	if ( GPIO_BL >= 0 ) {
		//gpio_pad_select_gpio(GPIO_BL);
		gpio_reset_pin(GPIO_BL);
		gpio_set_direction( GPIO_BL, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_BL, 0 );
	}

	BLOG_INFO("GPIO_MOSI=%d",GPIO_MOSI);
	BLOG_INFO("GPIO_SCLK=%d",GPIO_SCLK);
	spi_bus_config_t buscfg = {
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = -1,
		.sclk_io_num = GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};

	ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO );
	BLOG_INFO("spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));
	devcfg.clock_speed_hz = SPI_Frequency;
	devcfg.queue_size = 7;
	devcfg.mode = 2;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	if ( GPIO_CS >= 0 ) {
		devcfg.spics_io_num = GPIO_CS;
	} else {
		devcfg.spics_io_num = -1;
	}
	
	ret = spi_bus_add_device( HOST_ID, &devcfg, &lcd_7789_handle);
	BLOG_INFO("spi_bus_add_device=%d",ret);

	return ret;
}


int spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t* Data, size_t DataLength)
{
	spi_transaction_t SPITransaction;
	esp_err_t ret = -1;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
		ret = spi_device_transmit( SPIHandle, &SPITransaction );
	}

	return ret;
}

int spi_master_write_command(uint8_t cmd)
{
	static uint8_t Byte = 0;
	Byte = cmd;
	gpio_set_level(CONFIG_DC_GPIO, LCD_COMMAND_MODE);
	return spi_master_write_byte(lcd_7789_handle, &Byte, 1);
}

int spi_master_write_data_byte(uint8_t data)
{
	static uint8_t Byte = 0;
	Byte = data;
	gpio_set_level(CONFIG_DC_GPIO, LCD_DATA_MODE);
	return spi_master_write_byte(lcd_7789_handle, &Byte, 1);
}


int spi_master_write_data_word(uint16_t data)
{
	static uint8_t Byte[2];
	Byte[0] = (data >> 8) & 0xFF;
	Byte[1] = data & 0xFF;
	gpio_set_level(CONFIG_DC_GPIO, LCD_DATA_MODE);
	return spi_master_write_byte(lcd_7789_handle, Byte, 2);
}

int spi_master_write_addr(uint16_t addr1, uint16_t addr2)
{
	static uint8_t Byte[4];
	Byte[0] = (addr1 >> 8) & 0xFF;
	Byte[1] = addr1 & 0xFF;
	Byte[2] = (addr2 >> 8) & 0xFF;
	Byte[3] = addr2 & 0xFF;
	gpio_set_level(CONFIG_DC_GPIO, LCD_DATA_MODE );
	return spi_master_write_byte(lcd_7789_handle, Byte, 4);
}

int spi_master_write_color(uint16_t color, uint16_t size)
{
	int index = 0, ret = 0, i = 0;
	uint8_t *p_buffer = NULL;

	p_buffer = malloc(size*2);
	if(!p_buffer) return -1;

	for(i=0;i<size;i++) {
		p_buffer[index++] = (color >> 8) & 0xFF;
		p_buffer[index++] = color & 0xFF;
	}
	gpio_set_level(CONFIG_DC_GPIO, LCD_DATA_MODE );

	ret = spi_master_write_byte(lcd_7789_handle, p_buffer, size*2);

	free(p_buffer);

	return ret;
}

int st7789_lcd_init(int width, int height)
{
	int ret = 0;

	ret = spi_master_init(CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
	if(ret) {
		BLOG_ERR("Wrong SPI Master Init:%d", ret);
		return ret;
	}
	ret += spi_master_write_command(0x01);	//Software Reset
	lcd_delay_ms(150);
	ret += spi_master_write_command(0x11);	//Sleep Out
	lcd_delay_ms(255);
	ret += spi_master_write_command(0x3A);	//Interface Pixel Format
	spi_master_write_data_byte(0x55);
	lcd_delay_ms(10);
	ret += spi_master_write_command(0x36);	//Memory Data Access Control
	spi_master_write_data_byte(0x00);
	// spi_master_write_command(0x2A);	//Column Address Set
	// spi_master_write_data_byte(0x00);
	// spi_master_write_data_byte(0x00);
	// spi_master_write_data_word(240);
	// spi_master_write_command(0x2B);	//Row Address Set
	// spi_master_write_data_byte(0x00);
	// spi_master_write_data_byte(0x00);
	// spi_master_write_data_word(320);
	ret += spi_master_write_command(0x20);	//Display Inversion On
	lcd_delay_ms(10);
	ret += spi_master_write_command(0x13);	//Normal Display Mode On
	lcd_delay_ms(10);
	ret += spi_master_write_command(0x29);	//Display ON
	lcd_delay_ms(10);

	gpio_set_level(CONFIG_BL_GPIO, 1);

	BLOG_INFO("st7789 lcd init ret:%d", ret);
	return ret;
}


// Draw pixel
// x:X coordinate
// y:Y coordinate
// color:color
int lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
	int ret = 0;

	ret += spi_master_write_command(0x2A);	// set column(x) address
	ret += spi_master_write_addr(x, x);
	ret += spi_master_write_command(0x2B);	// set Page(y) address
	ret += spi_master_write_addr(y, y);
	ret += spi_master_write_command(0x2C);	//	Memory Write
	ret += spi_master_write_data_word(color);

	return ret;
}

// Draw rectangle of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
int lcd_draw_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) 
{
	uint16_t _x1 = x1;
	uint16_t _x2 = x2;
	uint16_t _y1 = y1;
	uint16_t _y2 = y2;
	int ret = 0;

	if (x1 >= CONFIG_WIDTH) return -1;
	if (x2 >= CONFIG_WIDTH) _x2=CONFIG_WIDTH-1;
	if (y1 >= CONFIG_HEIGHT) return -1;
	if (y2 >= CONFIG_HEIGHT) _y2=CONFIG_HEIGHT-1;

	ret += spi_master_write_command(0x2A);	// set column(x) address
	ret += spi_master_write_addr(_x1, _x2);
	ret += spi_master_write_command(0x2B);	// set Page(y) address
	ret += spi_master_write_addr(_y1, _y2);
	ret += spi_master_write_command(0x2C);	//	Memory Write
	for(int i=_x1;i<=_x2;i++){
		uint16_t size = _y2-_y1+1;
		ret += spi_master_write_color(color, size);
	}
	return ret;
}

int lcd_dispaly_control(uint8_t enable) 
{
	int ret = 0;
	if(enable) {
		ret = spi_master_write_command(0x29);	//Display on
	} else {
		ret = spi_master_write_command(0x28);	//Display off
	}

	return ret;
}

// Fill screen
// color:color
int lcd_fill_screen(uint16_t color)
{
	return lcd_draw_fill_rect(0, 0, CONFIG_WIDTH-1, CONFIG_HEIGHT-1, color);
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color 
int lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) 
{
	int i,dx,dy,sx,sy,E;
	int ret = 0;

	/* distance between two points */
	dx = ( x2 > x1 ) ? x2 - x1 : x1 - x2;
	dy = ( y2 > y1 ) ? y2 - y1 : y1 - y2;

	/* direction of two point */
	sx = ( x2 > x1 ) ? 1 : -1;
	sy = ( y2 > y1 ) ? 1 : -1;

	/* inclination < 1 */
	if ( dx > dy ) {
		E = -dx;
		for ( i = 0 ; i <= dx ; i++ ) {
			ret += lcd_draw_pixel(x1, y1, color);
			x1 += sx;
			E += 2 * dy;
			if ( E >= 0 ) {
				y1 += sy;
				E -= 2 * dx;
			}
		}
	} else {		/* inclination >= 1 */
		E = -dy;
		for ( i = 0 ; i <= dy ; i++ ) {
			ret += lcd_draw_pixel(x1, y1, color);
			y1 += sy;
			E += 2 * dx;
			if ( E >= 0 ) {
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}

	return ret;
}

// Draw rectangle
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color
int lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) 
{
	int ret = 0;

	ret += lcd_draw_line(x1, y1, x2, y1, color);
	ret += lcd_draw_line(x2, y1, x2, y2, color);
	ret += lcd_draw_line(x2, y2, x1, y2, color);
	ret += lcd_draw_line(x1, y2, x1, y1, color);

	return ret;
}

// Draw rectangle with angle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of rectangle
// h:Height of rectangle
// angle:Angle of rectangle
// color:color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
int lcd_draw_rect_angle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) 
{
	double xd,yd,rd;
	int x1,y1;
	int x2,y2;
	int x3,y3;
	int x4,y4;
	int ret = 0;

	rd = -angle * M_PI / 180.0;
	xd = 0.0 - w/2;
	yd = h/2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w/2;
	yd = h/2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x4 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y4 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	ret += lcd_draw_line(x1, y1, x2, y2, color);
	ret += lcd_draw_line(x1, y1, x3, y3, color);
	ret += lcd_draw_line(x2, y2, x4, y4, color);
	ret += lcd_draw_line(x3, y3, x4, y4, color);

	return ret;
}

/**
 * @brief Vertical Scrolling Definition.
 * @param   tfa    top fixed area
 * @param   vsa    scroll area
 * @param   bta    bottom fixed area
 * @return  errcode
 * @retval  0      success
 * @retval  -1     fail 
 */
int lcd_set_scroll_area(uint16_t start, uint16_t scroll_end, uint16_t hor)
{
    uint8_t data[6];
    
    if (start + scroll_end + hor != 320) {
		printf("*******error scroll area set*******\r\n");
        return -1;
    }
    
    spi_master_write_command(0x33);
    
    data[0] = start >> 8;
    data[1] = start;
    data[2] = scroll_end >> 8;
    data[3] = scroll_end;
    data[4] = hor >> 8;
    data[5] = hor;

    gpio_set_level(CONFIG_DC_GPIO, LCD_DATA_MODE);
    
	return spi_master_write_byte(lcd_7789_handle, data, sizeof(data));
}

/**
 * @brief Set Vertical scroll start address of RAM.
 * @param   vsp    scroll start address of RAM
 * @return  none
 */
int lcd_set_scroll_start_address(uint16_t start_addr)
{
    uint8_t data[2] = {0};
	static uint8_t Byte[2];

    spi_master_write_command(0x37);
    
    data[0] = start_addr >> 8;
    data[1] = (uint8_t)start_addr;

	gpio_set_level(CONFIG_DC_GPIO, LCD_DATA_MODE);
	return spi_master_write_byte(lcd_7789_handle, data, 2);
}

