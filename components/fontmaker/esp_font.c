#include "esp_font.h"											    
#include "st7789_driver.h"

extern unsigned char asc2_1206[95][12];
extern unsigned char asc2_1608[95][16];
extern unsigned char asc2_2412[95][36];

//将utf-8编码转为unicode编码（函数来自LVGL）
static uint32_t lv_txt_utf8_next(const char * txt, uint32_t * i)
{
    /* Unicode to UTF-8
     * 00000000 00000000 00000000 0xxxxxxx -> 0xxxxxxx
     * 00000000 00000000 00000yyy yyxxxxxx -> 110yyyyy 10xxxxxx
     * 00000000 00000000 zzzzyyyy yyxxxxxx -> 1110zzzz 10yyyyyy 10xxxxxx
     * 00000000 000wwwzz zzzzyyyy yyxxxxxx -> 11110www 10zzzzzz 10yyyyyy 10xxxxxx
     * */

    uint32_t result = 0;
//printf("txt[%d]=%X %X %X\n",*i,txt[*i],txt[*i+1],txt[*i+2]);
    /*Dummy 'i' pointer is required*/
    uint32_t i_tmp = 0;
    if(i == NULL) i = &i_tmp;

    /*Normal ASCII*/
    if((txt[*i] & 0x80) == 0) {
        result = txt[*i];
        (*i)++;
    }
    /*Real UTF-8 decode*/
    else {
        /*2 bytes UTF-8 code*/
        if((txt[*i] & 0xE0) == 0xC0) {
            result = (uint32_t)(txt[*i] & 0x1F) << 6;
            (*i)++;
            if((txt[*i] & 0xC0) != 0x80) return 0; /*Invalid UTF-8 code*/
            result += (txt[*i] & 0x3F);
            (*i)++;
        }
        /*3 bytes UTF-8 code*/
        else if((txt[*i] & 0xF0) == 0xE0) {
            result = (uint32_t)(txt[*i] & 0x0F) << 12;
            (*i)++;

            if((txt[*i] & 0xC0) != 0x80) return 0; /*Invalid UTF-8 code*/
            result += (uint32_t)(txt[*i] & 0x3F) << 6;
            (*i)++;

            if((txt[*i] & 0xC0) != 0x80) return 0; /*Invalid UTF-8 code*/
            result += (txt[*i] & 0x3F);
            (*i)++;
        }
        /*4 bytes UTF-8 code*/
        else if((txt[*i] & 0xF8) == 0xF0) {
            result = (uint32_t)(txt[*i] & 0x07) << 18;
            (*i)++;

            if((txt[*i] & 0xC0) != 0x80) return 0; /*Invalid UTF-8 code*/
            result += (uint32_t)(txt[*i] & 0x3F) << 12;
            (*i)++;

            if((txt[*i] & 0xC0) != 0x80) return 0; /*Invalid UTF-8 code*/
            result += (uint32_t)(txt[*i] & 0x3F) << 6;
            (*i)++;

            if((txt[*i] & 0xC0) != 0x80) return 0; /*Invalid UTF-8 code*/
            result += txt[*i] & 0x3F;
            (*i)++;
        }
        else {
            (*i)++; /*Not UTF-8 char. Go the next.*/
        }
    }
    return result;
}

/** Searches base[0] to base[n - 1] for an item that matches *key.
 *
 * @note The function cmp must return negative if its first
 *  argument (the search key) is less that its second (a table entry),
 *  zero if equal, and positive if greater.
 *
 *  @note Items in the array must be in ascending order.
 *
 * @param key    Pointer to item being searched for
 * @param base   Pointer to first element to search
 * @param n      Number of elements
 * @param size   Size of each element
 * @param cmp    Pointer to comparison function (see #lv_font_codeCompare as a comparison function
 * example)
 *
 * @return a pointer to a matching item, or NULL if none exists.
 */
void * _lv_utils_bsearch(const void * key, const void * base, uint32_t n, uint32_t size,
                         int32_t (*cmp)(const void * pRef, const void * pElement))
{
    const char * middle;
    int32_t c;

    for(middle = base; n != 0;) {
        middle += (n / 2) * size;
        if((c = (*cmp)(key, middle)) > 0) {
            n    = (n / 2) - ((n & 1) == 0);
            base = (middle += size);
        }
        else if(c < 0) {
            n /= 2;
            middle = base;
        }
        else {
            return (char *)middle;
        }
    }
    return NULL;
}

/** Code Comparator.
 *
 *  Compares the value of both input arguments.
 *
 *  @param[in]  pRef        Pointer to the reference.
 *  @param[in]  pElement    Pointer to the element to compare.
 *
 *  @return Result of comparison.
 *  @retval < 0   Reference is greater than element.
 *  @retval = 0   Reference is equal to element.
 *  @retval > 0   Reference is less than element.
 *
 */
static int32_t unicode_list_compare(const void * ref, const void * element)
{
    return ((int32_t)(*(uint16_t *)ref)) - ((int32_t)(*(uint16_t *)element));
}

static uint32_t get_glyph_dsc_id(const lv_font_t * font, uint32_t letter)
{
	if(letter == '\0') return 0;

    lv_font_fmt_txt_dsc_t * fdsc = (lv_font_fmt_txt_dsc_t *) font->dsc;

    /*Check the cache first*/
    if(letter == fdsc->last_letter) return fdsc->last_glyph_id;

	uint32_t glyph_id = 0;
	if(fdsc->cmap_num == 1) 
	{

        /*Relative code point*/
        uint32_t rcp = letter - fdsc->cmaps[0].range_start;
        if(rcp > fdsc->cmaps[0].range_length) 
		{
			fdsc->last_letter = letter;
			fdsc->last_glyph_id = 0;
			return 0;
		}
        if(fdsc->cmaps[0].type == LV_FONT_FMT_TXT_CMAP_SPARSE_TINY) {
            uint8_t * p = _lv_utils_bsearch(&rcp, fdsc->cmaps[0].unicode_list, fdsc->cmaps[0].list_length,
                                            sizeof(fdsc->cmaps[0].unicode_list[0]), unicode_list_compare);

            if(p) {
                lv_uintptr_t ofs = (lv_uintptr_t)(p - (uint8_t *) fdsc->cmaps[0].unicode_list);
                ofs = ofs >> 1;     /*The list stores `uint16_t` so the get the index divide by 2*/
                glyph_id = fdsc->cmaps[0].glyph_id_start + ofs;
            }
        }
	}
	
	/*Update the cache*/
	fdsc->last_letter = letter;
	fdsc->last_glyph_id = glyph_id;
	return glyph_id;
}

/***********************************************************************************
 * 顶部到字模数据的距离 = line_height - box_h - baseline - ofs_y
 * 左侧到字模数据的距离 = ofs_x
 * 左侧到下一个字左侧的距离 = 实际字符宽度 = (adv_w + 8)/16
 * 
 * 字体颜色 = 字模数据 * (前景色 - 背景色)/(0xff>>(8-bpp)) + 背景色 
 *
 * ********************************************************************************/
void show_font_pixel(const lv_font_t* font, uint16_t* x,uint16_t* y,uint32_t unicode, uint16_t font_color, uint16_t backg_color)
{
	if(unicode == '\t') unicode = ' ';

	uint16_t x0 = *x;
	uint16_t y0 = *y;

	lv_font_fmt_txt_dsc_t * fdsc = (lv_font_fmt_txt_dsc_t *) font->dsc;
	uint32_t gid = get_glyph_dsc_id(font,unicode);
	if(!gid) return;
	const lv_font_fmt_txt_glyph_dsc_t * gdsc = &fdsc->glyph_dsc[gid];

	uint32_t gsize = gdsc->box_w * gdsc->box_h;
	if(gsize == 0) return;

	int16_t g_ofs_y = font->line_height - gdsc->box_h - font->base_line - gdsc->ofs_y;
	int16_t g_ofs_x = gdsc->ofs_x;

	const uint8_t* bitmap = fdsc->glyph_bitmap;
	uint16_t index = gdsc->bitmap_index;
	uint8_t temp  = bitmap[index];
	uint8_t pixcount = 0;
	uint8_t pixperbyte = 8/fdsc->bpp;
    uint16_t colorR,colorG,colorB;
    
     if((x0 + ((gdsc->adv_w + 8)>>4)) > CONFIG_WIDTH)//换行
    {	    
        *y+=font->line_height;
        y0=*y;
        *x=0;
        x0=*x;
    }

    lcd_draw_fill_rect(x0, y0, x0+((gdsc->adv_w + 8)>>4)-1, y0+font->line_height-1, backg_color);

	for(*y=y0; *y < (y0 + font->line_height); (*y)++)
	{
		for(*x=x0; *x < (x0 + ((gdsc->adv_w + 8)>>4)); (*x)++)
		{            
			if(*y>=(y0+g_ofs_y) && *y<(y0 + g_ofs_y+gdsc->box_h) && *x>=(x0 + g_ofs_x) && *x<(x0 + g_ofs_x + gdsc->box_w)) {
                if(temp&0x80) lcd_draw_pixel(*x,*y, font_color);

				temp = temp<<fdsc->bpp;
				pixcount++;
				if(pixcount >= pixperbyte) {
					temp = bitmap[++index];
					pixcount = 0;
				}
			} else {
                // lcd_draw_pixel(*x,*y, WHITE);
			}
		}
	}
    
    *y = y0;
}

void lcd_show_zh_string(const lv_font_t* font, uint16_t x,uint16_t y, char* txt, uint16_t font_color, uint16_t background_color)
{
    uint32_t i = 0;	
	uint32_t unicode;
    
	do{
		unicode = lv_txt_utf8_next(txt,&i);
		show_font_pixel(font, &x,&y,unicode, font_color, background_color);
	}while(unicode);
}

//在指定位置显示一个字符
//x,y:起始坐标
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16/24
//mode:叠加方式(1)还是非叠加方式(0)
void lcd_show_char(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode,uint16_t font_color, uint16_t background_color)
{  							  
    uint8_t temp,t1,t;
	uint16_t y0=y;
	uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数	
 	num=num-' ';//得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）

    lcd_draw_fill_rect(x, y, x+(size/2)-1, y+size-1, background_color);

	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//调用1206字体
		else if(size==16)temp=asc2_1608[num][t];	//调用1608字体
		else if(size==24)temp=asc2_2412[num][t];	//调用2412字体
		else return;								//没有的字库

		for(t1=0;t1<8;t1++) {			    
			if(temp&0x80) {
                lcd_draw_pixel(x,y,font_color);
            } else if(mode==0) {
                // lcd_draw_pixel(x,y,background_color);
            }
			temp<<=1;
			y++;
			if(y>CONFIG_HEIGHT) return;		//超区域了
			if((y-y0)==size) {
				y=y0;
				x++;
				if(x>=CONFIG_WIDTH) return;	//超区域了
				break;
			}
		}  	 
	}  	    	   	 	  
}   

//显示字符串
//x,y:起点坐标
//width,height:区域大小  
//size:字体大小
//*p:字符串起始地址		  
void lcd_show_ascii_string(uint16_t x, uint16_t y, uint8_t size,uint8_t *p, uint16_t font_color, uint16_t background_color)
{         
    while((*p<='~')&&(*p>=' ')) {           //判断是不是非法字符!
        lcd_show_char(x, y, *p, size, 0, font_color, background_color);
        x+=size/2;
        p++;
    }  
}
























		  






