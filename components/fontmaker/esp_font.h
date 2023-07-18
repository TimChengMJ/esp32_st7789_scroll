
#pragma once

#include <stdio.h>
#include <stdbool.h>

/****************************************** MCU_Font***************************************/

#define LV_ATTRIBUTE_LARGE_CONST
#define lv_font_get_glyph_dsc_fmt_txt NULL
#define lv_font_get_bitmap_fmt_txt NULL
#define LV_FONT_SUBPX_NONE 0

typedef int16_t lv_coord_t;

typedef uint32_t lv_uintptr_t;

//lv_font_fmt_txt.h
/** This describes a glyph. */
typedef struct {
    uint32_t bitmap_index : 20;     /**< Start index of the bitmap. A font can be max 1 MB. */
    uint32_t adv_w : 12;            /**< Draw the next glyph after this width. 8.4 format (real_value * 16 is stored). */
    uint8_t box_w;                  /**< Width of the glyph's bounding box*/
    uint8_t box_h;                  /**< Height of the glyph's bounding box*/
    int8_t ofs_x;                   /**< x offset of the bounding box*/
    int8_t ofs_y;                  /**< y offset of the bounding box. Measured from the top of the line*/
} lv_font_fmt_txt_glyph_dsc_t;


/** Format of font character map. */
enum {
    LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY,
    LV_FONT_FMT_TXT_CMAP_FORMAT0_FULL,
    LV_FONT_FMT_TXT_CMAP_SPARSE_TINY,
    LV_FONT_FMT_TXT_CMAP_SPARSE_FULL,
};

typedef uint8_t lv_font_fmt_txt_cmap_type_t;

/* Map codepoints to a `glyph_dsc`s
 * Several formats are supported to optimize memory usage
 * See https://github.com/lvgl/lv_font_conv/blob/master/doc/font_spec.md
 */
typedef struct {
    /** First Unicode character for this range */
    uint32_t range_start;

    /** Number of Unicode characters related to this range.
     * Last Unicode character = range_start + range_length - 1*/
    uint16_t range_length;

    /** First glyph ID (array index of `glyph_dsc`) for this range */
    uint16_t glyph_id_start;

    const uint16_t * unicode_list;

    /** if(type == LV_FONT_FMT_TXT_CMAP_FORMAT0_...) it's `uint8_t *`
     * if(type == LV_FONT_FMT_TXT_CMAP_SPARSE_...)  it's `uint16_t *`
     */
    const void * glyph_id_ofs_list;

    /** Length of `unicode_list` and/or `glyph_id_ofs_list`*/
    uint16_t list_length;

    /** Type of this character map*/
    lv_font_fmt_txt_cmap_type_t type;
} lv_font_fmt_txt_cmap_t;


//lv_font_fmt_txt.h
/*Describe store additional data for fonts */
typedef struct {
    /*The bitmaps os all glyphs*/
    const uint8_t * glyph_bitmap;

    /*Describe the glyphs*/
    const lv_font_fmt_txt_glyph_dsc_t * glyph_dsc;

    /* Map the glyphs to Unicode characters.
     * Array of `lv_font_cmap_fmt_txt_t` variables*/
    const lv_font_fmt_txt_cmap_t * cmaps;

    /* Store kerning values.
     * Can be  `lv_font_fmt_txt_kern_pair_t *  or `lv_font_kern_classes_fmt_txt_t *`
     * depending on `kern_classes`
     */
    const void * kern_dsc;

    /*Scale kern values in 12.4 format*/
    uint16_t kern_scale;

    /*Number of cmap tables*/
    uint16_t cmap_num       : 10;

    /*Bit per pixel: 1, 2, 3, 4, 8*/
    uint16_t bpp            : 4;

    /*Type of `kern_dsc`*/
    uint16_t kern_classes   : 1;

    /*
     * storage format of the bitmap
     * from `lv_font_fmt_txt_bitmap_format_t`
     */
    uint16_t bitmap_format  : 2;

    /*Cache the last letter and is glyph id*/
    uint32_t last_letter;
    uint32_t last_glyph_id;

} lv_font_fmt_txt_dsc_t;

/** Describes the properties of a glyph. */
typedef struct {
    uint16_t adv_w; /**< The glyph needs this space. Draw the next glyph after this width. 8 bit integer, 4 bit fractional */
    uint16_t box_w;  /**< Width of the glyph's bounding box*/
    uint16_t box_h;  /**< Height of the glyph's bounding box*/
    int16_t ofs_x;   /**< x offset of the bounding box*/
    int16_t ofs_y;  /**< y offset of the bounding box*/
    uint8_t bpp;   /**< Bit-per-pixel: 1, 2, 4, 8*/
} lv_font_glyph_dsc_t;

//lv_font.h
/** Describe the properties of a font*/
typedef struct _lv_font_struct {
    /** Get a glyph's  descriptor from a font*/
    bool (*get_glyph_dsc)(const struct _lv_font_struct *, lv_font_glyph_dsc_t *, uint32_t letter, uint32_t letter_next);

    /** Get a glyph's bitmap from a font*/
    const uint8_t * (*get_glyph_bitmap)(const struct _lv_font_struct *, uint32_t);

    /*Pointer to the font in a font pack (must have the same line height)*/
    lv_coord_t line_height;         /**< The real line height where any text fits*/
    lv_coord_t base_line;           /**< Base line measured from the top of the line_height*/
    uint8_t subpx  : 2;             /**< An element of `lv_font_subpx_t`*/
    void * dsc;                     /**< Store implementation specific or run_time data or caching here*/
#if LV_USE_USER_DATA
    lv_font_user_data_t user_data;  /**< Custom user data for font. */
#endif
} lv_font_t;

// void Show_MCU_Str(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const char* txt,const lv_font_t* font);
void lcd_show_zh_string(const lv_font_t* font, uint16_t x,uint16_t y, char* txt, uint16_t font_color, uint16_t background_color);

void lcd_show_ascii_string(uint16_t x, uint16_t y, uint8_t size,uint8_t *p, uint16_t font_color, uint16_t background_color);
