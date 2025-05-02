#ifndef PINETIME_ST7789_H
#define PINETIME_ST7789_H

#include <stdint.h>


/*
 * rgb 565 little endian
 * 111111
 * 5432109876543210
 * gggBbbbbRrrrrGgg
 */
#define RGB_TO_RGB565LE(r, g, b) (uint16_t)(((((uint16_t)g))<<13) | ((((uint16_t)b)&0xf8)<<5) | (((uint16_t)r)&0xf8) | (((uint16_t)g)>>5))
/*
 * rgb 565 big endian (normal)
 * 111111
 * 5432109876543210
 * RrrrrGgggggBbbbb
 */
#define RGB_TO_RGB565BE(r, g, b) (((((uint16_t)r) & 0xf8) << 8) | ((((uint16_t)g) & 0xfc) << 3) | (((uint16_t)b) >> 3))

#define PINETIME_ST7789_BUFFER_SIZE (120) /* less or equal than PINETIME_ST7789_MAXTRANSFER and must be even */
#define PINETIME_ST7789_MAXTRANSFER (252) /* max 255, nRF52 SPIM DMA limit */

_Static_assert(PINETIME_ST7789_BUFFER_SIZE<=PINETIME_ST7789_MAXTRANSFER, "Internal buffer cannot be larger than MAXTRANSFER unit");
_Static_assert(PINETIME_ST7789_BUFFER_SIZE%2==0, "Internal buffer size should be even");

#define PINETIME_ST7789_SCR_WIDTH  (240)
#define PINETIME_ST7789_SCR_HEIGHT (240)

#define PINETIME_ST7789_CS_GPIO  	(25)
#define PINETIME_ST7789_DATA_GPIO 	(18)
#define PINETIME_ST7789_RESET_GPIO	(26)


typedef enum pinetime_st7789_pixel_format
{
  PINETIME_PXLFMT_RGB565,
  PINETIME_PXLFMTEND
} pinetime_st7789_pixel_format_t;

typedef enum pinetime_st7789_brightness
{
  PINETIME_BRIGHTNESS_OFF=0,
  PINETIME_BRIGHTNESS_LOW,
  PINETIME_BRIGHTNESS_MEDIUM,
  PINETIME_BRIGHTNESS_HIGH,
  PINETIME_BRIGHTNESS_MAX,
  PINETIME_BRIGHTNESSEND
} pinetime_st7789_brightness_t;

struct pinetime_st7789_capabilities
{
  int pixel_width;
  int pixel_height;
  int maxtransfer;
  pinetime_st7789_pixel_format_t pixel_format;
  int buffer_size;
};

typedef uint8_t *next_chunk_cb_t(uint8_t*,int*,int);

void pinetime_st7789_init(void);
void pinetime_st7789_deinit(void);
void pinetime_st7789_put_frame(const uint8_t *rgb565img);
void pinetime_st7789_stream_frame(next_chunk_cb_t next_chunk, int len);
void pinetime_st7789_put_lines(const uint8_t *rgb565buffer, int y, int line_cnt);
void pinetime_st7789_draw_line(uint8_t r, uint8_t g, uint8_t b, int x0, int y0, int x1, int y1);
void pinetime_st7789_fill_rect(uint8_t r, uint8_t g, uint8_t b, int x, int y, int w, int h);
void pinetime_st7789_put_icon(const uint8_t *rgb565buffer, int x, int y, int w, int h);
void pinetime_st7789_stream_icon(next_chunk_cb_t next_chunk, int x, int y, int w, int h);
void pinetime_st7789_clear();
void pinetime_st7789_fill(uint8_t r, uint8_t g, uint8_t b);
void pinetime_st7789_brightness(pinetime_st7789_brightness_t brightness);
void pinetime_st7789_put_pixel_rgb565(int x, int y, uint16_t rgb565);
void pinetime_st7789_put_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);
void pinetime_st7789_capabilities(struct pinetime_st7789_capabilities *cap);
void pinetime_st7789_wait_for_transfer(void);

#endif
