#include "pinetime_st7789/pinetime_st7789.h"

#include <assert.h>
#include <string.h>

#include "syscfg/syscfg.h"

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_spi.h"

#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "nrfx/hal/nrf_spi.h"


#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) ((sizeof x) / (sizeof *x))
#endif

#ifndef ABS
#define ABS(x)  (((x)<0)?-(x):(x))
#endif

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#define LCD_SPI_BUS  (0)

#define OFFS_DELAY  (0)
#define OFFS_LEN    (1)
#define OFFS_OPCODE (2)
#define OFFS_PARAM  (3)

#define HEADER_SIZE (3) /* delay, argc, opcode */

#define ST7789_2(o,d) 			d,0,o
#define ST7789_3(o,d,p1) 		d,1,o,p1
#define ST7789_4(o,d,p1,p2) 		d,2,o,p1,p2
#define ST7789_5(o,d,p1,p2,p3) 		d,3,o,p1,p2,p3
#define ST7789_6(o,d,p1,p2,p3,p4) 	d,4,o,p1,p2,p3,p4
#define ST7789_7(o,d,p1,p2,p3,p4,p5) 	d,5,o,p1,p2,p3,p4,p5
#define ST7789_8(o,d,p1,p2,p3,p4,p5,p6) d,6,o,p1,p2,p3,p4,p5,p6

#define GM(_1,_2,_3,_4,_5,_6,_7,_8,NAME,...) NAME
#define ST7789(...) GM(__VA_ARGS__, ST7789_8, ST7789_7, ST7789_6, ST7789_5, ST7789_4, ST7789_3, ST7789_2)(__VA_ARGS__)

#define OP_NOP             0x00        /* no operation command */
#define OP_SWRESET         0x01        /* software reset command */
#define OP_SLPIN           0x10        /* sleep in command */
#define OP_SLPOUT          0x11        /* sleep out command */
#define OP_PTLON           0x12        /* partial mode on command */
#define OP_NORON           0x13        /* normal display mode on command */
#define OP_INVOFF          0x20        /* display inversion off command */
#define OP_INVON           0x21        /* display inversion on command */
#define OP_GAMSET          0x26        /* display inversion set command */
#define OP_DISPOFF         0x28        /* display off command */
#define OP_DISPON          0x29        /* display on command */
#define OP_CASET           0x2a        /* column address set command */
#define OP_RASET           0x2b        /* row address set command */
#define OP_RAMWR           0x2c        /* memory write command */
#define OP_PTLAR           0x30        /* partial start/end address set command */
#define OP_VSCRDEF         0x33        /* vertical scrolling definition command */
#define OP_TEOFF           0x34        /* tearing effect line off command */
#define OP_TEON            0x35        /* tearing effect line on command */
#define OP_MADCTL          0x36        /* memory data access control command */
#define OP_VSCRSADD        0x37        /* vertical scrolling start address command */
#define OP_IDMOFF          0x38        /* idle mode off command */
#define OP_IDMON           0x39        /* idle mode on command */
#define OP_COLMOD          0x3a        /* interface pixel format command */
#define OP_RAMWRC          0x3c        /* memory write continue command */
#define OP_TESCAN          0x44        /* set tear scanline command */
#define OP_WRDISBV         0x51        /* write display brightness command */
#define OP_WRCTRLD         0x53        /* write CTRL display command */
#define OP_WRCACE          0x55        /* write content adaptive brightness control and color enhancement command */
#define OP_WRCABCMB        0x5e        /* write CABC minimum brightness command */
#define OP_RAMCTRL         0xb0        /* ram control command */
#define OP_RGBCTRL         0xb1        /* rgb control command */
#define OP_PORCTRL         0xb2        /* porch control command */
#define OP_FRCTRL1         0xb3        /* frame rate control 1 command */
#define OP_PARCTRL         0xb5        /* partial mode control command */
#define OP_GCTRL           0xb7        /* gate control command */
#define OP_GTADJ           0xb8        /* gate on timing adjustment command */
#define OP_DGMEN           0xba        /* digital gamma enable command */
#define OP_VCOMS           0xbb        /* vcoms setting command */
#define OP_LCMCTRL         0xc0        /* lcm control command */
#define OP_IDSET           0xc1        /* id setting command */
#define OP_VDVVRHEN        0xc2        /* vdv and vrh command enable command */
#define OP_VRHS            0xc3        /* vrh set command */
#define OP_VDVS            0xc4        /* vdv setting command */
#define OP_VCMOFSET        0xc5        /* vcoms offset set command */
#define OP_FRCTRL2         0xc6        /* fr control 2 command */
#define OP_CABCCTRL        0xc7        /* cabc control command */
#define OP_REGSEL1         0xc8        /* register value selection1 command */
#define OP_REGSEL2         0xca        /* register value selection2 command */
#define OP_PWMFRSEL        0xcc        /* pwm frequency selection command */
#define OP_PWCTRL1         0xd0        /* power control 1 command */
#define OP_VAPVANEN        0xd2        /* enable vap/van signal output command */
#define OP_CMD2EN          0xdf        /* command 2 enable command */
#define OP_PVGAMCTRL       0xe0        /* positive voltage gamma control command */
#define OP_NVGAMCTRL       0xe1        /* negative voltage gamma control command */
#define OP_DGMLUTR         0xe2        /* digital gamma look-up table for red command */
#define OP_DGMLUTB         0xe3        /* digital gamma look-up table for blue command */
#define OP_GATECTRL        0xe4        /* gate control command */
#define OP_SPI2EN          0xe7        /* spi2 command */
#define OP_PWCTRL2         0xe8        /* power control 2 command */
#define OP_EQCTRL          0xe9        /* equalize time control command */
#define OP_PROMCTRL        0xec        /* program control command */
#define OP_PROMEN          0xfa        /* program mode enable command */
#define OP_NVMSET          0xfc        /* nvm setting command */
#define OP_PROMACT         0xfe        /* program action command */

#define DELAY(ms) do { if((ms)>0) os_time_delay(((ms)*OS_TICKS_PER_SEC)/1000); } while(0)

#define SCR_WIDTH PINETIME_ST7789_SCR_WIDTH
#define SCR_HEIGHT PINETIME_ST7789_SCR_HEIGHT


static const uint8_t init_seq[]=
{
  //     opcode		delay	parameters						bitfields
  ST7789(OP_SWRESET,	200),
  ST7789(OP_CMD2EN,	0,	0x5a,0x69,0x02,0x01), 					/* C; C; C; EN(0) */
  ST7789(OP_SLPOUT,	200),
  ST7789(OP_COLMOD,	0,	0x55),							/* RGBicf(65k) CICF(16bit) */
  ST7789(OP_MADCTL,	0,	0x00),							/* MY(t2b) MX(l2r) MV(n) ML(t2b) RGB(rgb) MH(l2r) */
  ST7789(OP_CASET,	0,	0x00, 0x00, (SCR_WIDTH-1)>>8, (SCR_WIDTH-1)&0xff ),	/* SCR_WIDTH */
  ST7789(OP_RASET,	0,	0x00, 0x00, (SCR_HEIGHT-1)>>8, (SCR_HEIGHT-1)&0xff ),	/* SCR_HEIGHT */
  ST7789(OP_PORCTRL,	0,	0x02, 0x03, 0x01, 0xed, 0xed),				/* BPA(2); FPA(3); PSEN(1); BPB(14) FPB(13); BPC(14) FPC(13) */
  ST7789(OP_FRCTRL2,	0,	0x0a),							/* NLA(dot) RTNA(72) */
  ST7789(OP_FRCTRL1,	0,	0x00, 0x0a, 0x0a),					/* FRSEN(0) DV(0->div1); NLB(dot) RTNB(58); NLC(dot) RTNC(58) */
  ST7789(OP_NORON,	0),
  ST7789(OP_VDVS,	0,	0x10),							/* VDVS(-0.4) */
  ST7789(OP_PWCTRL1,	0,	0xa4, 0x00),						/* C; AVDD(6.4) AVCL(-4.4) VDS(2.19) */
  ST7789(OP_PWCTRL2,	0,	0x33),							/* SBCLK(/6) STP14CK(/6) */
  ST7789(OP_GCTRL,	0,	0x00),							/* VGHS(12.20) VGLS(-7.16) */
  ST7789(OP_EQCTRL,	0,	0x11, 0x11, 0x08),					/* sQ=SEQ(0x11); sP=SPRET(0x11); gQ=GEQ(0x08) */
  ST7789(OP_PROMEN,	0,	0x5a, 0x69, 0xee, 0x00),				/* C; C; C; PROMEN(off) */
  ST7789(OP_INVON,	0),
  ST7789(OP_DISPON,	0),
  ST7789(OP_IDMOFF,	50),
};
static struct os_sem mu_busy;
static int inited=0;

_Static_assert(PINETIME_ST7789_BUFFER_SIZE%2==0, "Internal buffer size should be even");
static uint16_t line_buf[2][PINETIME_ST7789_BUFFER_SIZE/2]; /* should be PINETIME_ST7789_BUFFER_SIZE bytes */
static volatile int lift_cs=0;
static int sleep_mode;


static void txrx_cb(void *arg, int len)
{
  if(lift_cs) hal_gpio_write(LCD_CHIP_SELECT_PIN, 1);
  lift_cs=0;
  os_sem_release(&mu_busy);
}

static void spi_wait(void)
{
  os_sem_pend(&mu_busy, OS_TIMEOUT_NEVER);
  os_sem_release(&mu_busy);
}

static int spi_init(void)
{
  int ret=0;
  static struct hal_spi_settings sst=
  {
    .data_order = HAL_SPI_MSB_FIRST,
    .data_mode  = HAL_SPI_MODE3,
    .baudrate   = MYNEWT_VAL(SPIFLASH_BAUDRATE),
    .word_size  = HAL_SPI_WORD_SIZE_8BIT,
  };

  if(OS_OK!=(os_sem_init(&mu_busy, 1))) ret=1;
  ret|=hal_gpio_init_out(LCD_CHIP_SELECT_PIN, 1);
  ret|=hal_spi_disable(LCD_SPI_BUS);
  ret|=hal_spi_config(LCD_SPI_BUS, &sst);
  ret|=hal_spi_set_txrx_cb(LCD_SPI_BUS, txrx_cb, NULL);
  ret|=hal_spi_enable(LCD_SPI_BUS);

  return(ret);
}

static void spi_deinit(void)
{
  hal_spi_abort(LCD_SPI_BUS);
  os_sem_pend(&mu_busy, 10);
  os_sem_release(&mu_busy);
  hal_gpio_write(LCD_CHIP_SELECT_PIN, 1);
  hal_spi_disable(LCD_SPI_BUS);
  hal_gpio_deinit(LCD_CHIP_SELECT_PIN);
}

_Static_assert(PINETIME_ST7789_BUFFER_SIZE<=PINETIME_ST7789_MAXTRANSFER, "Internal buffer cannot be larger than MAXTRANSFER unit");
static int spi_data_copy(const uint8_t *buf, int len)
{
  int ret=0;
  int bsel=0;

  if(len>0)
  {
    int rm=(len % PINETIME_ST7789_BUFFER_SIZE);
    int nt=(len / PINETIME_ST7789_BUFFER_SIZE) + !!rm;
    uint8_t *bf=(uint8_t *)buf;
    int chunk=(rm==0 ? PINETIME_ST7789_BUFFER_SIZE : rm);

    spi_wait();
    lift_cs=0;
    hal_gpio_write(LCD_WRITE_PIN, 1);
    hal_gpio_write(LCD_CHIP_SELECT_PIN, 0);
    for(int i=0; i<nt; i++)
    {
      bsel^=1;
      bf=(uint8_t *)line_buf[bsel];
      memcpy(bf, buf, chunk);
      os_sem_pend(&mu_busy, OS_TIMEOUT_NEVER);
      if(i>=nt) lift_cs=1;
      ret|=hal_spi_txrx_noblock(LCD_SPI_BUS, bf, NULL, chunk);
      bf+=chunk;
      chunk=PINETIME_ST7789_BUFFER_SIZE;
    }
  }

  return(ret);
}

static int spi_data_nocopy(const uint8_t *buf, int len)
{
  int ret=0;

  if(len>0)
  {
    int rm=(len % PINETIME_ST7789_MAXTRANSFER);
    int nt=(len / PINETIME_ST7789_MAXTRANSFER) + !!rm;
    uint8_t *bf=(uint8_t *)buf;
    int chunk=(rm==0 ? PINETIME_ST7789_MAXTRANSFER : rm);

    spi_wait();
    lift_cs=0;
    hal_gpio_write(LCD_WRITE_PIN, 1);
    hal_gpio_write(LCD_CHIP_SELECT_PIN, 0);
    for(int i=0; i<nt; i++)
    {
      os_sem_pend(&mu_busy, OS_TIMEOUT_NEVER);
      if(i>=nt) lift_cs=1;
      ret|=hal_spi_txrx_noblock(LCD_SPI_BUS, bf, NULL, chunk);
      bf+=chunk;
      chunk=PINETIME_ST7789_MAXTRANSFER;
    }
  }

  return(ret);
}

static int spi_data_feed(int len, uint8_t (*next_chunk_cb(uint8_t*,int*,int)))
{
  int ret=0;
  int chunk=0;
  uint8_t *buf;

  if(len>0)
  {
    spi_wait();
    hal_gpio_write(LCD_WRITE_PIN, 1);
    hal_gpio_write(LCD_CHIP_SELECT_PIN, 0);
    while(len>0)
    {
      chunk=0;
      buf=next_chunk_cb(NULL, &chunk, len);
      if(chunk<=0||chunk>PINETIME_ST7789_MAXTRANSFER||NULL==buf)
      {
        ret=-1;
        break;
      }
      os_sem_pend(&mu_busy, OS_TIMEOUT_NEVER);
      lift_cs=0;
      ret|=hal_spi_txrx_noblock(LCD_SPI_BUS, buf, NULL, chunk);
      len-=chunk;
    }
    spi_wait();
    hal_gpio_write(LCD_CHIP_SELECT_PIN, 1);
  }

  return(ret);
}

static int spi_cmd_param(const uint8_t *opcode, int argc, const uint8_t *param)
{
  int ret=0;
  
  if(NULL!=opcode)
  {
    spi_wait();
    hal_gpio_write(LCD_WRITE_PIN, 0);
    hal_gpio_write(LCD_CHIP_SELECT_PIN, 0);
    ret|=hal_spi_txrx(LCD_SPI_BUS, (uint8_t *)opcode, NULL, 1);
    hal_gpio_write(LCD_WRITE_PIN, 1);
    if(argc>0&&NULL!=param) ret|=hal_spi_txrx(LCD_SPI_BUS, (uint8_t *)param, NULL, argc);
    hal_gpio_write(LCD_CHIP_SELECT_PIN, 1);
  }
  
  return(ret);
}

static int gpio_init(void)
{
  int ret=0;
  
  ret|=hal_gpio_init_out(LCD_WRITE_PIN, 1);
  ret|=hal_gpio_init_out(LCD_RESET_PIN, 1);
  ret|=hal_gpio_init_out(LCD_BACKLIGHT_LOW_PIN, 0);
  ret|=hal_gpio_init_out(LCD_BACKLIGHT_MED_PIN, 0);
  ret|=hal_gpio_init_out(LCD_BACKLIGHT_HIGH_PIN, 0);
  
  return(ret);
}

static void gpio_deinit(void)
{
  hal_gpio_deinit(LCD_WRITE_PIN);
  hal_gpio_deinit(LCD_RESET_PIN);
  hal_gpio_deinit(LCD_BACKLIGHT_LOW_PIN);
  hal_gpio_deinit(LCD_BACKLIGHT_MED_PIN);
  hal_gpio_deinit(LCD_BACKLIGHT_HIGH_PIN);
}

static void send_seq(const uint8_t *seq, int len)
{
  for(int i=0; i<len; i+=HEADER_SIZE)
  {
    spi_cmd_param(&seq[i+OFFS_OPCODE], seq[i+OFFS_LEN], &seq[i+OFFS_PARAM]);
    DELAY(seq[i+OFFS_DELAY]);
    i+=seq[i+OFFS_LEN];
  }
}

void pinetime_st7789_put_frame(const uint8_t *rgb565img)
{
  static const uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	0, 0, (SCR_WIDTH-1)>>8,  (SCR_WIDTH-1)&0xff ),
    ST7789(OP_RASET,	0,	0, 0, (SCR_HEIGHT-1)>>8, (SCR_HEIGHT-1)&0xff ),
    ST7789(OP_RAMWR,	0),
  };
  if(inited)
  {
    send_seq(set_window, sizeof(set_window));
    spi_data_copy(rgb565img, (SCR_WIDTH*SCR_HEIGHT)*2);
  }
}

void pinetime_st7789_stream_frame(next_chunk_cb_t next_chunk, int len)
{
  static const uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	0, 0, (SCR_WIDTH-1)>>8,  (SCR_WIDTH-1)&0xff ),
    ST7789(OP_RASET,	0,	0, 0, (SCR_HEIGHT-1)>>8, (SCR_HEIGHT-1)&0xff ),
    ST7789(OP_RAMWR,    0),
  };
  if(inited)
  {
    send_seq(set_window, sizeof(set_window));
    spi_data_feed(len, next_chunk);
  }
}

void pinetime_st7789_init(void)
{
  if(!inited)
  {
    int st=0;
    st|=gpio_init();
    st|=spi_init();
    if(st==0) inited=1;
    hal_gpio_write(LCD_RESET_PIN, 0);
    DELAY(100);
    hal_gpio_write(LCD_RESET_PIN, 1);
    DELAY(125);
    send_seq(init_seq, sizeof(init_seq));
  }
}

void pinetime_st7789_deinit(void)
{
  if(inited)
  {
    spi_deinit();
    gpio_deinit();
    inited=0;
  }
}

void pinetime_st7789_capabilities(struct pinetime_st7789_capabilities *cap)
{
  if(NULL!=cap)
  {
    cap->pixel_width=PINETIME_ST7789_SCR_WIDTH;
    cap->pixel_height=PINETIME_ST7789_SCR_HEIGHT;
    cap->maxtransfer=PINETIME_ST7789_MAXTRANSFER;
    cap->pixel_format=PINETIME_PXLFMT_RGB565;
    cap->buffer_size=PINETIME_ST7789_BUFFER_SIZE;
  }
}

void pinetime_st7789_put_icon(const uint8_t *rgb565buffer, int x, int y, int w, int h, int copytoram)
{
  uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	0, x, 0, x+w-1 ),
    ST7789(OP_RASET,	0,	0, y, 0, y+h-1 ),
    ST7789(OP_RAMWR,    0),
  };
  if(inited)
  {
    send_seq(set_window, sizeof(set_window));
    if(copytoram) spi_data_copy(rgb565buffer, w*h*2);
    else spi_data_nocopy(rgb565buffer, w*h*2);
  }
}

void pinetime_st7789_stream_icon(next_chunk_cb_t next_chunk, int x, int y, int w, int h)
{
  uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	0, x, 0, x+w-1 ),
    ST7789(OP_RASET,	0,	0, y, 0, y+h-1 ),
    ST7789(OP_RAMWR,    0),
  };
  if(inited)
  {
    send_seq(set_window, sizeof(set_window));
    spi_data_feed(w*h*2, next_chunk);
  }
}

static uint8_t *ncc(uint8_t *buf, int *chunk, int remaining_len)
{
  static uint8_t *b;
  static int buflen;
  uint8_t *ret=NULL;

  if(NULL!=chunk)
  {
    *chunk=MIN(buflen, remaining_len);
    ret=b;
  }
  else
  {
    b=buf;
    buflen=remaining_len;
  }

  return(ret);
}

void pinetime_st7789_brightness(pinetime_st7789_brightness_t brightness)
{
  static const int leds[][3]=
  {
    //                           	  LCD_BACKLIGHT_LOW_PIN	LCD_BACKLIGHT_MED_PIN	LCD_BACKLIGHT_HIGH_PIN
    [PINETIME_BRIGHTNESS_OFF]=		{ 0, 			0, 			0 },
    [PINETIME_BRIGHTNESS_LOW]=		{ 1, 			0, 			0 },
    [PINETIME_BRIGHTNESS_MEDIUM]=	{ 0,			1,			0 },
    [PINETIME_BRIGHTNESS_HIGH]=		{ 0,			0,			1 },
    [PINETIME_BRIGHTNESS_MAX]=		{ 1,			1,			1 },
  };
  // TODO: add PWM dimmer
  static const int backlight_pins[]=	{ LCD_BACKLIGHT_LOW_PIN, LCD_BACKLIGHT_MED_PIN, LCD_BACKLIGHT_HIGH_PIN };
  
  for(int i=0;i<ARRAY_SIZE(backlight_pins);i++)
  {
    hal_gpio_write(backlight_pins[i], leds[brightness][i]);
  }
}

void pinetime_st7789_put_pixel_rgb565(int x, int y, uint16_t rgb565)
{
  uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	x>>8, x&0xff, x>>8, x&0xff ),
    ST7789(OP_RASET,	0,	y>>8, y&0xff, y>>8, y&0xff ),
    ST7789(OP_RAMWR,    0,	rgb565>>8, rgb565&0xff),
  };

  if(inited)
  {
    send_seq(set_window, sizeof(set_window));
  }
}

void pinetime_st7789_put_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
  pinetime_st7789_put_pixel_rgb565(x, y, RGB_TO_RGB565LE(r,g,b));
}

// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
void pinetime_st7789_draw_line(uint8_t r, uint8_t g, uint8_t b, int x0, int y0, int x1, int y1)
{
  int dx = ABS(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -ABS(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int error = (dx + dy)*100;
  int e2;
  uint16_t col;

  col=RGB_TO_RGB565LE(r,g,b);
  do
  {
    pinetime_st7789_put_pixel_rgb565(x0, y0, col);
    e2 = (2 * error) / 100;
    if( e2 >= dy)
    {
      if (x0 == x1) break;
      error = error + dy*100;
      x0 = x0 + sx;
    }
    if( e2 <= dx)
    {
      if (y0 == y1) break;
      error = error + dx*100;
      y0 = y0 + sy;
    }
  } while(1);
}

void pinetime_st7789_draw_horiz_line(uint8_t r, uint8_t g, uint8_t b, int y, int x0, int x1)
{
  uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	0, x0, 0, x1 ),
    ST7789(OP_RASET,	0,	0, y, 0, y ),
    ST7789(OP_RAMWR,    0),
  };
  uint16_t col=RGB_TO_RGB565LE(r,g,b);
  
  if(x1==x0) return;
  if(x0>x1) { int t=x0; x0=x1; x1=t; }
  if(inited)
  {
    int blen=MIN(ARRAY_SIZE(line_buf[0]),2*(x1-x0));
    for(int i=0; i<blen; i++) line_buf[0][i]=col;
    ncc((uint8_t *)line_buf[0], NULL, blen);
    send_seq(set_window, sizeof(set_window));
    spi_data_feed(2*(x1-x0), ncc);
  }  
}

void pinetime_st7789_draw_horiz_tex(uint8_t *tex, int y, int x0, int x1)
{
  uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	0, x0, 0, x1 ),
    ST7789(OP_RASET,	0,	0, y, 0, y ),
    ST7789(OP_RAMWR,    0),
  };
  
  if(x1==x0) return;
  if(x0>x1) { int t=x0; x0=x1; x1=t; }
  if(inited)
  {
    send_seq(set_window, sizeof(set_window));
    spi_data_nocopy(tex, 2*(x1-x0+1));
  }  
}

void pinetime_st7789_fill_rect(uint8_t r, uint8_t g, uint8_t b, int x, int y, int w, int h)
{
  uint8_t set_window[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_CASET,	0,	0, x, 0, x+w-1 ),
    ST7789(OP_RASET,	0,	0, y, 0, y+h-1 ),
    ST7789(OP_RAMWR,    0),
  };

  if(inited&&w>0&&h>0&&x>=0&&y>=0&&x<SCR_WIDTH&&y<SCR_HEIGHT&&(w+x)<=SCR_WIDTH&&(h+y)<=SCR_WIDTH)
  {
    for(int i=0; i<ARRAY_SIZE(line_buf[0]); i++) line_buf[0][i]=RGB_TO_RGB565LE(r,g,b);
    ncc((uint8_t *)line_buf[0], NULL, sizeof(line_buf[0]));
    send_seq(set_window, sizeof(set_window));
    spi_data_feed(2*w*h, ncc);
  }
}

void pinetime_st7789_fill(uint8_t r, uint8_t g, uint8_t b)
{
  pinetime_st7789_fill_rect(r,g,b,0,0,SCR_WIDTH,SCR_HEIGHT);
}

void pinetime_st7789_clear(void)
{
  pinetime_st7789_fill(255,255,255);
}

void pinetime_st7789_wait_for_transfer(void)
{
  spi_wait();
}

void pinetime_st7789_sleep(int en)
{
  static const uint8_t sleep[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_SLPIN,	126),
  };
  static const uint8_t wakeup[]=
  {
    //        opcode	delay	parameters
    ST7789(OP_SLPOUT,	6),
    ST7789(OP_VSCRSADD, 0,	0, 0),
    ST7789(OP_DISPON,   0),
  };
  if(en!=sleep_mode)
  {
    if(en)
    {
      // sleep
      DELAY(120);
      send_seq(sleep, sizeof(sleep));
    }
    else
    {
      // wakeup
      send_seq(wakeup, sizeof(wakeup));
    }
  }
}
