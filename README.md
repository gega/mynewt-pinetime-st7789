# PineTime ST7789 LCD Driver for Apache Mynewt

This is a minimal, efficient driver for the ST7789 display on the PineTime smartwatch, designed specifically for the [Apache Mynewt](https://mynewt.apache.org/) embedded OS.

## Features

- No heap usage
- Non-blocking SPI interface
- Flash-friendly with interleaved decompression support
- Partial updates (great for UI icons or widgets)
- Basic drawing primitives
- Small memory footprint

## Who is this for?

Embedded developers working with:

- The PineTime smartwatch
- Apache Mynewt RTOS
- Custom firmware or bootloaders
- or other constrained devices

## Example Usage

Below is a simplified example showing how to initialize the display and stream a full-screen image using double buffering. This assumes your image is stored in flash in RGB565, little endian format.

```c
#include "pinetime_st7789/pinetime_st7789.h"

static uint8_t *next(uint8_t *buf, int *chunk, int remaining_len)
{
  static uint8_t internal_buf[2][PINETIME_ST7789_MAXTRANSFER];
  static uint8_t *external_buf;
  static int buf_idx = 0;
  int len;
  uint8_t *ret = NULL;

  if (chunk)
  {
    len = min(PINETIME_ST7789_MAXTRANSFER, remaining_len);
    memcpy(internal_buf[buf_idx], external_buf, len);  // Flash -> RAM for DMA access
    *chunk = len;
    external_buf += len;
    ret = internal_buf[buf_idx];
    buf_idx ^= 1;
  }
  else
  {
    external_buf = buf;
    buf_idx = 0;
  }

  return ret;
}

static void maintask(void *arg)
{
  pinetime_st7789_init();
  pinetime_st7789_clear();
  pinetime_st7789_brightness(PINETIME_BRIGHTNESS_HIGH);

  int len = PINETIME_ST7789_SCR_WIDTH * PINETIME_ST7789_SCR_HEIGHT * 2;
  next((uint8_t *)image_array, NULL, len);
  pinetime_st7789_stream_frame(next, len);
}
```
## Building

```
newt new lcdtest
cd lcdtest
newt upgrade
newt target create boot-pinetime
newt target set boot-pinetime app=@mcuboot/boot/mynewt
newt target set boot-pinetime bsp=@apache-mynewt-core/hw/bsp/pinetime
newt target set boot-pinetime build_profile=optimized
newt target create lcdtest-pinetime
pushd apps
cp -ax blinky/ lcdtest
# edit pkg.yml
#   add these:
#      - "@mynewt-pinetime-st7789/hw/drivers/display/pinetime_st7789"
#      - "@apache-mynewt-core/hw/drivers/flash/spiflash"
# edit main.c to have some driver API calls
popd
newt target set lcdtest-pinetime app=apps/lcdtest
newt target set lcdtest-pinetime bsp=@apache-mynewt-core/hw/bsp/pinetime
newt target set lcdtest-pinetime build_profile=debug
pushd repos
git clone git@github.com:gega/mynewt-pinetime-st7789.git
newt build boot-pinetime
newt build lcdtest-pinetime
newt create-image lcdtest-pinetime 1.0.0
newt create-image boot-pinetime 1.0.0
newt load boot-pinetime
newt load lcdtest-pinetime
```
