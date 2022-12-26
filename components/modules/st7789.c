// Portions of this file taken from https://github.com/russhughes/st7789_mpy
// the copyright notice for which is included below.

/*
 * Modifications and additions Copyright (c) 2020, 2021 Russ Hughes
 *
 * This file licensed under the MIT License and incorporates work covered by
 * the following copyright and permission notice:
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ivan Belokobylskiy
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <string.h>
#include "module.h"
#include "common.h"
#include "lauxlib.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "spi_common.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define _swap_int(a, b) do { int t = a; a = b; b = t; } while(0)

// color modes
#define COLOR_MODE_65K      0x50
#define COLOR_MODE_262K     0x60
#define COLOR_MODE_12BIT    0x03
#define COLOR_MODE_16BIT    0x05
#define COLOR_MODE_18BIT    0x06
#define COLOR_MODE_16M      0x07

// commands
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_VSCRDEF 0x33
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36
#define ST7789_VSCSAD  0x37

#define ST7789_MADCTL_MY  0x80  // Page Address Order
#define ST7789_MADCTL_MX  0x40  // Column Address Order
#define ST7789_MADCTL_MV  0x20  // Page/Column Order
#define ST7789_MADCTL_ML  0x10  // Line Address Order
#define ST7789_MADCTL_MH  0x04  // Display Data Latch Order
#define ST7789_MADCTL_RGB 0x00
#define ST7789_MADCTL_BGR 0x08

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

#define COLOR_BLACK   0x0000
#define COLOR_BLUE    0x001F
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW  0xFFE0
#define COLOR_WHITE   0xFFFF

#define ROTATE_0 0
#define ROTATE_90 1
#define ROTATE_180 2
#define ROTATE_270 3

typedef struct _st7789_rotation_t {
  uint8_t madctl;
  uint16_t width;
  uint16_t height;
  uint16_t colstart;
  uint16_t rowstart;
} st7789_rotation_t;

//
// Default st7789 and st7735 display orientation tables
// can be overridden during init(), madctl values
// will be combined with color_mode
//
st7789_rotation_t ORIENTATIONS_240x320[4] = {
  { 0x00, 240, 320,  0,  0},
  { 0x60, 320, 240,  0,  0},
  { 0xc0, 240, 320,  0,  0},
  { 0xa0, 320, 240,  0,  0}
};

st7789_rotation_t ORIENTATIONS_170x320[4] = {
  {0x00, 170, 320, 35, 0},
  {0x60, 320, 170, 0, 35},
  {0xc0, 170, 320, 35, 0},
  {0xa0, 320, 170, 0, 35}
};

st7789_rotation_t ORIENTATIONS_240x240[4] = {
  {0x00, 240, 240,  0,  0},
  {0x60, 240, 240,  0,  0},
  {0xc0, 240, 240,  0, 80},
  {0xa0, 240, 240, 80,  0}
};

st7789_rotation_t ORIENTATIONS_135x240[4] = {
  {0x00, 135, 240, 52, 40},
  {0x60, 240, 135, 40, 53},
  {0xc0, 135, 240, 53, 40},
  {0xa0, 240, 135, 40, 52}
};

st7789_rotation_t ORIENTATIONS_128x160[4] = {
  {0x00, 128, 160, 0, 0},
  {0x60, 160, 128, 0, 0},
  {0xc0, 128, 160, 0, 0},
  {0xa0, 160, 128, 0, 0}
};

st7789_rotation_t ORIENTATIONS_128x128[4] = {
  {0x00, 128, 128, 2, 1},
  {0x60, 128, 128, 1, 2},
  {0xc0, 128, 128, 2, 3},
  {0xa0, 128, 128, 3, 2}
};

typedef struct {
  spi_device_handle_t device;
  int device_ref;

  gpio_num_t reset_pin;
  gpio_num_t dc_pin;
  gpio_num_t cs_pin;
  gpio_num_t backlight_pin;

  uint16_t display_width;     // physical width
  uint16_t width;             // logical width (after rotation)
  uint16_t display_height;    // physical width
  uint16_t height;            // logical height (after rotation)
  uint8_t colstart;
  uint8_t rowstart;
  uint8_t color_order;
  uint8_t rotation;
  bool inversion;


} st7789_t;

static const char st7789_mt[] = "st7789.mt";

// This really needs to be exposed somewhere...
typedef struct {
  spi_device_handle_t device;
  int host_ref, host;
} lspi_device_t;

static int st7789_ST7789(lua_State *L)
{
  lua_settop(L, 1);
  st7789_t *obj = (st7789_t *)lua_newuserdata(L, sizeof(st7789_t));
  luaL_getmetatable(L, st7789_mt);
  lua_setmetatable(L, -2);
  lua_insert(L, -2); // Move obj below options

  obj->device_ref = LUA_NOREF; // for st7789_gc(), in case below errors
  if (!opt_get(L, "spidevice", LUA_TUSERDATA)) {
    opt_error(L, "spidevice", "spidevice must be specified");
  }
  lspi_device_t *spidevice = (lspi_device_t *)luaL_checkudata( L, -1, "spi.device" );
  obj->device = spidevice->device;
  obj->device_ref = luaL_ref(L, LUA_REGISTRYINDEX); // pops spidevice

  obj->display_width = opt_checkint_range(L, "width", 0, 1, 320);
  obj->display_height = opt_checkint_range(L, "height", 0, 1, 320);
  obj->width = obj->display_width;
  obj->height = obj->display_height;
  obj->dc_pin = opt_checkint_range(L, "dc", -1, 0, GPIO_PIN_COUNT);
  obj->reset_pin = opt_checkint_range(L, "reset", -1, -1, GPIO_PIN_COUNT);
  obj->cs_pin = opt_checkint_range(L, "cs", -1, -1, GPIO_PIN_COUNT);
  obj->backlight_pin = opt_checkint_range(L, "backlight", -1, -1, GPIO_PIN_COUNT);
  obj->inversion = opt_checkbool(L, "inversion", false);
  obj->rotation = opt_checkint_range(L, "rotation", 0, 0, 3);
  obj->color_order = opt_checkint(L, "color_order", ST7789_MADCTL_RGB);

  lua_pop(L, 1); // remove options
  return 1;
}

static inline st7789_t *get_self(lua_State *L)
{
  return (st7789_t *)luaL_checkudata(L, 1, st7789_mt);
}

static int st7789_gc(lua_State *L)
{
  st7789_t *obj = get_self(L);
  luaL_unref(L, LUA_REGISTRYINDEX, obj->device_ref);
  obj->device_ref = LUA_NOREF;
  return 0;
}

static inline void set_reset(st7789_t *self, uint32_t val)
{
  if (self->reset_pin != -1) {
    gpio_set_level(self->reset_pin, val);
  }
}

static inline void set_cs(st7789_t *self, uint32_t val)
{
  if (self->cs_pin != -1) {
    gpio_set_level(self->cs_pin, val);
  }
}

static inline void set_dc(st7789_t *self, uint32_t val)
{
  gpio_set_level(self->dc_pin, val);
}

static inline void delay(int ms) {
  TickType_t xDelay = ms / portTICK_PERIOD_MS;
  vTaskDelay(xDelay);
}

static void st7789_hard_reset(st7789_t *self)
{
  set_cs(self, 0);
  set_reset(self, 1);
  delay(50);
  set_reset(self, 0);
  delay(50);
  set_reset(self, 1);
  delay(150);
  set_cs(self, 1);
}

static void st7789_cmd(st7789_t *self, const uint8_t* buf, size_t len)
{
  spi_transaction_t trans = {0};
  memcpy(trans.tx_data, buf, 1);
  trans.length = 8; // ie 1 byte
  trans.flags = SPI_TRANS_USE_TXDATA;
  trans.rx_buffer = NULL;
  trans.rxlength = 0;
  set_cs(self, 0);
  set_dc(self, 0);
  spi_device_transmit(self->device, &trans);
  if (len > 1) {
    set_dc(self, 1);
    trans.length = (len - 1) * 8;
    trans.rxlength = 0;
    memcpy(trans.tx_data, buf + 1, len - 1);
    spi_device_transmit(self->device, &trans);
  }
  set_cs(self, 1);
}

// Convenience macro to make calling st7789_cmd() easier. Note the payload is
// limited to 4 bytes because we hard-code use of txdata to simplify the logic.
// Fortunately there doesn't seem to be any commands that require more than 4
// data bytes.
#define cmd(self, ...) do { \
  const uint8_t bytes[] = { __VA_ARGS__ }; \
  _Static_assert(sizeof(bytes) <= 5, "cmd length must be <= 5 bytes"); \
  st7789_cmd(self, bytes, sizeof(bytes)); \
} while(0)

static void st7789_soft_reset(st7789_t *self)
{
  cmd(self, ST7789_SWRESET);
  delay(150);
}

static void set_rotation(lua_State *L, st7789_t *self)
{
  st7789_rotation_t *rotations = NULL; // TODO custom rotations check
  if (rotations == NULL) {
    if (self->display_width == 240 && self->display_height == 320) {
      rotations = ORIENTATIONS_240x320;
    } else if (self->display_width == 170 && self->display_height == 320) {
      rotations = ORIENTATIONS_170x320;
    } else if  (self->display_width == 240 && self->display_height == 240) {
      rotations = ORIENTATIONS_240x240;
    } else if (self->display_width == 135 && self->display_height == 240) {
      rotations = ORIENTATIONS_135x240;
    } else if (self->display_width == 128 && self->display_height == 160) {
      rotations = ORIENTATIONS_128x160;
    } else if (self->display_width == 128 && self->display_height == 128) {
      rotations = ORIENTATIONS_128x128;
    }
  }

  if (!rotations) {
    luaL_error(L, "No madctl/rotations data for this display size!");
  }

  st7789_rotation_t *rotation = &rotations[self->rotation];
  uint8_t madctl_value = rotation->madctl;
  self->width  = rotation->width;
  self->height = rotation->height;
  self->colstart = rotation->colstart;
  self->rowstart = rotation->rowstart;

  madctl_value |= self->color_order;

  cmd(self, ST7789_MADCTL, madctl_value);
}

static void set_window(st7789_t *self, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  if (x0 > x1 || x1 >= self->width || y0 > y1 || y1 >= self->height) {
    return;
  }
  uint8_t colstart = self->colstart;
  uint8_t rowstart = self->rowstart;
  cmd(self, ST7789_CASET, (x0 + colstart) >> 8, (x0 + colstart) & 0xFF, (x1 + colstart) >> 8, (x1 + colstart) & 0xFF);
  cmd(self, ST7789_RASET, (y0 + rowstart) >> 8, (y0 + rowstart) & 0xFF, (y1 + rowstart) >> 8, (y1 + rowstart) & 0xFF);
  cmd(self, ST7789_RAMWR);
}

#define PIXEL_BUFSZ 128

static void fill_color_buffer(spi_device_handle_t spidevice, uint16_t color, int length)
{
  if (length == 1) {
    // Fast path for pixel(), that we can do with txdata
    spi_transaction_t trans = {0};
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = color >> 8;
    trans.tx_data[1] = color << 8;
    trans.length = 16;
    spi_device_transmit(spidevice, &trans);
    return;
  }

  const uint16_t color_be = (color << 8) | (color >> 8);
  const int nbufs = length / PIXEL_BUFSZ;
  const int rest = length % PIXEL_BUFSZ;
  const int n = nbufs == 0 ? length : PIXEL_BUFSZ;

  uint16_t *buffer = (uint16_t *)heap_caps_malloc(n * 2, MALLOC_CAP_DMA);
  // TODO error check

  for (int i = 0; i < n; i++) {
    buffer[i] = color_be;
  }

  spi_transaction_t trans = {0};
  trans.tx_buffer = buffer;
  trans.length = 16 * PIXEL_BUFSZ;
  trans.rx_buffer = NULL;

  for (int i = 0; i < nbufs; i++) {
    spi_device_transmit(spidevice, &trans);
  }
  if (rest) {
    trans.length = 16 * rest;
    trans.rxlength = 0; // This gets reset by the earlier spi_device_transmit call, sigh
    spi_device_transmit(spidevice, &trans);
  }

  heap_caps_free(buffer);
}

static int clamp(int val, int min_val, int max_val)
{
  if (val < min_val) {
    val = min_val;
  } else if (val > max_val) {
    val = max_val;
  }
  return val;
}

static void intersect_with_screen(st7789_t *self, int *x, int *y, int *w, int *h)
{
  int right  = clamp(*x + *w - 1, 0, self->width - 1);
  int bottom = clamp(*y + *h - 1, 0, self->height - 1);
  *x = clamp(*x, 0, self->width - 1);
  *y = clamp(*y, 0, self->height - 1);
  *w = (right - *x) + 1;
  *h = (bottom - *y) + 1;
}

static void fill_rect(st7789_t *self, int x, int y, int w, int h, uint16_t color)
{
  intersect_with_screen(self, &x, &y, &w, &h);
  int right  = x + w - 1;
  int bottom = y + h - 1;

  set_window(self, x, y, right, bottom);
  set_dc(self, 1);
  set_cs(self, 0);
  fill_color_buffer(self->device, color, w * h);
  set_cs(self, 1);
}

static uint16_t check_color(lua_State *L, int idx)
{
  int color = luaL_checkinteger(L, idx);
  luaL_argcheck(L, color >= 0 && color <= 0xFFFF, idx, "Bad color value");
  return color;
}

static int st7789_init(lua_State *L)
{
  st7789_t *self = get_self(L);
  if (lua_isnoneornil(L, 2)) {
    lua_settop(L, 1);
    lua_pushinteger(L, COLOR_BLACK);
  }
  int bgcolor = check_color(L, 2);
  st7789_hard_reset(self);
  st7789_soft_reset(self);
  cmd(self, ST7789_SLPOUT);
  cmd(self, ST7789_COLMOD, COLOR_MODE_65K | COLOR_MODE_16BIT);
  delay(10);
  set_rotation(L, self);
  cmd(self, self->inversion ? ST7789_INVON : ST7789_INVOFF);
  delay(10);
  cmd(self, ST7789_NORON);
  delay(10);
  fill_rect(self, 0, 0, self->width, self->height, bgcolor);
  if (self->backlight_pin != -1) {
    gpio_set_level(self->backlight_pin, 1);
  }
  cmd(self, ST7789_DISPON);
  delay(150);

  return 0;
}

static uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint16_t)r & 0xF8) << 8) | (((uint16_t)g & 0xFC) << 3) | (((uint16_t)b & 0xF8) >> 3);
}

static int st7789_color565(lua_State *L)
{
  int r = luaL_checkinteger(L, 1);
  int g = luaL_checkinteger(L, 2);
  int b = luaL_checkinteger(L, 3);
  luaL_argcheck(L, r >= 0 && r <= 255, 1, "RGB component must be in range 0-255");
  luaL_argcheck(L, g >= 0 && g <= 255, 1, "RGB component must be in range 0-255");
  luaL_argcheck(L, b >= 0 && b <= 255, 1, "RGB component must be in range 0-255");
  lua_pushinteger(L, color565((uint8_t)r, (uint8_t)g, (uint8_t)b));
  return 1;
}

static int st7789_fill_rect(lua_State *L)
{
  st7789_t *self = get_self(L);
  int x = luaL_checkinteger(L, 2);
  int y = luaL_checkinteger(L, 3);
  int w = luaL_checkinteger(L, 4);
  int h = luaL_checkinteger(L, 5);
  int color = check_color(L, 6);
  fill_rect(self, x, y, w, h, color);
  return 0;
}

static int st7789_clear_screen(lua_State *L)
{
  st7789_t *self = get_self(L);
  fill_rect(self, 0, 0, self->width, self->height, COLOR_BLACK);
  return 0;
}

static void hline(st7789_t *self, int x, int y, int w, uint16_t color)
{
  fill_rect(self, x, y, w, 1, color);
}

static void vline(st7789_t *self, int x, int y, int h, uint16_t color)
{
  fill_rect(self, x, y, 1, h, color);
}

static void pixel(st7789_t *self, int x, int y, uint16_t color)
{
  fill_rect(self, x, y, 1, 1, color);
}

static void line(st7789_t *self, int x0, int y0, int x1, int y1, uint16_t color)
{
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int(x0, y0);
    _swap_int(x1, y1);
  }

  if (x0 > x1) {
    _swap_int(x0, x1);
    _swap_int(y0, y1);
  }

  int dx = x1 - x0, dy = abs(y1 - y0);
  int err = dx >> 1, ystep = -1, xs = x0, dlen = 0;

  if (y0 < y1) {
    ystep = 1;
  }

  // Split into steep and not steep for FastH/V separation
  if (steep) {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) {
          pixel(self, y0, xs, color);
        } else {
          vline(self, y0, xs, dlen, color);
        }
        dlen = 0;
        y0 += ystep;
        xs = x0 + 1;
      }
    }
    if (dlen) {
      vline(self, y0, xs, dlen, color);
    }
  } else {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) {
          pixel(self, xs, y0, color);
        } else {
          hline(self, xs, y0, dlen, color);
        }
        dlen = 0;
        y0 += ystep;
        xs = x0 + 1;
      }
    }
    if (dlen) {
      hline(self, xs, y0, dlen, color);
    }
  }
}

static int st7789_line(lua_State *L)
{
  st7789_t *self = get_self(L);
  int x0 = luaL_checkinteger(L, 2);
  int y0 = luaL_checkinteger(L, 3);
  int x1 = luaL_checkinteger(L, 4);
  int y1 = luaL_checkinteger(L, 5);
  uint16_t color = check_color(L, 6);

  if (x0 == x1) {
    vline(self, x0, y0, (y1 - y0) + 1, color);
  } else if (y0 == y1) {
    hline(self, x0, x1, y0, color);
  } else {
    line(self, x0, y0, x1, y1, color);
  }
  return 0;
}

static int st7789_draw_rect(lua_State *L)
{
  st7789_t *self = get_self(L);
  int x = luaL_checkinteger(L, 2);
  int y = luaL_checkinteger(L, 3);
  int w = luaL_checkinteger(L, 4);
  int h = luaL_checkinteger(L, 5);
  int color = check_color(L, 6);
  hline(self, x, y, w, color);
  hline(self, x, y + h - 1, w, color);
  vline(self, x, y, h, color);
  vline(self, x + w - 1, y, h, color);
  return 0;
}

static int st7789_draw_pixel(lua_State *L)
{
  st7789_t *self = get_self(L);
  int x = luaL_checkinteger(L, 2);
  int y = luaL_checkinteger(L, 3);
  int color = check_color(L, 4);
  pixel(self, x, y, color);
  return 0;
}

// Circle/Fill_Circle by https://github.com/c-logic
// https://github.com/russhughes/st7789_mpy/pull/46
// https://github.com/c-logic/st7789_mpy.git patch-1
static int st7789_draw_circle(lua_State *L)
{
  st7789_t *self = get_self(L);
  int xm = luaL_checkinteger(L, 2);
  int ym = luaL_checkinteger(L, 3);
  int r = luaL_checkinteger(L, 4);
  uint16_t color = check_color(L, 5);

  int f     = 1 - r;
  int ddF_x = 1;
  int ddF_y = -2 * r;
  int x     = 0;
  int y     = r;

  pixel(self, xm, ym + r, color);
  pixel(self, xm, ym - r, color);
  pixel(self, xm + r, ym, color);
  pixel(self, xm - r, ym, color);
  while (x < y) {
    if (f >= 0) {
      y -= 1;
      ddF_y += 2;
      f += ddF_y;
    }
    x += 1;
    ddF_x += 2;
    f += ddF_x;
    pixel(self, xm + x, ym + y, color);
    pixel(self, xm - x, ym + y, color);
    pixel(self, xm + x, ym - y, color);
    pixel(self, xm - x, ym - y, color);
    pixel(self, xm + y, ym + x, color);
    pixel(self, xm - y, ym + x, color);
    pixel(self, xm + y, ym - x, color);
    pixel(self, xm - y, ym - x, color);
  }
  return 0;
}

static int st7789_fill_circle(lua_State *L)
{
  st7789_t *self = get_self(L);
  int xm = luaL_checkinteger(L, 2);
  int ym = luaL_checkinteger(L, 3);
  int r = luaL_checkinteger(L, 4);
  uint16_t color = check_color(L, 5);

  int f     = 1 - r;
  int ddF_x = 1;
  int ddF_y = -2 * r;
  int x     = 0;
  int y     = r;

  vline(self, xm, ym - y, 2 * y + 1, color);

  while (x < y) {
    if (f >= 0) {
      y -= 1;
      ddF_y += 2;
      f += ddF_y;
    }
    x += 1;
    ddF_x += 2;
    f += ddF_x;
    vline(self, xm + x, ym - y, 2 * y + 1, color);
    vline(self, xm + y, ym - x, 2 * x + 1, color);
    vline(self, xm - x, ym - y, 2 * y + 1, color);
    vline(self, xm - y, ym - x, 2 * x + 1, color);
  }
  return 0;
}

// Length is in pixels, not bytes
static void blit_buffer(st7789_t *self, const uint8_t* buf, size_t length, uint8_t* dmabuf)
{
  const int nbufs = length / PIXEL_BUFSZ;
  const int rest = length % PIXEL_BUFSZ;

  spi_transaction_t trans = {0};
  trans.tx_buffer = dmabuf;
  trans.length = 16 * PIXEL_BUFSZ;
  trans.rx_buffer = NULL;

  for (int i = 0; i < nbufs; i++) {
    memcpy(dmabuf, buf + (i * PIXEL_BUFSZ * 2), PIXEL_BUFSZ * 2);
    spi_device_transmit(self->device, &trans);
  }
  if (rest) {
    trans.length = 16 * rest;
    trans.rxlength = 0; // This gets reset by the earlier spi_device_transmit call, sigh
    memcpy(dmabuf, buf + (length - rest) * 2, rest * 2);
    spi_device_transmit(self->device, &trans);
  }
}

static int st7789_blit(lua_State *L)
{
  st7789_t *self = get_self(L);
  size_t len;
  const uint8_t* buf = (const uint8_t*)luaL_checklstring(L, 2, &len);
  int x = luaL_checkinteger(L, 3);
  int y = luaL_checkinteger(L, 4);
  int w = luaL_checkinteger(L, 5);
  int h = luaL_checkinteger(L, 6);
  if (len < w * h * 2) {
    return luaL_error(L, "buffer too small for supplied width and height");
  }
  const int stride = w * 2;

  int ix = x;
  int iy = y;
  intersect_with_screen(self, &ix, &iy, &w, &h);
  int bufx = ix - x;
  int bufy = iy - y;
  set_window(self, ix, iy, ix + w - 1, iy + h - 1);

  const int bufsz = MIN(w * h * 2, PIXEL_BUFSZ * 2);
  uint8_t *dmabuf = (uint8_t *)heap_caps_malloc(bufsz, MALLOC_CAP_DMA);
  // TODO error check

  set_dc(self, 1);
  set_cs(self, 0);

  const uint8_t *data_start = buf + (bufy * stride) + (bufx * 2);
  if (w * 2 == stride) {
    // We can blit all in one go (modulo SPI chunking)
    blit_buffer(self, data_start, w * h, dmabuf);
  } else {
    // image is partly offscreen, have to blit row-by-row
    for (int row = 0; row < h; row++) {
      blit_buffer(self, data_start + (row * stride), w, dmabuf);
    }
  }

  set_cs(self, 1);

  heap_caps_free(dmabuf);
  return 0;
}

static int st7789_set_rotation(lua_State *L)
{
  st7789_t *self = get_self(L);
  int rotation = luaL_checkinteger(L, 2);
  luaL_argcheck(L, rotation >= 0 && rotation <= 3, 2, "Bad rotation parameter");
  self->rotation = rotation;
  set_rotation(L, self);
  return 0;
}

static int st7789_set_sleep_mode(lua_State *L)
{
  st7789_t *self = get_self(L);
  int save = lua_toboolean(L, 2);
  if (save) {
    cmd(self, ST7789_SLPIN);
  } else {
    cmd(self, ST7789_SLPOUT);
  }
  return 0;
}

static int st7789_set_backlight(lua_State *L)
{
  st7789_t *self = get_self(L);
  int enable = lua_toboolean(L, 2);
  if (self->backlight_pin != -1) {
    gpio_set_level(self->backlight_pin, enable);
  }
  return 0;
}

static int st7789_on(lua_State *L)
{
  st7789_t *self = get_self(L);
  if (self->backlight_pin != -1) {
    gpio_set_level(self->backlight_pin, 1);
  }
  return 0;
}

static int st7789_off(lua_State *L)
{
  st7789_t *self = get_self(L);
  if (self->backlight_pin != -1) {
    gpio_set_level(self->backlight_pin, 0);
  }
  return 0;
}

LROT_BEGIN(st7789_mt, NULL, LROT_MASK_GC_INDEX)
  LROT_FUNCENTRY(__gc,        st7789_gc)
  LROT_TABENTRY (__index,     st7789_mt)
  LROT_FUNCENTRY(init,        st7789_init)
  LROT_FUNCENTRY(clear,       st7789_clear_screen)
  LROT_FUNCENTRY(line,        st7789_line)
  LROT_FUNCENTRY(fillRect,    st7789_fill_rect)
  LROT_FUNCENTRY(rect,        st7789_draw_rect)
  LROT_FUNCENTRY(pixel,       st7789_draw_pixel)
  LROT_FUNCENTRY(circle,      st7789_draw_circle)
  LROT_FUNCENTRY(fillCircle,  st7789_fill_circle)
  LROT_FUNCENTRY(blit,        st7789_blit)
  LROT_FUNCENTRY(setSleepMode,st7789_set_sleep_mode)
  LROT_FUNCENTRY(on,          st7789_on)
  LROT_FUNCENTRY(off,         st7789_off)
  LROT_FUNCENTRY(setBacklight,st7789_set_backlight)
  LROT_FUNCENTRY(setRotation, st7789_set_rotation)

  // u8g2/ucg compatible APIs
  LROT_FUNCENTRY(clearBuffer, st7789_clear_screen)
  LROT_FUNCENTRY(drawLine,    st7789_line)
  LROT_FUNCENTRY(drawBox,     st7789_fill_rect)
  LROT_FUNCENTRY(drawFrame,   st7789_draw_rect)
  LROT_FUNCENTRY(drawPixel,   st7789_draw_pixel)
  LROT_FUNCENTRY(drawCircle,  st7789_draw_circle)
  LROT_FUNCENTRY(drawDisc,    st7789_fill_circle)
  LROT_FUNCENTRY(setPowerSave,st7789_set_sleep_mode)

LROT_END(st7789_mt, NULL, LROT_MASK_GC_INDEX)

LROT_BEGIN(st7789, NULL, 0)
  LROT_FUNCENTRY(ST7789, st7789_ST7789)
  LROT_FUNCENTRY(color565, st7789_color565)
  LROT_INTENTRY (BLACK, COLOR_BLACK)
  LROT_INTENTRY (BLUE, COLOR_BLUE)
  LROT_INTENTRY (RED, COLOR_RED)
  LROT_INTENTRY (GREEN, COLOR_GREEN)
  LROT_INTENTRY (CYAN, COLOR_CYAN)
  LROT_INTENTRY (MAGENTA, COLOR_MAGENTA)
  LROT_INTENTRY (YELLOW, COLOR_YELLOW)
  LROT_INTENTRY (WHITE, COLOR_WHITE)
  LROT_INTENTRY (PORTRAIT, ROTATE_0)
  LROT_INTENTRY (LANDSCAPE, ROTATE_90)
  LROT_INTENTRY (INVERSE_PORTRAIT, ROTATE_180)
  LROT_INTENTRY (INVERSE_LANDSCAPE, ROTATE_270)
LROT_END(st7789, NULL, 0)

static int luaopen_st7789(lua_State *L)
{
  luaL_rometatable(L, st7789_mt, LROT_TABLEREF(st7789_mt));
  return 0;
}

NODEMCU_MODULE(ST7789, "st7789", st7789, luaopen_st7789);
