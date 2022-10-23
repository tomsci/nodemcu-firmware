#include "module.h"
#include "lauxlib.h"
#include "lodepng.h"

typedef enum {
  FMT_RGB_565, // 16-bit
  FMT_RGB, // 24-bit
  FMT_RGBA // 32-bit
} OutputFormat;

static void convert_888_565(unsigned char* buf, const unsigned w, const unsigned h)
{
  // in-place convert RGB 888 to 565
  unsigned char* outptr = buf;
  const int imax = w * 3;
  for (int y = 0; y < h; y++) {
    unsigned char* rowptr = buf + (y * w * 3);
    for (int i = 0; i < imax; i += 3) {
      uint8_t r = rowptr[i];
      uint8_t g = rowptr[i+1];
      uint8_t b = rowptr[i+2];
      *outptr++ = (r & 0xF8) | (g >> 5);
      *outptr++ = (g << 5) | ((b & 0xF8) >> 3);
    }
  }
}

static void parse_format_arg(lua_State *L, OutputFormat* format, LodePNGColorType* colortype)
{
  int formatint = luaL_checkint(L, 2);
  switch (formatint) {
    case FMT_RGB_565:
      *colortype = LCT_RGB;
      break;
    case FMT_RGB:
      *colortype = LCT_RGB;
      break;
    case FMT_RGBA:
      *colortype = LCT_RGBA;
      break;
    default:
      luaL_error(L, "Bad output format");
  }

  // If we reach here formatint is valid
  *format = (OutputFormat)formatint;
}

static int push_result(lua_State* L, unsigned err, OutputFormat format, unsigned w, unsigned h, unsigned char* buf)
{
  if (err) {
    lua_pushnil(L);
    lua_pushstring(L, lodepng_error_text(err));
    return 2;
  }

  size_t sz;
  if (format == FMT_RGB_565) {
    convert_888_565(buf, w, h);
    sz = w * h * 2;
  } else if (format == FMT_RGB) {
    sz = w * h * 3;
  } else {
    sz = w * h * 4;
  }

  lua_pushlstring(L, (char*)buf, sz);
  free(buf);
  lua_pushinteger(L, (lua_Integer)w);
  lua_pushinteger(L, (lua_Integer)h);
  return 3;
}

static int decode(lua_State *L)
{
  size_t inbuf_sz;
  const unsigned char *inbuf = (const unsigned char *)luaL_checklstring(L, 1, &inbuf_sz);
  OutputFormat format;
  LodePNGColorType colortype;
  parse_format_arg(L, &format, &colortype);
  unsigned char* outbuf;
  unsigned w, h;
  unsigned err = lodepng_decode_memory(&outbuf, &w, &h, inbuf, inbuf_sz, colortype, 8);
  return push_result(L, err, format, w, h, outbuf);
}

static int decode_file(lua_State *L)
{
  const char *filename = luaL_checkstring(L, 1);
  OutputFormat format;
  LodePNGColorType colortype;
  parse_format_arg(L, &format, &colortype);

  unsigned char* buf = NULL;
  unsigned w, h;
  unsigned err = lodepng_decode_file(&buf, &w, &h, filename, colortype, 8);
  return push_result(L, err, format, w, h, buf);
}

LROT_BEGIN(lodepng, NULL, 0)
  LROT_FUNCENTRY(decode, decode)
  LROT_FUNCENTRY(decode_file, decode_file)
  LROT_INTENTRY(RGB, FMT_RGB)
  LROT_INTENTRY(RGB_565, FMT_RGB_565)
  LROT_INTENTRY(RGBA, FMT_RGBA)
LROT_END(lodepng, NULL, 0)

NODEMCU_MODULE(LODEPNG, "lodepng", lodepng, NULL);
