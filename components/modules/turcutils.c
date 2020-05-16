#include "module.h"
#include "lauxlib.h"
#include <stdint.h>

// bit.band() can't handle top-bit-set numbers because casting a double whose
// value > INT_MAX to an int results in INT_MAX, and band() is written in terms
// of lua_Integers which are generally 32-bit signed on esp32.
static int turcutils_and32(lua_State* L)
{
  uint32_t x = (uint32_t)luaL_checknumber(L, 1);
  uint32_t y = (uint32_t)luaL_checknumber(L, 2);
  uint32_t result = x & y;
  lua_pushnumber(L, result);
  return 1;
}

static int turcutils_not32(lua_State* L)
{
  uint32_t x = (uint32_t)luaL_checknumber(L, 1);
  uint32_t result = ~x;
  lua_pushnumber(L, result);
  return 1;
}
LROT_BEGIN(turcutils)
  LROT_FUNCENTRY(and32, turcutils_and32)
  LROT_FUNCENTRY(not32, turcutils_not32)
LROT_END(turcutils, NULL, 0)

NODEMCU_MODULE(TURCUTILS, "turcutils", turcutils, NULL);
