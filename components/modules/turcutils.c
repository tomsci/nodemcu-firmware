#include "turcutils.h"

#include "module.h"
#include "lauxlib.h"

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

void turcutils_pushint64(lua_State* L, int64_t val)
{
  int64_t* ud = (int64_t*)lua_newuserdata(L, sizeof(int64_t));
  *ud = val;
  luaL_newmetatable(L, TURCUTILS_INT64_MT);
  lua_setmetatable(L, -2);
}

int64_t turcutils_checkint64(lua_State* L, int idx)
{
  return *(int64_t*)luaL_checkudata(L, idx, TURCUTILS_INT64_MT);
}

static int turcutils_int64(lua_State* L)
{
  int32_t lo = luaL_checkinteger(L, 1);
  int64_t val;
  if (!lua_isnoneornil(L, 2)) {
    int32_t hi = luaL_checkinteger(L, 2);
    // Then lo must be treated as unsigned
    val = (int64_t)(uint32_t)lo + ((int64_t)hi << 32);
  } else {
    val = (int64_t)lo;
  }
  turcutils_pushint64(L, val);
  return 1;
}

static int turcutils_int64_tostring(lua_State* L)
{
  int64_t val = turcutils_checkint64(L, 1);
  char buf[19];
  snprintf(buf, sizeof(buf), "0x%016llX", val);
  lua_pushstring(L, buf);
  return 1;
}

static int turcutils_int64_toints(lua_State* L)
{
  int64_t val = turcutils_checkint64(L, 1);
  lua_pushinteger(L, (lua_Integer)val);
  if ((int64_t)(lua_Integer)val == val) {
    return 1;
  } else {
    lua_pushinteger(L, (lua_Integer)(val >> 32));
    return 2;
  }
}

static void turcutils_int64_getargs(lua_State* L, int64_t* args)
{
  for (int i = 0; i < 2; i++) {
    if (lua_type(L, i+1) == LUA_TNUMBER) {
      args[i] = lua_tointeger(L, i+1);
    } else {
      args[i] = turcutils_checkint64(L, i+1);
    }
  }
}

static int turcutils_int64_add(lua_State* L)
{
  int64_t args[2];
  turcutils_int64_getargs(L, args);
  turcutils_pushint64(L, args[0] + args[1]);
  return 1;
}

static int turcutils_int64_sub(lua_State* L)
{
  int64_t args[2];
  turcutils_int64_getargs(L, args);
  turcutils_pushint64(L, args[0] - args[1]);
  return 1;
}

static int turcutils_int64_le(lua_State* L)
{
  int64_t args[2];
  turcutils_int64_getargs(L, args);
  lua_pushboolean(L, args[0] <= args[1]);
  return 1;
}

static int turcutils_int64_lt(lua_State* L)
{
  int64_t args[2];
  turcutils_int64_getargs(L, args);
  lua_pushboolean(L, args[0] < args[1]);
  return 1;
}

static int turcutils_int64_eq(lua_State* L)
{
  int64_t args[2];
  turcutils_int64_getargs(L, args);
  lua_pushboolean(L, args[0] == args[1]);
  return 1;
}

LROT_BEGIN(turcutils)
  LROT_FUNCENTRY(and32, turcutils_and32)
  LROT_FUNCENTRY(not32, turcutils_not32)
  LROT_FUNCENTRY(int64, turcutils_int64)
LROT_END(turcutils, NULL, 0)

LROT_BEGIN(turcutils_int64_mt)
  LROT_FUNCENTRY(__add, turcutils_int64_add)
  LROT_FUNCENTRY(__sub, turcutils_int64_sub)
  LROT_FUNCENTRY(__lt, turcutils_int64_lt)
  LROT_FUNCENTRY(__le, turcutils_int64_le)
  LROT_FUNCENTRY(__eq, turcutils_int64_eq)
  LROT_FUNCENTRY(toints, turcutils_int64_toints)
  LROT_FUNCENTRY(__tostring, turcutils_int64_tostring)
  LROT_TABENTRY(__index, turcutils_int64_mt)
LROT_END(turcutils_int64_mt, NULL, 0)

static int luaopen_turcutils(lua_State *L)
{
  luaL_rometatable(L, TURCUTILS_INT64_MT, (void *)turcutils_int64_mt_map);
  return 0;
}

NODEMCU_MODULE(TURCUTILS, "turcutils", turcutils, luaopen_turcutils);
