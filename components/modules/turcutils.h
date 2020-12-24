#ifndef TURCUTILS_H
#define TURCUTILS_H

#include <stdint.h>

typedef struct lua_State lua_State;

#define TURCUTILS_INT64_MT "int64_t"

void turcutils_pushint64(lua_State* L, int64_t val);
int64_t turcutils_checkint64(lua_State* L, int idx);

#endif
