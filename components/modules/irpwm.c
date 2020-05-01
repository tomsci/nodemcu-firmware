
#include "module.h"
#include "lauxlib.h"
#include <string.h>

#define USE_RMT

#ifdef USE_RMT
#include <driver/rmt.h>
#endif

#ifdef CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif
#include <soc/rtc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#define MAX_PULSES 200
#define IDLE_TIME 200000 // microseconds

#define PULSE_MAX INT32_MAX
#define PULSE_MIN INT32_MIN

typedef struct irpwm_data
{
  uint64_t last_time;
  // uint64_t last_timer_time;
  int32_t pulses[MAX_PULSES]; // microseconds between falling edges, always < IDLE_TIME
  uint8_t count;
  uint8_t gpio;
  uint8_t channel; // rmt_channel_t
  int timer_ref;
  int callback_ref;
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_handle_t pm_lock;
#endif
} irpwm_data;


static irpwm_data* dataForPin(lua_State* L, int gpio)
{
  // table = registry[dataForPin]
  lua_pushlightuserdata(L, &dataForPin);
  lua_rawget(L, LUA_REGISTRYINDEX);
  if (lua_isnil(L, -1)) {
    lua_pop(L, 1);
    lua_newtable(L);
    lua_pushlightuserdata(L, &dataForPin);
    lua_pushvalue(L, -2); // dup table
    lua_rawset(L, LUA_REGISTRYINDEX);
    // table is now on top of stack
  }
  lua_rawgeti(L, -1, gpio);
  if (lua_isnil(L, -1)) {
    lua_pop(L, 1);
    irpwm_data *data = (irpwm_data *)lua_newuserdata(L, sizeof(irpwm_data));
    memset(data, 0, sizeof(irpwm_data));
    data->gpio = gpio;
    data->timer_ref = LUA_REFNIL;
    data->callback_ref = LUA_REFNIL;
    lua_rawseti(L, -2, gpio); // pops data
    lua_pop(L, 1); // indextable
    return data;
  } else {
    void *result = lua_touserdata(L, -1);
    lua_pop(L, 2); // result, indextable
    return (irpwm_data *)result;
  }
}

static void freeData(lua_State* L, irpwm_data *data)
{
#ifdef CONFIG_PM_ENABLE
  if (data->pm_lock) {
    esp_pm_lock_delete(data->pm_lock);
    data->pm_lock = 0;
  }
#endif
  lua_pushlightuserdata(L, &dataForPin);
  lua_rawget(L, LUA_REGISTRYINDEX);
  lua_pushnil(L);
  lua_rawseti(L, -2, data->gpio);
  lua_pop(L, 1); // table
}

static void irpwm_isr(void *ctx)
{
  irpwm_data *data = (irpwm_data *)ctx;
  if (data->count >= MAX_PULSES) {
    data->count = 0;
  }

  int64_t t = esp_timer_get_time();
  if (data->last_time == 0) {
    data->last_time = t;
  } else {
    int64_t delta = t - data->last_time;
    if (delta > IDLE_TIME) {
      //TODO DONE
      // return;
    }
    if (gpio_get_level(data->gpio) == 0) {
      // Falling edge
      delta = -delta;
    }
    if (delta > PULSE_MAX) {
      delta = PULSE_MAX;
    } else if (delta < PULSE_MIN) {
      delta = PULSE_MIN;
    }
    data->pulses[data->count++] = (int32_t)(delta);
    data->last_time = t;
  }
}

static void check_err(lua_State *L, esp_err_t err)
{
  switch (err)
  {
    case ESP_ERR_INVALID_ARG: luaL_error(L, "invalid argument");
    case ESP_ERR_INVALID_STATE: luaL_error(L, "internal logic error");
    case ESP_OK: break;
    default: luaL_error(L, "Error %d!", err);
  }
}

static int timer_tick(lua_State *L)
{
  // Note this potentially accesses data while ISR is running, hence the volatile
  const void *ptr = lua_touserdata(L, lua_upvalueindex(1));
  volatile irpwm_data *data = (volatile irpwm_data *)ptr;

  if (data->count && esp_timer_get_time() > data->last_time + IDLE_TIME) {
    // TODO some buffer switching cleverness here?
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->callback_ref);

    check_err(L, gpio_intr_disable(data->gpio));
    lua_createtable(L, data->count, 0);
    for (int i = 0; i < data->count; i++) {
      lua_pushinteger(L, data->pulses[i]);
      lua_rawseti(L, -2, i + 1);
    }
    data->count = 0;
    check_err(L, gpio_intr_enable(data->gpio));

    lua_call(L, 1, 0);
  }

  return 0;
}

#ifdef USE_RMT
// static int irpwm_listen_rmt(lua_State *L)
// {
//   int gpio = luaL_checkint(L, 1);
//   lua_settop(L, 2);

//   rmt_config_t config;
//   config.rmt_mode = RMT_MODE_RX;
//   config.channel = RMT_CHANNEL_0;
//   config.clk_div = 80; // ie tick is 80Mhz/80 = 1us which is nice and tidy
//   config.gpio_num = gpio;
//   config.mem_block_num = 1;
//   config.rx_config.filter_en = true;
//   config.rx_config.filter_ticks_thresh = 100;
//   config.rx_config.idle_threshold = 9500;
//   check_err(L, rmt_config(&config));
//   check_err(L, rmt_driver_install(config.channel, 1000, 0));
//   return 0;
// }

static int irpwm_txconfig(lua_State *L)
{
  int gpio = luaL_checkint(L, 1);
  irpwm_data* data = dataForPin(L, gpio);

  if (lua_isnoneornil(L, 2)) {
    freeData(L, data);
    return 0;
  }
  int channel = luaL_checkint(L, 2);

#ifdef CONFIG_PM_ENABLE
  if (!data->pm_lock) {
    check_err(L, esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "irpwm", &data->pm_lock));
  }
#endif

  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = channel;
  config.clk_div = 40; // 80; // ie tick is 80Mhz/80 = 1us which implifies ticks vs microseconds nicely
  config.gpio_num = gpio;
  config.mem_block_num = 1;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_duty_percent = 50;
  config.tx_config.carrier_freq_hz = 38000;
  config.tx_config.carrier_level = 1;
  config.tx_config.carrier_en = true;
  config.tx_config.idle_level = 0;
  config.tx_config.idle_output_en = true;

  // ESP_LOGI("irpwm", "APB_CLK frequency = %d", rtc_clk_apb_freq_get());

  check_err(L, rmt_config(&config));
  data->channel = config.channel;
  check_err(L, rmt_driver_install(config.channel, 0, 0));
  return 0;
}

static inline void set_item(rmt_item32_t *item, int first, int second)
{
  if (first > 0) {
    item->level0 = 1;
    item->duration0 = first;
  } else {
    item->level0 = 0;
    item->duration0 = -first;
  }

  if (second > 0) {
    item->level1 = 1;
    item->duration1 = second;
  } else {
    item->level1 = 0;
    item->duration1 = -second;
  }
}

// irpwm.send(gpio, pulses)
static int irpwm_send(lua_State *L)
{
  int gpio = luaL_checkint(L, 1);
  irpwm_data* data = dataForPin(L, gpio);

#ifdef CONFIG_PM_ENABLE
  check_err(L, esp_pm_lock_acquire(data->pm_lock));
#endif

  rmt_item32_t *items = (rmt_item32_t *)data->pulses;
  int nitems = 0;
  for (int i = 0; i < MAX_PULSES/2; i++) {
    lua_rawgeti(L, 2, i + 1);
    int first = lua_tointeger(L, -1);
    lua_rawgeti(L, 2, i + 2);
    int second = lua_tointeger(L, -1);
    lua_pop(L, 2);
    if (!first || !second) {
      break;
    }
    set_item(items + nitems++, first, second);
  }

  // ESP_LOGI("irpwm", "APB_CLK frequency = %d", rtc_clk_apb_freq_get());
  esp_err_t err = rmt_write_items((rmt_channel_t)data->channel, items, nitems, true);

#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_release(data->pm_lock);
#endif

  check_err(L, err);
  return 0;
}

#endif

// irpwm.listen(gpio, callbackFn)
static int irpwm_listen(lua_State* L)
{
  int gpio = luaL_checkint(L, 1);
  lua_settop(L, 2);

  irpwm_data* data = dataForPin(L, gpio);

  check_err(L, gpio_intr_disable(gpio));

  data->gpio = gpio;
  data->count = 0;
  data->last_time = 0;

  check_err(L, gpio_set_intr_type(gpio, GPIO_INTR_ANYEDGE));
  check_err(L, gpio_isr_handler_add(gpio, irpwm_isr, (void *)data));
  check_err(L, gpio_intr_enable(gpio));

  lua_getglobal(L, "tmr"); // stack position 3
  lua_getfield(L, -1, "create");
  lua_call(L, 0, 1);

  // timer obj now at top
  lua_getfield(L, -1, "alarm");
  lua_pushvalue(L, -2); // timer obj
  lua_pushinteger(L, 20); // interval_ms. Ugh, 10ms is the minimum granularity for tmr due to it being the freertos tick rate
  lua_getfield(L, 3, "ALARM_AUTO"); // mode

  lua_pushlightuserdata(L, data);
  lua_pushcclosure(L, timer_tick, 1);

  lua_call(L, 4, 1);
  if (!lua_toboolean(L, -1)) {
    return luaL_error(L, "irpwm: Failed to create timer!");
  }
  lua_pop(L, 1); // timer obj back on top
  data->timer_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  lua_pop(L, 1); // tmr
  data->callback_ref = luaL_ref(L, LUA_REGISTRYINDEX);

  return 0;
}

// bit.band() can't handle top-bit-set numbers because casting a double whose
// value > INT_MAX to an int results in INT_MAX, and band() is written in terms
// of lua_Integers which are generally 32-bit signed on esp32.
static int irpwm_and32(lua_State* L)
{
  uint32_t x = (uint32_t)luaL_checknumber(L, 1);
  uint32_t y = (uint32_t)luaL_checknumber(L, 2);
  uint32_t result = x & y;
  lua_pushnumber(L, result);
  return 1;
}

static int irpwm_not32(lua_State* L)
{
  uint32_t x = (uint32_t)luaL_checknumber(L, 1);
  uint32_t result = ~x;
  lua_pushnumber(L, result);
  return 1;
}

// This is a hack, needs to be replaced by an idle timer
static int irpwm_gather(lua_State* L)
{
  int gpio = luaL_checkint(L, 1);
  check_err(L, gpio_intr_disable(gpio));
  // irpwm_data* data = &g_data; // TODO
  irpwm_data* data = dataForPin(L, gpio);
  lua_createtable(L, data->count, 0);
  for (int i = 0; i < data->count; i++) {
    lua_pushinteger(L, data->pulses[i]);
    lua_rawseti(L, -2, i + 1);
  }
  data->count = 0;
  check_err(L, gpio_intr_enable(gpio));
  return 1;
}

LROT_BEGIN(irpwm)
  LROT_FUNCENTRY(listen, irpwm_listen)
  LROT_FUNCENTRY(gather, irpwm_gather)
  LROT_FUNCENTRY(txconfig, irpwm_txconfig)
  LROT_FUNCENTRY(send, irpwm_send)
  LROT_FUNCENTRY(and32, irpwm_and32)
  LROT_FUNCENTRY(not32, irpwm_not32)
LROT_END(irpwm, NULL, 0)

NODEMCU_MODULE(IRPWM, "irpwm", irpwm, NULL);
