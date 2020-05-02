
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
#include "task/task.h"

#define MAX_PULSES 200
#define IDLE_TIME 200000 // microseconds

#define PULSE_MAX INT32_MAX
#define PULSE_MIN INT32_MIN

// Ideally and theoretically this would be 80 to give us a tick of 1 microsecond
// (80MHz / 80 = 1us) meaning no translation of times to ticks would be
// required. However for some reason it has to be half that, ie 40 not 80. But
// then we need to make the tick 4 times longer so that our maximum time period
// of ~100ms will fit in a 15-bit number of ticks. Hence our ticks are 4us.
#define CLK_DIV 40 * 4
#define US_TO_TICKS(val) ((val) / 4)
#define TICKS_TO_US(val) ((val) * 4)

typedef struct irpwm_data
{
  uint64_t last_time;
  int32_t buf1[MAX_PULSES];
  int32_t buf2[MAX_PULSES];
  int32_t *pulses; // Points to either buf1 or buf2
  uint8_t count;
  uint8_t gpio;
  uint8_t channel; // rmt_channel_t
  uint8_t async_sending;
  int timer_ref;
  int callback_ref;
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_handle_t pm_lock;
#endif
} irpwm_data;


static void push_data_table(lua_State *L)
{
  lua_pushlightuserdata(L, &push_data_table);
  lua_rawget(L, LUA_REGISTRYINDEX);
  if (lua_isnil(L, -1)) {
    lua_pop(L, 1);
    lua_newtable(L);
    lua_pushlightuserdata(L, &push_data_table);
    lua_pushvalue(L, -2); // dup table
    lua_rawset(L, LUA_REGISTRYINDEX);
    // table is now on top of stack
  }
}

// Allocates irpwm_data if it doesn't already exist
static irpwm_data* dataForPin(lua_State* L, int gpio)
{
  push_data_table(L);
  lua_rawgeti(L, -1, gpio);
  if (lua_isnil(L, -1)) {
    lua_pop(L, 1);
    irpwm_data *data = (irpwm_data *)lua_newuserdata(L, sizeof(irpwm_data));
    memset(data, 0, sizeof(irpwm_data));
    data->gpio = gpio;
    data->timer_ref = LUA_REFNIL;
    data->callback_ref = LUA_REFNIL;
    data->pulses = data->buf1;
    lua_rawseti(L, -2, gpio); // pops data
    lua_pop(L, 1); // indextable
    return data;
  } else {
    void *result = lua_touserdata(L, -1);
    lua_pop(L, 2); // result, indextable
    return (irpwm_data *)result;
  }
}

// Returns NULL if not found
static irpwm_data* dataForChannel(lua_State* L, int channel)
{
  push_data_table(L);
  lua_pushnil(L);
  while (lua_next(L, -2)) {
    irpwm_data *data = (irpwm_data *)lua_touserdata(L, -1);
    lua_pop(L, 1); // Removes value
    if (data->channel == channel) {
      lua_pop(L, 2); // Removes key and table
      return data;
    }
    // Otherwise keep iterating
  }
  // If we get here, not found
  lua_pop(L, 1); // data table
  return NULL;
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
    // Stop collecting data but don't overwrite existing
    return;
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
    default: luaL_error(L, "ESP Error %d!", err);
  }
}

static int timer_tick(lua_State *L)
{
  // Note this potentially accesses data while ISR is running, hence the volatile
  const void *ptr = lua_touserdata(L, lua_upvalueindex(1));
  volatile irpwm_data *vdata = (volatile irpwm_data *)ptr;

  if (vdata->count && esp_timer_get_time() > vdata->last_time + IDLE_TIME) {
    // Switch buffers quickly with interrupts disabled
    irpwm_data *data = (irpwm_data *)ptr;
    check_err(L, gpio_intr_disable(data->gpio));
    uint8_t count = data->count;
    int32_t *buf = data->pulses;
    data->pulses = (buf == data->buf1) ? data->buf2 : data->buf1;
    data->count = 0;
    check_err(L, gpio_intr_enable(data->gpio));

    // Now we can examine buf at our leisure
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->callback_ref);
    lua_createtable(L, count, 0);
    for (int i = 0; i < count; i++) {
      lua_pushinteger(L, buf[i]);
      lua_rawseti(L, -2, i + 1);
    }
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
  luaL_argcheck(L, 2, channel >= RMT_CHANNEL_0 && channel < RMT_CHANNEL_MAX, "Bad channel id");


#ifdef CONFIG_PM_ENABLE
  if (!data->pm_lock) {
    check_err(L, esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "irpwm", &data->pm_lock));
  }
#endif

  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = channel;
  config.clk_div = CLK_DIV;
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
    item->duration0 = US_TO_TICKS(first);
  } else {
    item->level0 = 0;
    item->duration0 = US_TO_TICKS(-first);
  }

  if (second > 0) {
    item->level1 = 1;
    item->duration1 = US_TO_TICKS(second);
  } else {
    item->level1 = 0;
    item->duration1 = US_TO_TICKS(-second);
  }
}

static int populate_items(lua_State *L, int idx, rmt_item32_t *items, int max_items)
{
  int nitems = 0;
  for (int i = 0; i < max_items; i++) {
    lua_rawgeti(L, idx, i + 1);
    int first = lua_tointeger(L, -1);
    lua_rawgeti(L, idx, i + 2);
    int second = lua_tointeger(L, -1);
    lua_pop(L, 2);
    if (!first || !second) {
      break;
    }
    set_item(items + nitems, first, second);
    nitems++;
  }
  return nitems;
}


// irpwm.send(gpio, pulses)
static int irpwm_send(lua_State *L)
{
  int gpio = luaL_checkint(L, 1);
  irpwm_data* data = dataForPin(L, gpio);

#ifdef CONFIG_PM_ENABLE
  check_err(L, esp_pm_lock_acquire(data->pm_lock));
#endif

  rmt_item32_t *items = (rmt_item32_t *)data->buf1;
  // I don't know why but the first pulse is always halved so insert some dummy data
  set_item(items, -500, -500);
  int nitems = 1 + populate_items(L, 2, items + 1, MAX_PULSES - 1);

  // ESP_LOGI("irpwm", "APB_CLK frequency = %d", rtc_clk_apb_freq_get());
  esp_err_t err = rmt_write_items((rmt_channel_t)data->channel, items, nitems, true);

#ifdef CONFIG_PM_ENABLE
  esp_pm_lock_release(data->pm_lock);
#endif

  check_err(L, err);
  return 0;
}

static void irpwm_tx_end_task(task_param_t param, task_prio_t prio)
{
  rmt_channel_t channel = (rmt_channel_t)param;
  irpwm_data *data = dataForChannel(lua_getstate(), channel);
  if (data && data->count) {
    // Indicates there's a repeat to send
    rmt_item32_t *items = (rmt_item32_t *)data->buf2;
    esp_err_t err = rmt_write_items(channel, items, data->count, false);
    if (err) {
      ESP_LOGW("irpwm", "Error from repeat rmt_write_items: %d", err);
    }
  } else if (data && data->async_sending) {
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_release(data->pm_lock);
#endif
    data->async_sending = 0;
  }
}

static task_handle_t tx_end_task = 0;

static void irpwm_tx_end(rmt_channel_t channel, void * arg)
{
  // This is called in ISR context so post a task to handle completion
  task_post_low(tx_end_task, channel);
}

// irpwm.sendasync(gpio, pulses, [repeatpulses])
static int irpwm_sendasync(lua_State *L)
{
  int gpio = luaL_checkint(L, 1);
  irpwm_data* data = dataForPin(L, gpio);
  if (data->async_sending) {
    return luaL_error(L, "Cannot send again before the previous send has completed");
  }

#ifdef CONFIG_PM_ENABLE
  check_err(L, esp_pm_lock_acquire(data->pm_lock));
  data->async_sending = 1;
#endif

  // Use buf1 for send pulses, buf2 for repeats
  rmt_item32_t *items = (rmt_item32_t *)data->buf1;
  // I don't know why but the first pulse is always halved so insert some dummy data
  set_item(items, -500, -500);
  int nitems = 1 + populate_items(L, 2, items + 1, MAX_PULSES - 1);

  if (lua_isnoneornil(L, 3)) {
    data->count = 0;
  } else {
    rmt_item32_t *reps = (rmt_item32_t *)data->buf2;
    data->count = populate_items(L, 3, reps, MAX_PULSES);
  }

  if (!tx_end_task) {
    tx_end_task = task_get_id(irpwm_tx_end_task);
  }

  rmt_register_tx_end_callback(irpwm_tx_end, NULL);
  esp_err_t err = rmt_write_items((rmt_channel_t)data->channel, items, nitems, false);

#ifdef CONFIG_PM_ENABLE
  if (err) {
    esp_pm_lock_release(data->pm_lock);
    data->async_sending = 0;
  }
#endif

  check_err(L, err);
  return 0;
}

// irpwm.stopsend(gpio)
static int irpwm_stopsend(lua_State *L)
{
  int gpio = luaL_checkint(L, 1);
  irpwm_data* data = dataForPin(L, gpio);
  data->count = 0; // Prevents any further repeats
  check_err(L, rmt_wait_tx_done(data->channel, portMAX_DELAY));
  if (data->async_sending) {
    // Ie if the irpwm_tx_end_task hasn't run yet, manually release the lock
    // and reset so we're immediately ready to send again
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_release(data->pm_lock);
#endif
    data->async_sending = 0;
  }
  return 0;
}

#endif

// irpwm.listen(gpio, callbackFn)
static int irpwm_listen(lua_State* L)
{
  int gpio = luaL_checkint(L, 1);
  irpwm_data* data = dataForPin(L, gpio);

  if (lua_isnoneornil(L, 2)) {
    if (data->timer_ref != LUA_REFNIL) {
      lua_rawgeti(L, LUA_REGISTRYINDEX, data->timer_ref);
      luaL_callmeta(L, -1, "unregister");
      luaL_unref(L, LUA_REGISTRYINDEX, data->timer_ref);
      data->timer_ref = LUA_REFNIL;
    }
    if (data->callback_ref != LUA_REFNIL) {
      luaL_unref(L, LUA_REGISTRYINDEX, data->callback_ref);
      data->callback_ref = LUA_REFNIL;
    }
    check_err(L, gpio_intr_disable(gpio));
    check_err(L, gpio_isr_handler_remove(gpio));
    return 0;
  }

  lua_settop(L, 2);

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

LROT_BEGIN(irpwm)
  LROT_FUNCENTRY(listen, irpwm_listen)
  LROT_FUNCENTRY(txconfig, irpwm_txconfig)
  LROT_FUNCENTRY(send, irpwm_send)
  LROT_FUNCENTRY(sendasync, irpwm_sendasync)
  LROT_FUNCENTRY(stopsend, irpwm_stopsend)
  LROT_FUNCENTRY(and32, irpwm_and32)
  LROT_FUNCENTRY(not32, irpwm_not32)
LROT_END(irpwm, NULL, 0)

NODEMCU_MODULE(IRPWM, "irpwm", irpwm, NULL);
