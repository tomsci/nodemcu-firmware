# node Module
| Since  | Origin / Contributor  | Maintainer  | Source  |
| :----- | :-------------------- | :---------- | :------ |
| 2014-12-22 | [Zeroday](https://github.com/funshine) | [Zeroday](https://github.com/funshine) | [node.c](../../components/modules/node.c)|

The node module provides access to system-level features such as sleep, restart and various info and IDs.

## node.bootreason()

Returns the boot reason and extended reset info.

The first value returned is the raw code, not the new "reset info" code which was introduced in recent SDKs. Values are:

  - 1, power-on
  - 2, reset (software?)
  - 3, hardware reset via reset pin
  - 4, WDT reset (watchdog timeout)

The second value returned is the extended reset cause. Values are:

  - 0, power-on
  - 1, hardware watchdog reset
  - 2, exception reset
  - 3, software watchdog reset
  - 4, software restart
  - 5, wake from deep sleep
  - 6, external reset

In general, the extended reset cause supercedes the raw code. The raw code is kept for backwards compatibility only. For new applications it is highly recommended to use the extended reset cause instead.

In case of extended reset cause 3 (exception reset), additional values are returned containing the crash information. These are, in order, EXCCAUSE, EPC1, EPC2, EPC3, EXCVADDR, and DEPC.

On the esp32 if the extended reset cause is 5 (wake from deep sleep) and the wakeup was caused by a GPIO, then two additional values are returned which are a bitmask of the GPIO(s) that triggered the wakeup. The third value represents GPIOs 0-31 and the fourth value is for GPIOs 32 and above. For example the result for a wake from sleep caused by GPIO 31 would be `3, 5, 0x80000000, 0` (on a build with `LUA_NUMBER_INTEGRAL` defined. Use `bit.isset(val, 31)` to check for GPIO 31 being set in a way which works regardless of whether it is an integral build or not) A wakeup caused by GPIO 33 would return `3, 5, 0, 2`.

On the esp32 if the extended reset cause is 5 (wake from deep sleep) and the wakeup was caused by a touch pad event, a bitmask of the pin which triggered the wakeup is returned in results 3 and 4, in the same way as GPIO-triggered wakeups are returned (see above). Note this is the _gpio_, not the touch pad id, for example a wakeup cause by a touch on T0 (GPIO 4) would return `3, 5, 16, 0` (because `16` is `1 << 4`). Note that configuring the touch module clears the boot reason, meaning the underlying SDK API will assert if you call this function after configuring touch - therefore if you are intending to use a touch pin for wakeup, you must ensure to call `node.bootreason()` _before_ configuring the touch module.

#### Syntax
`node.bootreason()`

#### Parameters
none

#### Returns
`rawcode, reason [, exccause, epc1, epc2, epc3, excvaddr, depc ]`

#### Example
```lua
_, reset_reason = node.bootreason()
if reset_reason == 0 then print("Power UP!") end
```

## node.chipid()

Returns the ESP chip ID.

#### Syntax
`node.chipid()`

#### Parameters
none

#### Returns
chip ID (string)

Note that due to the chip id being a much larger value on the ESP32, it is
reported as a string now. E.g. `"0x1818fe346a88"`.

## node.compile()

Compiles a Lua text file into Lua bytecode, and saves it as .lc file.

#### Syntax
`node.compile("file.lua")`

#### Parameters
`filename` name of Lua text file

#### Returns
`nil`

#### Example
```lua
file.open("hello.lua","w+")
file.writeline([[print("hello nodemcu")]])
file.writeline([[print(node.heap())]])
file.close()

node.compile("hello.lua")
dofile("hello.lua")
dofile("hello.lc")
```

## node.dsleep()

Enters deep sleep mode. When the processor wakes back up depends on the supplied `options`. Unlike light sleep, waking from deep sleep restarts the processor, therefore this API never returns. Wake up can be triggered by a time period, or when a GPIO (or GPIOs) change level, or when a touchpad event occurs. If multiple different wakeup sources are specified, the processor will wake when any of them occur. Use [`node.bootreason()`](#nodebootreason) to determine what caused the wakeup.

Only RTC GPIO pins can be used to trigger wake from deep sleep, and they should be configured as inputs prior to calling this API. On the ESP32, the RTC pins are GPIOs 0, 2, 4, 12-15, 25-27, and 32-39. An error will be raised if any of the specified pins are not RTC-capable. If multiple pins are specified and `level=1` (which is the default), the wakeup will occur if *any* of the pins are high. If `level=0` then the wakeup will only occur if *all* the specified pins are low.

For compatibility, a number parameter `usecs` can be supplied instead of an `options` table, which is equivalent to `node.dsleep({us = usecs})`.

#### Syntax
`node.dsleep(usecs)` or `node.dsleep(options)`

#### Parameters

- `options`, a table containing some of:
    - `secs`, a number of seconds to sleep. This permits longer sleep periods compared to using the `us` parameter.
    - `us`, a number of microseconds to sleep. If both `secs` and `us` are provided, the values are combined.
    - `gpio`, a single GPIO number or a list of GPIOs. These pins must all be RTC-capable otherwise an error is raised.
    - `level`. Whether to trigger when *any* of the GPIOs are high (`level=1`, which is the default if not specified), or when *all* the GPIOs are low (`level=0`).
    - `isolate`. A list of GPIOs to isolate. Isolating a GPIO disables input, output, pullup, pulldown, and enables hold feature for an RTC IO. Use this function if an RTC IO needs to be disconnected from internal circuits in deep sleep, to minimize leakage current.
    - `pull`, boolean, whether to keep powering previously-configured internal pullup/pulldown resistors. Default is `false` if not specified.
    - `touch`, boolean, whether to trigger wakeup from any previously-configured touchpads. Default is `false` if not specified.

If an empty options table is specified, ie no wakeup sources, then the chip will sleep forever with no way to wake it (except for power cycling or triggering the reset pin/button).

#### Returns
Does not return.

#### Example
```lua
-- sleep 10 seconds then reboot
node.dsleep({ secs = 10 })

-- sleep until 10 seconds have elapsed or either of GPIO 13 or 15 becomes high
node.dsleep({ secs = 10, gpio = { 13, 15 } })

-- Sleep forever until GPIO 13 is low, and keep its pullup powered
node.dsleep({ gpio = 13, level = 0, pull = true })
```

## node.flashid()

Returns the flash chip ID.

#### Syntax
`node.flashid()`

#### Parameters
none

#### Returns
flash ID (number)

## node.heap()

Returns the current available heap size in bytes. Note that due to fragmentation, actual allocations of this size may not be possible.

#### Syntax
`node.heap()`

#### Parameters
none

#### Returns
system heap size left in bytes (number)

## node.info()

Returns NodeMCU version, chipid, flashid, flash size, flash mode, flash speed.

#### Syntax
`node.info()`

#### Parameters
none

#### Returns
 - `majorVer` (number)
 - `minorVer` (number)
 - `devVer` (number)
 - `chipid` (number)
 - `flashid` (number)
 - `flashsize` (number)
 - `flashmode` (number)
 - `flashspeed` (number)

#### Example
```lua
majorVer, minorVer, devVer, chipid, flashid, flashsize, flashmode, flashspeed = node.info()
print("NodeMCU "..majorVer.."."..minorVer.."."..devVer)
```

## node.input()

Submits a string to the Lua interpreter. Similar to `pcall(loadstring(str))`, but without the single-line limitation.

!!! note "Note:"

    This function only has an effect when invoked from a callback. Using it directly on the console **does not work**.

#### Syntax
`node.input(str)`

#### Parameters
`str` Lua chunk

#### Returns
`nil`

#### Example
```lua
sk:on("receive", function(conn, payload) node.input(payload) end)
```

#### See also
[`node.output()`](#nodeoutput)

## node.key() --deprecated

Defines action to take on button press (on the old devkit 0.9), button connected to GPIO 16.

This function is only available if the firmware was compiled with DEVKIT_VERSION_0_9 defined.

#### Syntax
`node.key(type, function())`

#### Parameters
  - `type`: type is either string "long" or "short". long: press the key for 3 seconds, short: press shortly(less than 3 seconds)
  - `function`: user defined function which is called when key is pressed. If nil, remove the user defined function. Default function: long: change LED blinking rate,  short: reset chip

#### Returns
`nil`

#### Example
```lua
node.key("long", function() print('hello world') end)
```
#### See also
[`node.led()`](#nodeled-deprecated)

## node.led() --deprecated

Sets the on/off time for the LED (on the old devkit 0.9), with the LED connected to GPIO16, multiplexed with [`node.key()`](#nodekey-deprecated).

This function is only available if the firmware was compiled with DEVKIT_VERSION_0_9 defined.

#### Syntax
`node.led(low, high)`

#### Parameters
  - `low` LED off time, LED keeps on when low=0. Unit: milliseconds, time resolution: 80~100ms
  - `high` LED on time. Unit: milliseconds, time resolution: 80~100ms

#### Returns
`nil`

#### Example
```lua
-- turn led on forever.
node.led(0)
```

#### See also
[`node.key()`](#nodekey-deprecated)

## node.output()

Redirects the Lua interpreter output to a callback function. Optionally also prints it to the serial console.

!!! note "Note:"

    Do **not** attempt to `print()` or otherwise induce the Lua interpreter to produce output from within the callback function. Doing so results in infinite recursion, and leads to a watchdog-triggered restart.

#### Syntax
`node.output(function(str), serial_output)`

#### Parameters
  - `output_fn(str)` a function accept every output as str, and can send the output to a socket (or maybe a file). `nil` to unregister the previous function.
  - `serial_output` 1 output also sent out the serial port. 0: no serial output. Defaults to 1 if not specified.

#### Returns
`nil`

#### Example
```lua
function tonet(str)
  sk:send(str)
end
node.output(tonet, 1)  -- serial also get the lua output.
```

```lua
-- a simple telnet server
s=net.createServer(net.TCP)
s:listen(2323,function(c)
   con_std = c
   function s_output(str)
      if(con_std~=nil)
         then con_std:send(str)
      end
   end
   node.output(s_output, 0)   -- re-direct output to function s_ouput.
   c:on("receive",function(c,l)
      node.input(l)           -- works like pcall(loadstring(l)) but support multiple separate line
   end)
   c:on("disconnection",function(c)
      con_std = nil
      node.output(nil)        -- un-regist the redirect output function, output goes to serial
   end)
end)
```

```lua
-- disable all output completely
node.output(nil, 0)
```
#### See also
[`node.input()`](#nodeinput)

## node.readvdd33() --deprecated
Moved to [`adc.readvdd33()`](adc/#adcreadvdd33).

## node.restart()

Restarts the chip.

#### Syntax
`node.restart()`

#### Parameters
none

#### Returns
`nil`

## node.restore()

Restores system configuration to defaults using the SDK function `system_restore()`, which doesn't document precisely what it erases/restores.

#### Syntax
`node.restore()`

#### Parameters
none

#### Returns
`nil`

#### Example
```lua
node.restore()
node.restart() -- ensure the restored settings take effect
```

## node.setcpufreq()

Change the working CPU Frequency.

#### Syntax
`node.setcpufreq(speed)`

#### Parameters
`speed` constant 'node.CPU80MHZ' or 'node.CPU160MHZ'

#### Returns
target CPU frequency (number)

#### Example
```lua
node.setcpufreq(node.CPU80MHZ)
```


## node.sleep()

Enters light sleep mode, which saves power without losing state. The state of the CPU and peripherals is preserved during light sleep and is resumed once the processor wakes up. When the processor wakes back up depends on the supplied `options`. Wake up from light sleep can be triggered by a time period, or when a GPIO (or GPIOs) change level, when a touchpad event occurs, when data is received on a UART, or by the ULP (ultra low power processor, generally not used by NodeMCU). If multiple different wakeup sources are specified, the processor will wake when any of them occur. The return value of the function can be used to determine which source caused the wakeup. The function does not return until a wakeup occurs (and therefore may not return at all if a wakeup trigger never occurs).

UART buffers are not flushed on entering light sleep, rather they are suspended and resumed on wakeup, meaning that some data written before entering light sleep may not be output over the UART until after wakeup.

Unlike with the `dsleep()` API, _any_ GPIO (not just the RTC-capable pins) may be used to trigger wakeup from light sleep. To configure which GPIOs should trigger wakeup, and under what circumstances, call `gpio.wakeup()` prior to calling `node.sleep()`.

Wakeup from Light sleep can also be triggered by incoming data on UART0 or UART1, (but not UART2) by passing in `uart = 0` or `uart = 1` (or `uart = {0, 1}` to wake on either). Note that the byte(s) which trigger the wakeup are consumed in the process, and will therefore not be seen by the UART after wakeup. Before using uart wakeup for the first time, you must call [`uart.wakeup()`](uart/#uartwakeup) to configure what data should trigger wakeup.

WiFi and Bluetooth must be switched off before entering light sleep, otherwise an error will be thrown. It is not supported to enable gpio wakeups at the same time as either touch or ULP.

#### Syntax
`node.sleep(options)`

#### Parameters
- `options`, a table containing some of:
    - `secs`, a number of seconds to sleep. This permits longer sleep periods compared to using the `us` parameter.
    - `us`, a number of microseconds to sleep. If both `secs` and `us` are provided, the values are combined.
    - `gpio`, a boolean, whether to allow wakeup by GPIOs. Default is `false` if not specified.
    - `touch`, boolean, whether to trigger wakeup from any previously-configured touchpads. Default is `false` if not specified.
    - `uart`, an integer or list of integers. Which UARTs should trigger wakeup.
    - `ulp`, a boolean, whether to allow the ULP to trigger wakeup.

If an empty options table is specified, ie no wakeup sources, then the chip will sleep forever with no way to wake it (except for power cycling or triggering the reset pin/button).

#### Returns
One of `node.wakeup.GPIO`, `node.wakeup.TIMER`, `node.wakeup.TOUCHPAD`, `node.wakeup.UART`, `node.wakeup.ULP`, depending on what triggered the wakeup.

If the wakeup was trigged by a touchpad, the pad id which triggered the wakeup will be returned as a second value (ie `node.wakeup.TOUCHPAD, <padid>`).

#### Example
```lua
-- sleep 10 seconds then continue
node.sleep({ secs = 10 })

-- sleep until 10 seconds have elapsed or either of GPIO 13 or 15 becomes high
gpio.wakeup(13, gpio.INTR_HIGH)
gpio.wakeup(15, gpio.INTR_HIGH)
node.sleep({ secs = 10, gpio = true })

-- Sleep forever until a previously-configured touchpad is touched, or a byte
-- arrives on UART0.
uart.wakeup(0, 3)
reason, pad = node.sleep({ touch = true, uart = 0 })
if reason == node.WAKEUP_TOUCHPAD then
  print(string.format("Pad %d was touched!", pad))
end
```

## node.stripdebug()

Controls the amount of debug information kept during [`node.compile()`](#nodecompile), and allows removal of debug information from already compiled Lua code.

Only recommended for advanced users, the NodeMCU defaults are fine for almost all use cases.

####Syntax
`node.stripdebug([level[, function]])`

#### Parameters
- `level`
	- 1, don't discard debug info
	- 2, discard Local and Upvalue debug info
	- 3, discard Local, Upvalue and line-number debug info
- `function` a compiled function to be stripped per setfenv except 0 is not permitted.

If no arguments are given then the current default setting is returned. If function is omitted, this is the default setting for future compiles. The function argument uses the same rules as for `setfenv()`.

####  Returns
If invoked without arguments, returns the current level settings. Otherwise, `nil` is returned.

#### Example
```lua
node.stripdebug(3)
node.compile('bigstuff.lua')
```

#### See also
[`node.compile()`](#nodecompile)

## node.osprint()

Controls whether the debugging output from the Espressif SDK is printed. Note that this is only available if
the firmware is build with DEVELOPMENT_TOOLS defined.

####Syntax
`node.osprint(enabled)`

#### Parameters
- `enabled` This is either `true` to enable printing, or `false` to disable it. The default is `false`.

#### Returns
Nothing

#### Example
```lua
node.osprint(true)
```

## node.uptime()

Returns the value of the system counter, which counts in microseconds starting at 0 when the device is booted. Because a 32-bit signed integer can only fit 35 minutes' worth of microseconds before wrapping, when using integer-only 32-bit Lua this API returns 2 results, the first being the bottom 31 bits of the counter, and the second being the next 31 bits (ie bits 31 through 62 of the underlying 64-bit counter).

Therefore the first result will wrap back to 0 after about 35 minutes (2^31 microseconds), at which point the second result will increment by one. It is important to consider both values, or otherwise handle the first result wrapping, when for example using this function to [debounce or throttle GPIO input](https://github.com/hackhitchin/esp8266-co-uk/issues/2).

When using floating-point Lua, hundreds of years of microseconds can fit in a single value with no loss of precision. In such configurations, the first result will be the bottom 53 bits of the system counter as a floating-point number, and the second result will be the top 11 bits, which in practice will always be zero unless the system counter exceeds 285 years.

#### Syntax
`node.uptime()`

#### Parameters
none

#### Returns
Two results `lowbits, highbits`. The first is the time in microseconds since boot or the last time the counter wrapped, and the second is the number of times the counter has wrapped. Depending on whether Lua was compiled with floating-point support or not determines when the counter wraps.

#### Example
```lua
print(node.uptime())
print(node.uptime())

lowbits, highbits = node.uptime()
print(lowbits, "microseconds have elapsed since the uptime last wrapped")
print(string.format("Uptime has wrapped %d times", highbits))
```

# node.egc module

## node.egc.setmode()

Sets the Emergency Garbage Collector mode. [The EGC whitepaper](http://www.eluaproject.net/doc/v0.9/en_elua_egc.html)
provides more detailed information on the EGC.

####Syntax
`node.egc.setmode(mode, [param])`

#### Parameters
- `mode`
	- `node.egc.NOT_ACTIVE` EGC inactive, no collection cycle will be forced in low memory situations
	- `node.egc.ON_ALLOC_FAILURE` Try to allocate a new block of memory, and run the garbage collector if the allocation fails. If the allocation fails even after running the garbage collector, the allocator will return with error. 
	- `node.egc.ON_MEM_LIMIT` Run the garbage collector when the memory used by the Lua script goes beyond an upper `limit`. If the upper limit can't be satisfied even after running the garbage collector, the allocator will return with error.
	- `node.egc.ALWAYS` Run the garbage collector before each memory allocation. If the allocation fails even after running the garbage collector, the allocator will return with error. This mode is very efficient with regards to memory savings, but it's also the slowest.
- `level` in the case of `node.egc.ON_MEM_LIMIT`, this specifies the memory limit.
  
#### Returns
`nil`

#### Example

`node.egc.setmode(node.egc.ALWAYS, 4096)  -- This is the default setting at startup.`
`node.egc.setmode(node.egc.ON_ALLOC_FAILURE) -- This is the fastest activeEGC mode.`

# node.task module

## node.task.post()

Enable a Lua callback or task to post another task request. Note that as per the 
example multiple tasks can be posted in any task, but the highest priority is 
always delivered first.

If the task queue is full then a queue full error is raised.  

####Syntax
`node.task.post([task_priority], function)`

#### Parameters
- `task_priority` (optional)
	- `node.task.LOW_PRIORITY` = 0
	- `node.task.MEDIUM_PRIORITY` = 1
	- `node.task.HIGH_PRIORITY` = 2
- `function` a callback function to be executed when the task is run. 

If the priority is omitted then  this defaults  to `node.task.MEDIUM_PRIORITY`

####  Returns
`nil`

#### Example
```lua
for i = node.task.LOW_PRIORITY, node.task.HIGH_PRIORITY do 
  node.task.post(i,function(p2)
    print("priority is "..p2)
  end) 
end      
``` 
prints
```
priority is 2
priority is 1
priority is 0
```

