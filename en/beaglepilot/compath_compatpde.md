# Compat.h /compat.pde


This two files include funtions that will be very used in other files:

Link to the code:
 [Compat.h](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/compat.h)

 ```cpp

#ifndef __COMPAT_H__
#define __COMPAT_H__

#define HIGH 1
#define LOW 0

/* Forward declarations to avoid broken auto-prototyper (coughs on '::'?) */
static void run_cli(AP_HAL::UARTDriver *port);

#endif // __COMPAT_H__
```
Defines `COMPAT_H`. Also defines `HIGH` and `LOW` values to 1 and 0 (as binary values).
Also defines `run_cli` function.

*Note*: To pass from analogic to digital values, Hihg stream will cause a ON or 1 value and low stream will cause a OFF value or 0.


Link to the code: [Compat.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/compat.pde)

```cpp


static void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static uint32_t millis()
{
    return hal.scheduler->millis();
}

static uint32_t micros()
{
    return hal.scheduler->micros();
}
```
This code include some funtions from the `AP_HAL/Scheduler.h` like `delay`or `millis`, you can find them [here](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h#L14).
The `millis`function pass to ms and the `micros`function to microseconds.
