# AnalogIn

----

**At the time of writting (29-07-2014) this class has not been implemented yet**

----

Class that takes care of **ADC** (Analogic to Digital Conversion). Defines and implements methods for **voltage measurement** of **analog signals** in **linux-based** systems.

This class is divided into two files, **header** (`AnalogIn.h`) and **source code** (`AnalogIn.cpp`).

###AnalogIn.h


Link to the code: [AnalogIn.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/AnalogIn.h).

`Linux::AnalogIn`class defines the methods inherited from the [AP_HAL::AnalogIn](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/AnalogIn.h) abstract class. The `Linux::LinuxAnalogSource` class is also defined in this header.

```cpp

#ifndef __AP_HAL_LINUX_ANALOGIN_H__
#define __AP_HAL_LINUX_ANALOGIN_H__

#include <AP_HAL_Linux.h>

class Linux::LinuxAnalogSource : public AP_HAL::AnalogSource {
public:
```
Defines the `LinuxAnalogSource` class (class of [AP_HAL::AnalogSource](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/AnalogIn.h))

```cpp
    LinuxAnalogSource(float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
```
`void set_stop_pin(uint8_t p);`: optionally allows setting a pin that stops the device from reading. This is needed for sonar devices where you have more than one sonar, and you want to stop them interfering with each other. It assumes that if held low the device is stopped, if held high the device starts reading.


---

*Note*: uint8_t, uint16_t, uint32_t, uint64_t are integer type with a width of exactly 8, 16, 32, or 64 bits.Are part of `<cstdint> (stdint.h)`.This type is imported when including `AP_HAL.h`

---

```cpp
    void set_settle_time(uint16_t settle_time_ms);
```
`virtual void set_settle_time(uint16_t settle_time_ms);`:
optionally allow a settle period in milliseconds. This is only used if a stop pin is set. If the settle period is non-zero then the analog input code will wait to get a reading for that number of milliseconds. Note that this will slow down the reading of analog inputs.

```cpp
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }
private:
    float _v;
};
...
```


The float returned is a `voltage` value between 0.0-5.0 V.

```
...
class Linux::LinuxAnalogIn : public AP_HAL::AnalogIn {
public:
    LinuxAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);

    // we don't yet know how to get the board voltage
    float board_voltage(void) { return 0.0f; }

};
#endif // __AP_HAL_LINUX_ANALOGIN_H__
```
This piece of code:

- Defines the `LinuxAnalogIn` class of `AP_HAL::AnalogIN`.


- Declares the init() method with a pointer as argument.


- The pointer is initialized to a channel.


- `board_voltage` should read the board voltage. It is not implemented yet.

###AnalogIn.cpp


[AnalogIn.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/AnalogIn.cpp) implements the methods defined  in `AnalogIn.h` for read/write analog signals.  **Not implemented yet**.

```cpp
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn.h"

using namespace Linux;
...
```

Inside a **namespace** you include all functions appropriate that fulfill a certain goal. You can then refer to the functions that are part of a namespace by prefixing the function with the namespace name followed by the scope operator `::` .

```cpp

...
LinuxAnalogSource::LinuxAnalogSource(float v) :
    _v(v)
{}

float LinuxAnalogSource::read_average() {
    return _v;
}

float LinuxAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float LinuxAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float LinuxAnalogSource::read_latest() {
    return _v;
}

void LinuxAnalogSource::set_pin(uint8_t p)
{}

void LinuxAnalogSource::set_stop_pin(uint8_t p)
{}

void LinuxAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

LinuxAnalogIn::LinuxAnalogIn()
{}

void LinuxAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* LinuxAnalogIn::channel(int16_t n) {
    return new LinuxAnalogSource(1.11);
}

#endif // CONFIG_HAL_BOARD
```
