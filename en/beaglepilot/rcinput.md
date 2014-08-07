# RCInput


Class that takes care of **RC** (Radio Control) input signals. This signals are usually either PPM (PPM-SUM), PWM, D-BUS or other sort of RC starndart. This class implements a layer to abstract **linux-based** systems.

---

This class is still a work in progress.

---
This class is divided into two files, **header** (`RCInput.h`) and **source code** (`RCInput.cpp`).

###RCInput.h


Link to the code:[RCInput.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/RCInput.h)

`Linux::LinuxRCInput`defines the methods inherited from the [AP_HAL::RCInput](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/RCInput.h) abstract class, for handling **Radio Control (RC)** input signals processing like telemetry.

```cpp

#ifndef __AP_HAL_LINUX_RCINPUT_H__
#define __AP_HAL_LINUX_RCINPUT_H__

#include <AP_HAL_Linux.h>

class Linux::LinuxRCInput : public AP_HAL::RCInput {
public:
    LinuxRCInput();
    void init(void* machtnichts);
    bool new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

 private:
    bool new_rc_input;

    /* override state */
    uint16_t _override[8];
};

#endif // __AP_HAL_LINUX_RCINPUT_H__
```
- Defines the `LinuxRCInput`class that inherits from [AP_HAL::RCInput](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/RCInput.h).


- The init method is defined. Call init from the platform hal instance init, so that both the type of
the RCInput implementation and init argument are known to the programmer.


- There are some method defined for later implementation in `RCInput.cpp`.


###RCInput.cpp


[RCInput.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/RCInput.cpp) implements the methods defined  in `RCInput.h` header file for read and override the RC input channels.

```cpp
...
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCInput.h"

using namespace Linux;
LinuxRCInput::LinuxRCInput() :
new_rc_input(false)
{}

void LinuxRCInput::init(void* machtnichts)
{}


...
```

- `AP_HAL.h`and `RCInput.h` are imported and the board is defined.


- Note that `new_rc_input` is initialized to False.

```cpp
...
bool LinuxRCInput::new_input()
{
    return new_rc_input;
}
...
```
- This method  returns true if there has been new input since the last `read()` call.

```cpp
...

uint8_t LinuxRCInput::num_channels()
{
    return 8;
}
...
```

- This slice of code returns the number of valid channels in the last read.


```cpp
...
uint16_t LinuxRCInput::read(uint8_t ch)
{
    new_rc_input = false;
    if (_override[ch]) {
        return _override[ch];
    }
    if (ch == 2) {
        // force low throttle for now
        return 900;
    }
    return 1500;
}
...
```
- `read()` reads a single channel at a time.

```cpp
...
uint8_t LinuxRCInput::read(uint16_t* periods, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
	periods[i] = read(i);
    }
    return 8;
}
...

```



- This method  reads an array of channels, return the valid count. Note that the name is th e same as the previous one, that means that the one we select depend on the arguments.


```cpp
...
bool LinuxRCInput::set_overrides(int16_t *overrides, uint8_t len)
{
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool LinuxRCInput::set_override(uint8_t channel, int16_t override)
{
    if (override < 0) return false; /* -1: no change. */
    if (channel < 8) {
        _override[channel] = override;
        if (override != 0) {
            new_rc_input = true;
            return true;
        }
    }
    return false;
}

void LinuxRCInput::clear_overrides()
{
    for (uint8_t i = 0; i < 8; i++) {
	_override[i] = 0;
    }
}
```
- This method do the following:
 + The first `set_overrides`: array starts at ch 0, for len channels.
 + The second `set_override`: set just a specific channel .
 + The last `set_override`: equivelant to setting all overrides to 0 .

