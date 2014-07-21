# AP_HAL

### Overview

[AP_HAL](https://github.com/diydrones/ardupilot/tree/master/libraries/AP_HAL) is hardware abstraction layer for the `ArduPilot` project. The `AP_HAL` consists of a set of headers (`.h`) that define the classes and methods that should be implemmented if ardupilot should run in a new device/architecture. The code contained in this HALs (**Hardware Abstraction Layer**s) is usually quite **low level** and close to the hardware.


### Using the AP_HAL

`AP_HAL`, found in `/libraries/AP_HAL/`, is a module of purely virtual classes: there is no concrete code in the `AP_HAL` module, only interfaces. All code in the `ArduPilot` libraries and example sketches should depend only on the interfaces exposed by `AP_HAL`.

---

**This class is expected to be inherited in order to creat a Hardware Abstraction Layer**

---

The collection of classes in the `AP_HAL` module exist in the `AP_HAL` C++ namespace. The convention is for a program to instantiate a single instance of the `AP_HAL::HAL class`, under a reference to the name **hal**.

This way:

```cpp
#include <AP_HAL.h>
const AP_HAL::HAL& hal = specific_hal_implementation;
```

will create an instance that should be in a single object file. All other object files, including libraries (even those inside an `AP_HAL` implementation, should use the `AP_HAL` interface by declaring an extern reference to hal:

```cpp
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;
```


###AP_HAL Contents

The `AP_HAL module` has the following contents:

```bash

tree ardupilot/libraries/AP_HAL

AP_HAL
├── AnalogIn.h
├── AP_HAL_Boards.h
├── AP_HAL.h
├── AP_HAL_Macros.h
├── AP_HAL_Namespace.h
├── examples
│   ├── AnalogIn
│   │   ├── AnalogIn.pde
│   │   ├── Makefile
│   │   ├── nobuild.txt
│   │   └── nocore.inoflag
│   ├── Printf
│   │   ├── Makefile
│   │   └── Printf.pde
│   └── RCOutput
│       ├── Makefile
│       └── RCOutput.pde
├── GPIO.h
├── HAL.h
├── I2CDriver.h
├── RCInput.h
├── RCOutput.h
├── Scheduler.h
├── Semaphores.h
├── SPIDriver.h
├── Storage.h
├── UARTDriver.cpp
├── UARTDriver.h
├── Util.cpp
├── Util.h
└── utility
    ├── BetterStream.h
    ├── FastDelegate.h
    ├── ftoa_engine.cpp
    ├── ftoa_engine.h
    ├── Print.cpp
    ├── Print.h
    ├── print_vprintf.cpp
    ├── print_vprintf.h
    ├── Stream.h
    ├── utoa_invert.cpp
    └── xtoa_fast.h

5 directories, 37 files
```


#### AP_HAL_Namespace.h
Exposes the C++ [namespace AP_HAL](https://github.com/diydrones/ardupilot/tree/master/libraries/AP_HAL/AP_HAL_Namespace.h). The namespace declaration declares each class by name (not implementation) and some useful `typedefs`.

```cpp
namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;

    class SPIDeviceDriver;
    class SPIDeviceManager;

    class AnalogSource;
    class AnalogIn;
    class Storage;
    class DigitalSource;
    ...
```


#### AP_HAL_Boards.h
[AP_HAL_Boards](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/AP_HAL_Boards.h) has a C preprocesor enumeration of the boards supported by the `AP_HAL`.This list exists so `HAL_BOARD == HAL_BOARD_xxx` preprocessor blocks can be used to exclude HAL boards from the build when appropriate.

``` cpp
...
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#define AP_HAL_BOARD_DRIVER AP_HAL_Linux
#define HAL_BOARD_NAME "Linux"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_OS_POSIX_IO 1
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NONE
#define HAL_BOARD_LOG_DIRECTORY "logs"
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_HIL
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HIL
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
#define HAL_BOARD_LOG_DIRECTORY "/var/APM/logs"
#define HAL_INS_DEFAULT HAL_INS_MPU9250
#define HAL_BARO_DEFAULT HAL_BARO_MS5611_SPI
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#endif

#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
#define AP_HAL_BOARD_DRIVER AP_HAL_Empty
#define HAL_BOARD_NAME "EMPTY"
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_HIL
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HIL
...
```

#### AP_HAL_Macros.h
These **macros** allow the code to build on multiple platforms more easily. It also defines a const expression to avoid issues between c++11 with NuttX and C++10 on other platforms.


``` cpp
#ifndef __AP_HAL_MACROS_H__
#define __AP_HAL_MACROS_H__

/*
  macros to allow code to build on multiple platforms more easily
 */

#ifdef __GNUC__
 #define WARN_IF_UNUSED __attribute__ ((warn_unused_result))
#else
 #define WARN_IF_UNUSED
#endif

// use this to avoid issues between C++11 with NuttX and C++10 on
// other platforms.
#if !(defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L)
# define constexpr const
#endif

#endif // __AP_HAL_MACROS_H__

```

---

**The `AP_HAL` interface classes are each defined in a header file bearing their name.**

---

The following abstractions compound the module `AP_HAL`:


####AP_HAL::HAL
[AP_HAL::HAL](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/AP_HAL.h) is a container for the a complete set of device drivers. It also has a virtual (i.e. overridable) method to handle driver initialization. Each device driver is exposed as a pointer an `AP_HAL` driver class, (e.g. each serial driver is exposed as a public `UARTDriver* uartN`).


```cpp
...
class AP_HAL::HAL {
public:
    HAL(AP_HAL::UARTDriver* _uartA, // console
        AP_HAL::UARTDriver* _uartB, // 1st GPS
        AP_HAL::UARTDriver* _uartC, // telem1
        AP_HAL::UARTDriver* _uartD, // telem2
        AP_HAL::UARTDriver* _uartE, // 2nd GPS
        AP_HAL::I2CDriver* _i2c,
        AP_HAL::SPIDeviceManager* _spi,
        AP_HAL::AnalogIn* _analogin,
        AP_HAL::Storage* _storage,
        AP_HAL::UARTDriver* _console,
        AP_HAL::GPIO* _gpio,
        AP_HAL::RCInput* _rcin,
        AP_HAL::RCOutput* _rcout,
        AP_HAL::Scheduler* _scheduler,
        AP_HAL::Util* _util)
        :
        uartA(_uartA),
        uartB(_uartB),
        uartC(_uartC),
        uartD(_uartD),
        uartE(_uartE),
        i2c(_i2c),
        spi(_spi),
        analogin(_analogin),
        storage(_storage),
        console(_console),
        gpio(_gpio),
        rcin(_rcin),
        rcout(_rcout),
        scheduler(_scheduler),
        util(_util)
    {}

    virtual void init(int argc, char * const argv[]) const = 0;

    AP_HAL::UARTDriver* uartA;
    AP_HAL::UARTDriver* uartB;
    AP_HAL::UARTDriver* uartC;
    AP_HAL::UARTDriver* uartD;
    AP_HAL::UARTDriver* uartE;
    AP_HAL::I2CDriver* i2c;
    AP_HAL::SPIDeviceManager* spi;
    AP_HAL::AnalogIn* analogin;
    AP_HAL::Storage* storage;
    AP_HAL::UARTDriver* console;
    AP_HAL::GPIO* gpio;
    AP_HAL::RCInput* rcin;
    AP_HAL::RCOutput* rcout;
    AP_HAL::Scheduler* scheduler;
    AP_HAL::Util* util;
};
...
```

####AP_HAL::AnalogIn
[AP_HAL::AnalogIn](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/AnalogIn.h) defines abstract methods for voltage measurement of analog signals. The pure virtual `AP_HAL::AnalogSource` class is also defined in this class.


```cpp
...
class AP_HAL::AnalogSource {
public:
    virtual float read_average() = 0;
    virtual float read_latest() = 0;
    virtual void set_pin(uint8_t p) = 0;
...

class AP_HAL::AnalogIn {
public:
    virtual void init(void* implspecific) = 0;
    virtual AP_HAL::AnalogSource* channel(int16_t n) = 0;

    // board 5V rail voltage in volts
    virtual float board_voltage(void) = 0;

    // servo rail voltage in volts, or 0 if unknown
    virtual float servorail_voltage(void) { return 0; }

    // power supply status flags, see MAV_POWER_STATUS
    virtual uint16_t power_status_flags(void) { return 0; }
};
...
```


####AP_HAL::GPIO
[AP_HAL::GPIO](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/GPIO.h) defines abstract methods for handling the General Purpose Input/Output pines in embedded systems. The pure virtual `AP_HAL::DigitalSource` class is also defined in this class.


```cpp
...
class AP_HAL::DigitalSource {
public:
    virtual void mode(uint8_t output) = 0;
    virtual uint8_t read() = 0;
    virtual void write(uint8_t value) = 0;
    virtual void toggle() = 0;
};

class AP_HAL::GPIO {
public:
    GPIO() {}
    virtual void init() = 0;
    virtual void pinMode(uint8_t pin, uint8_t output) = 0;
    virtual uint8_t read(uint8_t pin) = 0;
    virtual void write(uint8_t pin, uint8_t value) = 0;
    virtual void toggle(uint8_t pin) = 0;
    virtual int8_t analogPinToDigitalPin(uint8_t pin) = 0;

    /* Alternative interface: */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;

    /* Interrupt interface: */
    virtual bool attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode) = 0;

    /* return true if USB cable is connected */
    virtual bool usb_connected(void) = 0;
};
...
```

####AP_HAL::I2CDriver
The [AP_HAL::I2CDriver](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/I2CDriver.h) interface supports the timeout features we require to assure safe timing properties when polling the hardware I2C peripeheral. It also defines abstract methods for **read/write registers** through the **I2C** bus.


```cpp
...
class AP_HAL::I2CDriver {
public:
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void setTimeout(uint16_t ms) = 0;
    virtual void setHighSpeed(bool active) = 0;

    /* write: for i2c devices which do not obey register conventions */
    virtual uint8_t write(uint8_t addr, uint8_t len, uint8_t* data) = 0;
    /* writeRegister: write a single 8-bit value to a register */
    virtual uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val) = 0;
...
    /* read: for i2c devices which do not obey register conventions */
    virtual uint8_t read(uint8_t addr, uint8_t len, uint8_t* data) = 0;
    /* readRegister: read from a device register - writes the register,
* then reads back an 8-bit value. */
    virtual uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data) = 0;
...

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    /* readRegistersMultiple: read contigious device registers.
Equivalent to count calls to readRegisters() */
    virtual uint8_t readRegistersMultiple(uint8_t addr, uint8_t reg,
                                          uint8_t len, uint8_t count,
                                          uint8_t* data) = 0;
#endif

...
};
...
```

####AP_HAL::RCInput
The [AP_HAL::RCInput](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/RCInput.h) interface is based on the input related methods of the existing ArduPilot `APM_RC` class. It also defines methods for handling **Radio Control (RC)** input signals processing like telemetry.


```cpp
...
#define RC_INPUT_MIN_PULSEWIDTH 900
#define RC_INPUT_MAX_PULSEWIDTH 2100

class AP_HAL::RCInput {
public:
    /**
* Call init from the platform hal instance init, so that both the type of
* the RCInput implementation and init argument (e.g. ISRRegistry) are
* known to the programmer. (Its too difficult to describe this dependency
* in the C++ type system.)
*/
    virtual void init(void* implspecific) = 0;

    /**
* Return true if there has been new input since the last read() call
*/
    virtual bool new_input() = 0;

    /**
* Return the number of valid channels in the last read
*/
    virtual uint8_t num_channels() = 0;

    /* Read a single channel at a time */
    virtual uint16_t read(uint8_t ch) = 0;

    /* Read an array of channels, return the valid count */
    virtual uint8_t read(uint16_t* periods, uint8_t len) = 0;
...

```

####AP_HAL::RCOutput
The [AP_HAL::RCOutput](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/RCOutput.h) interface is based on the input related methods of the existing ArduPilot `APM_RC` class. This abstract class defines methods for handling **Radio Control (RC) **output signals generation as the sent to the motors.


```cpp
...
/* Define the CH_n names, indexed from 1, if we don't have them already */
#ifndef CH_1
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
...
class AP_HAL::RCOutput {
public:
    virtual void init(void* implspecific) = 0;

    /* Output freq (1/period) control */
    virtual void set_freq(uint32_t chmask, uint16_t freq_hz) = 0;
    virtual uint16_t get_freq(uint8_t ch) = 0;

    /* Output active/highZ control, either by single channel at a time
* or a mask of channels */
    virtual void enable_ch(uint8_t ch) = 0;
    virtual void disable_ch(uint8_t ch) = 0;

    /* Output, either single channel or bulk array of channels */
    virtual void write(uint8_t ch, uint16_t period_us) = 0;
    virtual void write(uint8_t ch, uint16_t* period_us, uint8_t len) = 0;

    /* Read back current output state, as either single channel or
* array of channels. */
    virtual uint16_t read(uint8_t ch) = 0;
    virtual void read(uint16_t* period_us, uint8_t len) = 0;

    /*
set PWM to send to a set of channels when the safety switch is
in the safe state
*/
    virtual void set_safety_pwm(uint32_t chmask, uint16_t period_us) {}
...
```

####AP_HAL::SPIDriver
[AP_HAL::SPIDriver](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/SPIDriver.h) defines two classes `AP_HAL::SPIDeviceManager` and `AP_HAL::SPIDeviceDriver`, that defines abstract methods to managing the synchronous **Serial Peripheral Interface Bus**.


```cpp
...
class AP_HAL::SPIDeviceManager {
public:
    virtual void init(void *) = 0;
    virtual AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice) = 0;
};

/**
* We still need an abstraction for performing bulk
* transfers to be portable to other platforms.
*/

class AP_HAL::SPIDeviceDriver {
public:
    virtual void init() = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) = 0;

    virtual void cs_assert() = 0;
    virtual void cs_release() = 0;
    virtual uint8_t transfer (uint8_t data) = 0;
    virtual void transfer (const uint8_t *data, uint16_t len) = 0;

    /**
       optional set_bus_speed() interface. This can be used by drivers
       to request higher speed for sensor registers once the sensor is
       initialised. This is used by the MPU6000 driver which can
       handle 20MHz for sensor register reads, but only 1MHz for other
       registers.
    */
    enum bus_speed {
        SPI_SPEED_LOW, SPI_SPEED_HIGH
    };

    virtual void set_bus_speed(enum bus_speed speed) {}

};
...
```

####AP_HAL::Scheduler
The [AP_HAL::Scheduler](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h) interface is designed to encapsulate **scheduling asynchronous processes** as a replacement to the ArduPilot `AP_PeriodicProcess` driver. Is responsible for managing the time of the processes and tasks.


```cpp
...
class AP_HAL::Scheduler {
public:
    Scheduler() {}
    virtual void init(void* implspecific) = 0;
    virtual void delay(uint16_t ms) = 0;
    virtual uint32_t millis() = 0;
    virtual uint32_t micros() = 0;
    virtual void delay_microseconds(uint16_t us) = 0;
    virtual void register_delay_callback(AP_HAL::Proc,
                                             uint16_t min_time_ms) = 0;

    // register a high priority timer task
    virtual void register_timer_process(AP_HAL::MemberProc) = 0;

    // register a low priority IO task
    virtual void register_io_process(AP_HAL::MemberProc) = 0;

    // suspend and resume both timer and IO processes
    virtual void suspend_timer_procs() = 0;
    virtual void resume_timer_procs() = 0;
...
    /**
      optional function to set timer speed in Hz
     */
    virtual void set_timer_speed(uint16_t speed_hz) {}

    /**
      optional function to stop clock at a given time, used by log replay
     */
    virtual void stop_clock(uint64_t time_usec) {}
...
```

####AP_HAL::Semaphores
This class defines a **semaphore abstract data type** that is used for controlling access, by multiple processes, to a common resource in a parallel programming.


```cpp
#ifndef __AP_HAL_SEMAPHORES_H__
#define __AP_HAL_SEMAPHORES_H__

#include <AP_HAL_Namespace.h>

#define HAL_SEMAPHORE_BLOCK_FOREVER ((uint32_t) 0xFFFFFFFF)

class AP_HAL::Semaphore {
public:
    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;
    virtual bool give() = 0;
};

#endif // __AP_HAL_SEMAPHORES_H__
```

####AP_HAL::Storage
Defines abstract methods for **read and write** data storage media, like a SDCard or a EEPROM, normally for storing flight/ride data.


```cpp
#ifndef __AP_HAL_STORAGE_H__
#define __AP_HAL_STORAGE_H__

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Storage {
public:
    virtual void init(void *) = 0;
    virtual uint8_t read_byte(uint16_t loc) = 0;
    virtual uint16_t read_word(uint16_t loc) = 0;
    virtual uint32_t read_dword(uint16_t loc) = 0;
    virtual void read_block(void *dst, uint16_t src, size_t n) = 0;

    virtual void write_byte(uint16_t loc, uint8_t value) = 0;
    virtual void write_word(uint16_t loc, uint16_t value) = 0;
    virtual void write_dword(uint16_t loc, uint32_t value) = 0;
    virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;
};

#endif // __AP_HAL_STORAGE_H__
```

####AP_HAL::UARTDriver
[UARTDriver.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.h) is an abstract class that defines methods for handling asynchronous  serial ports and peripherals. It also is the replacement for ArduPilot's `FastSerial` library. The class hierchary for `AP_HAL::UARTDriver` is also derived directly from the `FastSerial` class's hierarchy . `AP_HAL::UARTDriver` is a public `AP_HAL::BetterStream`, which is a public `AP_HAL::Stream`, which is a public `AP_HAL::Print`.


```cpp
...
class AP_HAL::UARTDriver : public AP_HAL::BetterStream {
public:
    UARTDriver() {}
    virtual void begin(uint32_t baud) = 0;
      /// Extended port open method
      ///
      /// Allows for both opening with specified buffer sizes, and re-opening
      /// to adjust a subset of the port's settings.
      ///
      /// @note Buffer sizes greater than ::_max_buffer_size will be rounded
      /// down.
      ///
      /// @param baud Selects the speed that the port will be
      /// configured to. If zero, the port speed is left
      /// unchanged.
      /// @param rxSpace Sets the receive buffer size for the port. If zero
      /// then the buffer size is left unchanged if the port
      /// is open, or set to ::_default_rx_buffer_size if it is
      /// currently closed.
      /// @param txSpace Sets the transmit buffer size for the port. If zero
      /// then the buffer size is left unchanged if the port
      /// is open, or set to ::_default_tx_buffer_size if it
      /// is currently closed.
      ///
    virtual void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) = 0;
    virtual void end() = 0;
    virtual void flush() = 0;
    virtual bool is_initialized() = 0;
    virtual void set_blocking_writes(bool blocking) = 0;
    virtual bool tx_pending() = 0;

    enum flow_control {
        FLOW_CONTROL_DISABLE=0, FLOW_CONTROL_ENABLE=1, FLOW_CONTROL_AUTO=2
    };
    virtual void set_flow_control(enum flow_control flow_control_setting) {};
    virtual enum flow_control get_flow_control(void) { return FLOW_CONTROL_DISABLE; };

    /* Implementations of BetterStream virtual methods. These are
     * provided by AP_HAL to ensure consistency between ports to
     * different boards
    */
    void print_P(const prog_char_t *s);
    void println_P(const prog_char_t *s);
    void printf(const char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));
    void _printf_P(const prog_char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));

    void vprintf(const char *s, va_list ap);
    void vprintf_P(const prog_char *s, va_list ap);
};
```

[UARTDriver.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.cpp) implements BetterStream methods in AP_HAL to ensure consistent behaviour on all boards.


```cpp
...
#include "utility/print_vprintf.h"
#include "UARTDriver.h"

/*
BetterStream method implementations
These are implemented in AP_HAL to ensure consistent behaviour on
all boards, although they can be overridden by a port
*/

void AP_HAL::UARTDriver::print_P(const prog_char_t *s)
{
    char c;
    while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
        write(c);
}

void AP_HAL::UARTDriver::println_P(const prog_char_t *s)
{
    print_P(s);
    println();
}

void AP_HAL::UARTDriver::printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}
...
```

####AP_HAL::Util
[Util.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Util.h) class is a util member for string utilities. It also defines virtual methods like `set_system_clock()`, `get_system_id()`, `available_memory()`...


```cpp
class AP_HAL::Util {
public:
    int snprintf(char* str, size_t size,
                 const char *format, ...);

    int snprintf_P(char* str, size_t size,
                   const prog_char_t *format, ...);

    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    int vsnprintf_P(char* str, size_t size,
                    const prog_char_t *format, va_list ap);

    // run a debug shall on the given stream if possible. This is used
    // to support dropping into a debug shell to run firmware upgrade
    // commands
    virtual bool run_debug_shell(AP_HAL::BetterStream *stream) = 0;

    enum safety_state {
        SAFETY_NONE, SAFETY_DISARMED, SAFETY_ARMED
    };

    /*
return state of safety switch, if applicable
*/
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /*
set system clock in UTC microseconds
*/
    virtual void set_system_clock(uint64_t time_utc_usec) {}

...
```

[Util.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Util.cpp) implements `Util.h` methods and defines `BufferPrinter` helper class, that implements `AP_HAL::Print` so we can use utility/vprintf.


```cpp
/* Helper class implements AP_HAL::Print so we can use utility/vprintf */
class BufferPrinter : public AP_HAL::Print {
public:
    BufferPrinter(char* str, size_t size) : _offs(0), _str(str), _size(size) {}
    size_t write(uint8_t c) {
        if (_offs < _size) {
            _str[_offs] = c;
            _offs++;
            return 1;
        } else {
            return 0;
        }
    }
    size_t write(const uint8_t *buffer, size_t size) {
        size_t n = 0;
        while (size--) {
            n += write(*buffer++);
        }
        return n;
    }

    size_t _offs;
    char* const _str;
    const size_t _size;
};

int AP_HAL::Util::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}
...
```
