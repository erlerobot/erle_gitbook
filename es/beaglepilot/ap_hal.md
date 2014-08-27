# AP_HAL

### Overview

`AP_HAL` is hardware abstraction layer for the `ArduPilot` project. The `AP_HAL` consists of a set of headers (`.h`) that define the classes and methods that should be implemmented if ardupilot should run in a new device/architecture. The code contained in this HALs (**Hardware Abstraction Layer**s) is usually quite **low level** and close to the hardware.

 The `AP_HAL` code can be found at  [/libraries/AP_HAL/](https://github.com/diydrones/ardupilot/tree/master/libraries/AP_HAL).

### Using the AP_HAL

`AP_HAL`, found in `/libraries/AP_HAL/`, is a library of purely virtual classes: there is no concrete code in the `AP_HAL` library, only interfaces. All code in the `ArduPilot` libraries and example sketches should depend only on the interfaces exposed by `AP_HAL`.

---

**This class is expected to be inherited in order to creat a Hardware Abstraction Layer**

---

The collection of classes in the `AP_HAL` library exist in the `AP_HAL` C++ namespace. The convention is for a program to instantiate a single instance of the `AP_HAL::HAL class`, under a reference to the name hal. This way:

```cpp
#include <AP_HAL.h>
const AP_HAL::HAL& hal = specific_hal_implementation;
```

will create an instance that should be in a single object file. All other object files, including libraries (even those inside an `AP_HAL` implementation), should use the `AP_HAL` interface by declaring an extern reference to hal:

```cpp
#include <AP_HAL.h> extern const AP_HAL::HAL& hal;
```


###AP_HAL Contents

The `AP_HAL library` has the following contents:

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
Exposes the C++ namespace [AP_HAL](https://github.com/diydrones/ardupilot/tree/master/libraries/AP_HAL). The namespace declaration declares each class by name (not implementation) and some useful `typedefs`.

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
C preprocesor enumeration of the boards supported by the `AP_HAL`.

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
These macros allow the code to build on multiple platforms more easily.


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

####AP_HAL interface classes

The `AP_HAL interface classes` are each defined in a header file bearing their name.

The following abstractions compound the module `AP_HAL`:


- `AP_HAL::HAL` class is a container for the a complete set of device drivers. The class is defined in [/libraries/AP_HAL/HAL.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/HAL.h). It also has a virtual (i.e. overridable) method to handle driver initialization. Each device driver is exposed as a pointer an `AP_HAL` driver class, (e.g. each serial driver is exposed as a public `UARTDriver* uartN`).


- `AP_HAL::AnalogIn` class is pure virtual and can be found in [/libraries/AP_HAL/AnalogIn.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/AnalogIn.h). The pure virtual `AP_HAL::AnalogSource` class is also defined in that class. Defines abstract methods for voltage measurement of analog signals.


- `AP_HAL::GPIO` class is pure virtual and can be found in [/libraries/AP_HAL/GPIO.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/GPIO.h).Defines abstract methods for handling the General Purpose Input/Output pines in embedded systems.


- `AP_HAL::I2CDriver` class is a pure virtual interface, found in [/libraries/AP_HAL/I2CDriver.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/I2CDriver.h) and is the `AP_HAL` replacment for ArduPilot's `I2C` library. The `I2CDriver` interface supports the timeout features we require to assure safe timing properties when polling the hardware I2C peripeheral(I2C bus helps you monitor the health of your system). It also only exposes whole-transaction interfaces to the user, to support more efficient implementations in a threaded environment.


- `AP_HAL::RCInput` class is pure virtual and can be found in [/libraries/AP_HAL/RCInput.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/RCInput.h). The RCInput interface is based on the input related methods of the existing ArduPilot `APM_RC` class. RCInput methods were separated from RCOutput methods for clarity.


- `AP_HAL::RCOutput` class is pure virtual and can be found in [/libraries/AP_HAL/RCOutput.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/RCOutput.h). The RCOutput interface is based on the input related methods of the existing ArduPilot `APM_RC` class. RCOutput methods were separated from RCInput methods for clarity.


- `AP_HAL::Scheduler` class is pure virtual and can be found in [/libraries/AP_HAL/Scheduler.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h). The `AP_HAL::Scheduler` interface is designed to encapsulate scheduling asynchronous processes as a replacement to the ArduPilot `AP_PeriodicProcess` driver.


- `AP_HAL::Semaphores`class is pure virtual and can be found in [/libraries/AP_HAL/Semaphores.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Semaphores.h).This class defines a semaphore abstract data type that is used for controlling access, by multiple processes, to a common resource in a parallel programming.


- `AP_HAL::SPIDriver` class is pure virtual and can be found in [/libraries/AP_HAL/SPIDriver.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/SPIDriver.h).It defines two classes `AP_HAL::SPIDeviceManager` and `AP_HAL::SPIDeviceDriver`, that defines methods to managin the Serial Peripheral Interface Bus.


- `AP_HAL::Storage`class is pure virtual and can be found in [/libraries/AP_HAL/Storage.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Storage.h) and it defines abstract methods for read and write data storage media like a SDCard.


- `AP_HAL::UARTDriver`class is a pure virtual interface and the replacement for ArduPilot's `FastSerial` library. It provides the methods `begin()`, `end()`, `flush()`, `is_initialized()`, `set_blocking_writes`, and `tx_pending`. The class hierchary for `AP_HAL::UARTDriver` is also derived directly from the `FastSerial` class's hierarchy . `AP_HAL::UARTDriver` is a public `AP_HAL::BetterStream`, which is a public `AP_HAL::Stream`, which is a public `AP_HAL::Print`.
You can find `AP_HAL::UARTDriver` class at [/libraries/AP_HAL/UARTDriver.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.h) and at [/libraries/AP_HAL/UARTDriver.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.cpp).


- `AP_HAL::Util` class is pure virtual and can be found in [/libraries/AP_HAL/Util.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Util.h).`AP_HAL::Util` class defines Util member for string utilities.

