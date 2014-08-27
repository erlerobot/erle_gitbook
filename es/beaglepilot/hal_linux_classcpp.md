# HAL_Linux_Class.cpp

Link to the code [HAL_Linux_Class.cpp](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/HAL_Linux_Class.cpp)

```cpp
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "HAL_Linux_Class.h"
#include "AP_HAL_Linux_Private.h"

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
...
```
-  `AP_HAL.h` , `HAL_Linux_Class.h`and
`AP_HAL_Linux_Private.h` are imported. The board is defined.


 Also some libraries and functions are imported:

+ The getopt function was written to be a standard mechanism that all programs could use to parse command-line options so that there would be a common interface that everyone could depend on.(automate some of the chore involved in parsing typical unix command line options.) - [geopt](http://www.gnu.org/savannah-checkouts/gnu/libc/manual/html_node/Getopt.html)

+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)


+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+ This header defines several general purpose functions, including dynamic memory management, random number generation, communication with the environment, integer arithmetics, searching, sorting and converting.- [(stdlib.h)](http://www.cplusplus.com/reference/cstdlib/?kw=stdlib.h)

```cpp
...
using namespace Linux;

// 3 serial ports on Linux for now
static LinuxUARTDriver uartADriver(true);
static LinuxUARTDriver uartBDriver(false);
static LinuxUARTDriver uartCDriver(false);

static LinuxSemaphore  i2cSemaphore;
static LinuxI2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static LinuxSPIDeviceManager spiDeviceManager;
static LinuxAnalogIn analogIn;
static LinuxStorage storageDriver;
static LinuxGPIO gpioDriver;
static LinuxRCInput rcinDriver;
static LinuxRCOutput rcoutDriver;
static LinuxScheduler schedulerInstance;
static LinuxUtil utilInstance;
...
```
- Define three UARTDrivers (The Universal Asynchronous Receiver/Transmitter (UART) controller is the key component of the serial communications subsystem of a computer. The UART takes bytes of data and transmits the individual bits in a sequential fashion. At the destination, a second UART re-assembles the bits into complete bytes). Initialize A-true and B/C-False.


- Define *static members* ( There is only one common variable for all the objects of that same class, sharing the same value, this is, its value is not different from one object of this class to another.)

```cpp
...
HAL_Linux::HAL_Linux() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}
...
```
- Implements the `HAL()` method passing the values by reference.

```cpp
...
void _usage(void)
{
    printf("Usage: -A uartAPath -B uartBPath -C uartCPath\n");
    printf("Options:\n");
    printf("\t-serial:          -A /dev/ttyO4\n");
    printf("\t                  -B /dev/ttyS1\n");
    printf("\t-tcp:             -C tcp:192.168.2.15:1243:wait\n");
    printf("\t                  -A tcp:11.0.0.2:5678\n");
}
...
```
- Defines a void function (doesn't return anything) that prints some information using `printf` from stdio.h library.

```cpp
...
void HAL_Linux::init(int argc,char* const argv[]) const
{
    int opt;
    /*
      parse command line options
     */
    while ((opt = getopt(argc, argv, "A:B:C:h")) != -1) {
        switch (opt) {
        case 'A':
            uartADriver.set_device_path(optarg);
            break;
        case 'B':
            uartBDriver.set_device_path(optarg);
            break;
        case 'C':
            uartCDriver.set_device_path(optarg);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    scheduler->init(NULL);
    gpio->init();
    rcout->init(NULL);
    uartA->begin(115200);
    i2c->begin();
    spi->init(NULL);
    utilInstance.init(argc, argv);
}

const HAL_Linux AP_HAL_Linux;

#endif
```
- Defines a void function:
  + Include a `case` that calls the `set_device_path` with optarg. optarg is an environment variable set by getopt to point at the value of the option argument, for those options that accept arguments.

  + Initializes the varibles passed to HAL by reference.

  + Defines the const exportable member.
