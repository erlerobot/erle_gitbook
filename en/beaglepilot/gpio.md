# GPIO

Class that takes care of **GPIO** (General-purpose input/output).

---
**GPIO** is a generic pin on an integrated circuit whose behavior, including whether it is an input or output pin, can be controlled by the user at run time.

**GPIO pins** have no special purpose defined, and go unused by default. The idea is that sometimes the system integrator building a full system that uses the chip might find it useful to have a handful of additional digital control lines, and having these available from the chip can save the hassle of having to arrange additional circuitry to provide them.

---
This class is divided into two files, **header** (`GPIO.h`) and **source code** (`GPIO.cpp`).

###GPIO.h


Link to the code: [GPIO.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/GPIO.h)

`Linux::LinuxGPIO `class defines the methods inherited from the [AP_HAL::GPIO](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/GPIO.h) abstract class.Linux::LinuxDigitalSource` class is also defined in this file and inherit from `AP_HAL::DigitalSource`.


For more clarity in the code: The GPIO files define C preproccesor Hexadecimal addresses and BeagleBone Black GPIO mappings in linux .

```cpp

#ifndef __AP_HAL_LINUX_GPIO_H__
#define __AP_HAL_LINUX_GPIO_H__

#include <AP_HAL_Linux.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"

#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000

#define GPIO_SIZE  0x00000FFF

// OE: 0 is output, 1 is input
#define GPIO_OE    0x14d
#define GPIO_IN    0x14e
#define GPIO_OUT   0x14f

#define LED_AMBER       117
#define LED_BLUE        48
#define LED_SAFETY      61
#define SAFETY_SWITCH   116
#define LOW             0
#define HIGH            1

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
#define LINUX_GPIO_NUM_BANKS 4
#else
// disable GPIO
#define LINUX_GPIO_NUM_BANKS 0
#endif

// BeagleBone Black GPIO mappings
#define BBB_USR0 53
#define BBB_USR1 54
#define BBB_USR2 55
#define BBB_USR3 56
#define BBB_P8_3 38
#define BBB_P8_4 39
#define BBB_P8_5 34
...
#define BBB_P9_42 7
...

```
- Following the statement:` #define identifier replacement`, when the preprocessor encounters this directive, it replaces any occurrence of `identifier` in the rest of the code by `replacement`.This is what is done for the identifiers in the code above.


- It aslo containts a *or statemt* (||) in order to enable and sisable the GPIO, depending on the specified board.



```cpp
...

class Linux::LinuxGPIO : public AP_HAL::GPIO {
private:
    struct GPIO {
        volatile uint32_t *base;
        volatile uint32_t *oe;
        volatile uint32_t *in;
        volatile uint32_t *out;
     } gpio_bank[LINUX_GPIO_NUM_BANKS];

public:
    LinuxGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};
...
```

- The class `LinuxGPIO` is defined. It inherits from [AP_HAL::GPIO](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/GPIO.h).


- The volatile keyword is a type qualifier used to declare that an object can be modified in the program by something such as the operating system, the hardware, or a concurrently executing thread.Here are defined some volatile pointers.


- It also defines a GPIO struct called `gpio_bank[]`, as the name suggests, is used to group pins in banks.You can access a bank by index.


- In the public fields: there is a `init()` method definition, `read()` and `write()` methods... (these functions are implemented in `GPIO.cpp`)


- The methods defined inside `GPIO.h` in class `Linux::LinuxGPIO` like `LinuxGPIO::pinMode()`,`LinuxGPIO::read()`,`LinuxGPIO::write()` and so on, are implemented for handle the GPIO pin banks of the board.

```cpp
...
class Linux::LinuxDigitalSource : public AP_HAL::DigitalSource {
public:
    LinuxDigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value);
    void    toggle();
private:
    uint8_t _v;

};

#endif // __AP_HAL_LINUX_GPIO_H__
```
- Defines the `LinuxDigitalSource`that inherits from `DigitalSource`, which is part of [AP_HAL::GPIO](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/GPIO.h)


- The methods defined inside `gpio.h` in `Linux :: LinuxDigitalSource` class, have been implemented here to manage digital external sources.


- Some methods are defined here, for later implementation in `GPIO.cpp`.


###GPIO.cpp


[GPIO.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/GPIO.cpp) implements the methods defined  in `GPIO.h`.

The most remarkable is the `init()` method which enables all GPIO banks, open the export directory `/sys/class/gpio/export`,  and map the GPIO banks in `/dev/mem`.

```cpp
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "GPIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
...
```
- In this piece of code `AP_HAL.h` and `GPIO.h` are included and the board is defined.

Some functions and libraries are included:

+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)


+ This header defines several general purpose functions, including dynamic memory management, random number generation, communication with the environment, integer arithmetics, searching, sorting and converting.- [(stdlib.h)](http://www.cplusplus.com/reference/cstdlib/?kw=stdlib.h)


+  The `string.h` header file defines several functions to manipulate C strings and arrays. - [(string.h)](http://www.cplusplus.com/reference/cstring/)


+ C Header that defines the following macro:
`errno->Last error number (macro )`; plus at least three additional macro constants: EDOM, ERANGE and EILSEQ.`errno`delas with errors (see [errno](http://www.cplusplus.com/reference/cerrno/errno/) for more details). - [(errno.h)](http://www.cplusplus.com/reference/cerrno/?kw=errno.h)


+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+ The `fcntl.h` header shall define some requests and arguments for use by the functions `fcntl()` and `open()`. - [(fcntl.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/fcntl.h.html)



+ The `poll.h` header defines the `pollfd` structure that includes at least the following member:
int *fd*(the following descriptor being polled),
short int *events* (the input event flags) and
short int *revents * ( the output event flags) - [(poll.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/poll.h.html)


+ The header `sys/mman.h` defines protection options when Advisory Information, Memory Mapped Files, or Shared Memory Objects options are supported.- [(sys/mman.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/mman.h.html)


+ The `sys/stat.h` header defines the structure of the data returned by the functions `fstat()`,` lstat()`, and `stat()`. - [(sys/stat.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html)

```cpp
...
using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
LinuxGPIO::LinuxGPIO()
{}
...
```
- Begins a namespace definition, takes the `AP_HAL::HAL& hal` value and call `LinuxGPIO()` funtion that manage pin banks.

```cpp
...
void LinuxGPIO::init()
{
#if LINUX_GPIO_NUM_BANKS == 4
    int mem_fd;
    // Enable all GPIO banks
    // Without this, access to deactivated banks (i.e. those with no clock source set up) will (logically) fail with SIGBUS
    // Idea taken from https://groups.google.com/forum/#!msg/beagleboard/OYFp4EXawiI/Mq6s3sg14HoJ

    uint8_t bank_enable[3] = { 5, 65, 105 };
    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
    if (export_fd == -1) {
        hal.scheduler->panic("unable to open /sys/class/gpio/export");
    }
    for (uint8_t i=0; i<3; i++) {
        dprintf(export_fd, "%u\n", (unsigned)bank_enable[i]);
    }
    close(export_fd);


    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
            printf("can't open /dev/mem \n");
            exit (-1);
    }

    /* mmap GPIO */
    off_t offsets[LINUX_GPIO_NUM_BANKS] = { GPIO0_BASE, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE };
    for (uint8_t i=0; i<LINUX_GPIO_NUM_BANKS; i++) {
        gpio_bank[i].base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, offsets[i]);
        if ((char *)gpio_bank[i].base == MAP_FAILED) {
            hal.scheduler->panic("unable to map GPIO bank");
        }
        gpio_bank[i].oe = gpio_bank[i].base + GPIO_OE;
        gpio_bank[i].in = gpio_bank[i].base + GPIO_IN;
        gpio_bank[i].out = gpio_bank[i].base + GPIO_OUT;
    }

    close(mem_fd);
#endif // LINUX_GPIO_NUM_BANKS
}
...
```
Enable all GPIO banks(setting up the banks):

+ First, try to open  the file `/sys/class/gpio/export`: if fail print a message, if not print the content.


+ Open `/dev/mem`.


+ `off_t` is a type used to pass offset to various file related functions. With this we enable GPIO banks, mapping them in `/dev/mem`.

```cpp

...

void LinuxGPIO::pinMode(uint8_t pin, uint8_t output)
{
    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (output == HAL_GPIO_INPUT) {
        *gpio_bank[bank].oe |= (1U<<bankpin);
    } else {
        *gpio_bank[bank].oe &= ~(1U<<bankpin);
    }
}

int8_t LinuxGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}
...
```
- Check the bank numbers and the output. Note the use of 1U for the left shift, to make it unsigned and the allocation using binary asigments.


- The return -1 is like a return boolean=False.

```cpp
...
uint8_t LinuxGPIO::read(uint8_t pin) {

    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return 0;
    }
    return *gpio_bank[bank].in & (1U<<bankpin) ? HIGH : LOW;

}


void LinuxGPIO::write(uint8_t pin, uint8_t value)
{
    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (value == LOW) {
        *gpio_bank[bank].out &= ~(1U<<bankpin);
    } else {
        *gpio_bank[bank].out |= 1U<<bankpin;
    }
}
...
```
- Similarly to `pinmode()` acts `read()` and `write()`.

```
...
void LinuxGPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* LinuxGPIO::channel(uint16_t n) {
    return new LinuxDigitalSource(n);
}

/* Interrupt interface: */
bool LinuxGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}

bool LinuxGPIO::usb_connected(void)
{
    return false;
}

LinuxDigitalSource::LinuxDigitalSource(uint8_t v) :
    _v(v)
{

}

void LinuxDigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_v, output);
}

uint8_t LinuxDigitalSource::read()
{
    return hal.gpio->read(_v);
}

void LinuxDigitalSource::write(uint8_t value)
{
    return hal.gpio->write(_v,value);
}

void LinuxDigitalSource::toggle()
{
    write(!read());
}

#endif // CONFIG_HAL_BOARD
```
- Here the missing functions are implemented.
