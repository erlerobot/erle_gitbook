# SPIDriver


Class that takes care of **SPI** (Serial Peripheral Interface) bus.

---
The **Serial Peripheral Interface** or SPI bus is a synchronous serial data link that operates in full duplex mode. It is used for short distance, **single master** communication, for example in embedded systems, sensors, and SD cards.

Devices communicate in **master/slave** mode where the master device initiates the **data frame**. Multiple slave devices are allowed with individual slave select lines.

The SPI is a **synchronous protocol**. The synchronization and data transmission is performed by means of four signals:

- **SCLK** : Serial Clock (output from master).
- **MOSI** : Master Output, Slave Input (output from master).
- **MISO** : Master Input, Slave Output (output from slave).
- **SS** : Slave Select (active low, output from master).

SPI is a **single-master communication protocol**. This means that one central device initiates all the communications with the slaves. When the **SPI master** wishes to send data to a **slave** and/or request information from it, it selects slave by pulling the corresponding **SS line low** and it activates the clock signal at a clock frequency usable by the master and the slave. The master generates information onto **MOSI line** while it samples the **MISO line**.

![spidriver](../en/img/beaglepilot/SPI-d.png)

---
This class is divided into two files, **header** (`SPIDriver.h`) and **source code** (`SPIDriver.cpp`).


### SPIDriver.h


Link to the code:[SPIDriver.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/SPIDriver.h)

`Linux::LinuxSPIDeviceDriver`class defines the methods inherited from the [AP_HAL::SPIDeviceDriver](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/SPIDriver.h) abstract class. Mainly used to access the SPI device.

`Linux::LinuxSPIDeviceManager` is declared as a friend class so it can access to the private members of `Linux::LinuxSPIDeviceDriver` class without instantiating.

```cpp
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include <AP_HAL_Linux.h>
#include "Semaphores.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
#define LINUX_SPI_NUM_BUSES 2
#define LINUX_SPI_DEVICE_NUM_DEVICES 5
#else
#define LINUX_SPI_NUM_BUSES 0
#define LINUX_SPI_DEVICE_NUM_DEVICES 0
#endif

...
```

- Imported headers, configuration on the board and definitions are included in this slice if code.

```cpp
...
class Linux::LinuxSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    friend class Linux::LinuxSPIDeviceManager;
    LinuxSPIDeviceDriver(uint8_t bus, enum AP_HAL::SPIDevice type, uint8_t mode, uint8_t bitsPerWord, uint8_t cs_pin, uint32_t lowspeed, uint32_t highspeed);
    void init();
    AP_HAL::Semaphore *get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
    void set_bus_speed(enum bus_speed speed);

private:
    uint8_t _cs_pin;
    AP_HAL::DigitalSource *_cs;
    uint8_t _mode;
    uint8_t _bitsPerWord;
    uint32_t _lowspeed;
    uint32_t _highspeed;
    uint32_t _speed;
    enum AP_HAL::SPIDevice _type;
    uint8_t _bus;
};
...
```
- The `LinusSPiDeviceDriver inherits from [AP_HAL::SPIDeviceDriver](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/SPIDriver.h).


- Some methods are defined here for later implementation in `SPIDriver.cpp`. Note the `Semaphore method` used to control access.

```cpp
...
class Linux::LinuxSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

    static AP_HAL::Semaphore *get_semaphore(uint8_t bus);

    static void cs_assert(enum AP_HAL::SPIDevice type);
    static void cs_release(enum AP_HAL::SPIDevice type);
    static void transaction(LinuxSPIDeviceDriver &driver, const uint8_t *tx, uint8_t *rx, uint16_t len);

private:
    static LinuxSPIDeviceDriver _device[LINUX_SPI_DEVICE_NUM_DEVICES];
    static LinuxSemaphore _semaphore[LINUX_SPI_NUM_BUSES];
    static int _fd[LINUX_SPI_NUM_BUSES];
};

#endif // __AP_HAL_LINUX_SPIDRIVER_H__
```
- This class inherits from `AP_HAL::SPIDeviceManager` and is also declared as a friend of `LinuxSPIDeviceDriver` so it can access to the members of both classes.

### SPIDriver.cpp


[SPIDriver.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/SPIDriver.cpp) implements the methods of `Linux::LinuxSPIDeviceDriver` and `Linux::LinuxSPIDeviceManager` classes defined  in `SPIDriver.h`.

---
It instantiate a SPI driver for each connected device, and then `LinuxSPIDeviceManager` class uses those drivers and an `AP_HAL::SPIDevice type` for his methods.

---

It is important to remark that it defines different **SPI tables per board subtype**, in this case 6 SPI devices for PXF cape, else defines a empty device table.It also instantiate a separate **semaphore** and **file descriptor** per bus.
```cpp

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "SPIDriver.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "GPIO.h"

#define SPI_DEBUGGING 1
...
```
- Include `AP_HAL.h`, `SPIDriver.h` and `GPIO.h`. Configures the board.


Some functions and libraries are included:

+ Header `sys/types.h` shall include definitions for types- [(sys/types.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/types.h.html)


+ The `sys/stat.h` header defines the structure of the data returned by the functions `fstat()`,` lstat()`, and `stat()`. - [(sys/stat.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html)



+ The `fcntl.h` header shall define some requests and arguments for use by the functions `fcntl()` and `open()`. - [(fcntl.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/fcntl.h.html)


+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)


+ This header defines a set of integral type aliases with specific width requirements, along with macros specifying their limits and macro functions to create values of these types.- [(stdint.h)](http://www.cplusplus.com/reference/cstdint/?kw=stdint.h)


+ `ioctl() `function manipulates the underlying device parameters of special files.You can read more in `man ioctl()` - [(sys/ioctl.h)](http://unix.superglobalmegacorp.com/Net2/newsrc/sys/ioctl.h.html)



+ `linux/spi/spidev.h` offers user space versions of kernel symbols for SPI clocking modes. - [(linux/spi/spidev.h)](http://www.cs.fsu.edu/~baker/devices/lxr/http/source/linux/include/linux/spi/spidev.h)

```cpp
...

using namespace Linux;

extern const AP_HAL::HAL& hal;

#define MHZ (1000U*1000U)

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
LinuxSPIDeviceDriver LinuxSPIDeviceManager::_device[LINUX_SPI_DEVICE_NUM_DEVICES] = {
    // different SPI tables per board subtype
    LinuxSPIDeviceDriver(1, AP_HAL::SPIDevice_MS5611,  SPI_MODE_3, 8, BBB_P9_42,  6*MHZ, 6*MHZ),
    LinuxSPIDeviceDriver(1, AP_HAL::SPIDevice_MPU6000, SPI_MODE_3, 8, BBB_P9_28,  500*1000, 20*MHZ),
    /* MPU9250 is restricted to 1MHz for non-data and interrupt registers */
    LinuxSPIDeviceDriver(1, AP_HAL::SPIDevice_MPU9250, SPI_MODE_3, 8, BBB_P9_23,  1*MHZ, 20*MHZ),
    // LinuxSPIDeviceDriver(0, AP_HAL::SPIDevice_LSM9DS0, SPI_MODE_3, 8, BBB_P9_17,  10*MHZ,10*MHZ),
    LinuxSPIDeviceDriver(0, AP_HAL::SPIDevice_L3GD20, SPI_MODE_3, 8, BBB_P8_9,  1*MHZ,10*MHZ),
    LinuxSPIDeviceDriver(1, AP_HAL::SPIDevice_Dataflash,SPI_MODE_0, 8, BBB_P8_12,  6*MHZ, 6*MHZ)
};
#else
// empty device table
LinuxSPIDeviceDriver LinuxSPIDeviceManager::_device[0];
#endif
// have a separate semaphore per bus
LinuxSemaphore LinuxSPIDeviceManager::_semaphore[LINUX_SPI_NUM_BUSES];
int LinuxSPIDeviceManager::_fd[LINUX_SPI_NUM_BUSES];
...
```
- As we have said it **defines  different SPI tables per board subtype** and ,also ,**empty device table**.There are  a separate semaphore instantiate and file descriptor per bus.

```cpp
...

LinuxSPIDeviceDriver::LinuxSPIDeviceDriver(uint8_t bus, enum AP_HAL::SPIDevice type, uint8_t mode, uint8_t bitsPerWord, uint8_t cs_pin, uint32_t lowspeed, uint32_t highspeed):
    _bus(bus),
    _type(type),
    _mode(mode),
    _bitsPerWord(bitsPerWord),
    _lowspeed(lowspeed),
    _highspeed(highspeed),
    _speed(highspeed),
    _cs_pin(cs_pin)
{
}

void LinuxSPIDeviceDriver::init()
{
    // Init the CS
    _cs = hal.gpio->channel(_cs_pin);
    if (_cs == NULL) {
        hal.scheduler->panic("Unable to instantiate cs pin");
    }
    _cs->mode(HAL_GPIO_OUTPUT);
    _cs->write(HIGH);       // do not hold the SPI bus initially
}

AP_HAL::Semaphore* LinuxSPIDeviceDriver::get_semaphore()
{
    return LinuxSPIDeviceManager::get_semaphore(_bus);
}

void LinuxSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    LinuxSPIDeviceManager::transaction(*this, tx, rx, len);
}

void LinuxSPIDeviceDriver::set_bus_speed(enum bus_speed speed)
{
    if (speed == SPI_SPEED_LOW) {
        _speed = _lowspeed;
    } else {
        _speed = _highspeed;
    }
}


void LinuxSPIDeviceDriver::cs_assert()
{
    LinuxSPIDeviceManager::cs_assert(_type);
}

void LinuxSPIDeviceDriver::cs_release()
{
    LinuxSPIDeviceManager::cs_release(_type);
}

uint8_t LinuxSPIDeviceDriver::transfer(uint8_t data)
{
    uint8_t v = 0;
    transaction(&data, &v, 1);
    return v;
}

void LinuxSPIDeviceDriver::transfer(const uint8_t *data, uint16_t len)
{
    transaction(data, NULL, len);
}
...
```
- `LinuxSPIDeviceDriver` methods are implemented here.


- `LinuxSPIDeviceDriver::LinuxSPIDeviceDriver` is a constructor .`LinuxSPIDeviceDriver::init()` method is used to initialize chip select (`_cs = SS`), method `LinuxSPIDeviceDriver::get_semaphore()`.


- The `LinuxSPIDeviceDriver::transaction` method, `LinuxSPIDeviceDriver::set_bus_speed()`, ` LinuxSPIDeviceDriver::cs_assert()`... are initialized too.


- All this methods calls to methods of `LinuxSPIDeviceManager` ( with the same name ), sending them some **private members** of LinuxSPIDeviceDriver like `_type` or `*this` (&driver).

```cpp
...
oid LinuxSPIDeviceManager::init(void *)
{
    char path[] = "/dev/spidevN.0";
    for (uint8_t i=0; i<LINUX_SPI_NUM_BUSES; i++) {
        path[11] = '1' + i;
        _fd[i] = open(path, O_RDWR);
        if (_fd[i] == -1) {
            hal.scheduler->panic("SPIDriver: unable to open SPI bus");
        }
    }
    for (uint8_t i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        _device[i].init();
    }
}

void LinuxSPIDeviceManager::cs_assert(enum AP_HAL::SPIDevice type)
{
    uint8_t bus = 0, i;
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == type) {
            bus = _device[i]._bus;
            break;
        }
    }
    if (i == LINUX_SPI_DEVICE_NUM_DEVICES) {
        hal.scheduler->panic("Bad device type");
    }
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._bus != bus) {
            // not the same bus
            continue;
        }
        if (_device[i]._type != type) {
            if (_device[i]._cs->read() != 1) {
                hal.scheduler->panic("two CS enabled at once");
            }
        }
    }
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == type) {
            _device[i]._cs->write(0);
        }
    }
}

void LinuxSPIDeviceManager::cs_release(enum AP_HAL::SPIDevice type)
{
    uint8_t bus = 0, i;
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == type) {
            bus = _device[i]._bus;
            break;
        }
    }
    if (i == LINUX_SPI_DEVICE_NUM_DEVICES) {
        hal.scheduler->panic("Bad device type");
    }
    for (i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._bus != bus) {
            // not the same bus
            continue;
        }
        _device[i]._cs->write(1);
    }
}

void LinuxSPIDeviceManager::transaction(LinuxSPIDeviceDriver &driver, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    // we set the mode before we assert the CS line so that the bus is
    // in the correct idle state before the chip is selected
    ioctl(_fd[driver._bus], SPI_IOC_WR_MODE, &driver._mode);

    cs_assert(driver._type);
    struct spi_ioc_transfer spi[1];
    memset(spi, 0, sizeof(spi));
    spi[0].tx_buf        = (uint64_t)tx;
    spi[0].rx_buf        = (uint64_t)rx;
    spi[0].len           = len;
    spi[0].delay_usecs   = 0;
    spi[0].speed_hz      = driver._speed;
    spi[0].bits_per_word = driver._bitsPerWord;
    spi[0].cs_change     = 0;

    if (rx != NULL) {
        // keep valgrind happy
        memset(rx, 0, len);
    }

    ioctl(_fd[driver._bus], SPI_IOC_MESSAGE(1), &spi);
    cs_release(driver._type);
}
...
```

- `LinuxSPIDeviceManager` methods are implemented here.


- `LinuxSPIDeviceManager::init()` method try to open the `/dev/spidevN.0` path and write a file per bus and per num of devices.

-`LinuxSPIDeviceManager::cs_assert` enables devices.

- `LinuxSPIDeviceManager::cs_assert()` and `LinuxSPIDeviceManager::cs_release()` methods, like its names indicates, they are responsible of assert (`_device[i]._cs->write(0);`) or release (`_device[i]._cs->write(1);`) the chip select pin(_cs);
with corresponding checks.


- `LinuxSPIDeviceManager::transaction()` method handles, using the above methods, to write the data in the corresponding bus (`ioctl(_fd[driver._bus], SPI_IOC_MESSAGE(1), &spi);`).


```
...
/*
  return a SPIDeviceDriver for a particular device
 */
AP_HAL::SPIDeviceDriver *LinuxSPIDeviceManager::device(enum AP_HAL::SPIDevice dev)
{
    for (uint8_t i=0; i<LINUX_SPI_DEVICE_NUM_DEVICES; i++) {
        if (_device[i]._type == dev) {
            return &_device[i];
        }
    }
    return NULL;
}

/*
  return the bus specific semaphore
 */
AP_HAL::Semaphore *LinuxSPIDeviceManager::get_semaphore(uint8_t bus)
{
    return &_semaphore[bus];
}

#endif // CONFIG_HAL_BOARD
```
- This last two methods return specific devide SPIDeviceDriver and  bus semaphore.

