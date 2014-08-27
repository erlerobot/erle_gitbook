# I2CDriver

Class that takes care of **I2C** (Inter-Integrated Circuit) serial bus communications.

---

**I2C** is a multi-master, multi-slave, single-ended, **serial computer bus**  and is used for attaching low-speed peripherals to a computer motherboard, embedded system such as a cellphone or microcontroller or other digital electronic devices.

The **I2C bus** consists of two bi-directional lines, one line for **data (SDA)** and one for **clock (SCL)**, by means of which a single master device can send informations serially to one ore more slave devices. To prevent any conflict every device hooked up to the bus has its own unique address. The standard I2C specifies two different addressing schema, 7 and 10 bits allowing at most 128 and 1024 devices connected at the same time.


---

**I2CDriver** supports the timeout features we require to assure safe timing properties when polling the hardware I2C peripheral in linux-based systems.

This class is divided into two files, **header** (`I2CDriver.h`) and **source code** (`I2CDriver.cpp`).

###I2CDriver.h


Link to the code:[I2CDriver.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/I2CDriver.h)

`Linux::I2CDriver`class defines the methods inherited from the [AP_HAL::I2CDriver](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/I2CDriver.h) abstract class for **read/write registers** through the **I2C** bus.

```cpp

#ifndef __AP_HAL_LINUX_I2CDRIVER_H__
#define __AP_HAL_LINUX_I2CDRIVER_H__

#include <AP_HAL_Linux.h>

class Linux::LinuxI2CDriver : public AP_HAL::I2CDriver {
public:
    LinuxI2CDriver(AP_HAL::Semaphore* semaphore, const char *device);

```
The `LinuxI2CDriver` class inherits from [AP_HAL::I2CDriver](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/I2CDriver.h).

```cpp
    void begin();
    void end();
    void setTimeout(uint16_t ms);
    void setHighSpeed(bool active);

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);
    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
    /* writeRegisters: write bytes to contigious registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                                   uint8_t len, uint8_t* data);

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);
    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);

    /* readRegister: read contigious device registers - writes the first
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data);

    uint8_t readRegistersMultiple(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t count,
                                  uint8_t* data);

    uint8_t lockup_count();

    AP_HAL::Semaphore* get_semaphore() { return _semaphore; }
    ...
    ```



 - The public members include functions for writing and reading from registers.


 - Semaphore method is used to controll the access to registers.


```cpp
...


private:
    AP_HAL::Semaphore* _semaphore;
    bool set_address(uint8_t addr);
    int _fd;
    uint8_t _addr;
    const char *_device;
};

#endif // __AP_HAL_LINUX_I2CDRIVER_H__
```

In the private members whe found a Semaphore method implementation and some varibles.


###I2CDriver.cpp


[I2CDriver.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/I2CDriver.cpp) basically,implements the methods defined  in `I2CDriver.h` header file.

```cpp

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "I2CDriver.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#ifndef I2C_SMBUS_BLOCK_MAX
#include <linux/i2c.h>
#endif
...
```
- In this piece of code `AP_HAL.h` and `GPIO.h` are included and the board is defined.

Some functions and libraries are included:

+ Header `sys/types.h` shall include definitions for types- [(sys/types.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/types.h.html)


+ The `sys/stat.h` header defines the structure of the data returned by the functions `fstat()`,` lstat()`, and `stat()`. - [(sys/stat.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html)


+ The `fcntl.h` header shall define some requests and arguments for use by the functions `fcntl()` and `open()`. - [(fcntl.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/fcntl.h.html)


+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)



+ `ioctl() `function manipulates the underlying device parameters of special files.You can read more in `man ioctl()` - [(sys/ioctl.h)](http://unix.superglobalmegacorp.com/Net2/newsrc/sys/ioctl.h.html)


+ `linuxi2c-dev.h`and `linux/i2c.h` headers contain options for dealing with i2c bus. - [(linux/i2c-dev.h)](https://www.kernel.org/pub/linux/kernel/people/marcelo/linux-2.4/include/linux/i2c-dev.h) and [(linux/i2c.h)](http://lxr.free-electrons.com/source/include/linux/i2c.h)

```cpp
...

using namespace Linux;

/*
  constructor
 */
LinuxI2CDriver::LinuxI2CDriver(AP_HAL::Semaphore* semaphore, const char *device) :
    _semaphore(semaphore),
    _fd(-1),
    _device(device)
{
}

/*
  called from HAL class init()
 */
void LinuxI2CDriver::begin()
{
    if (_fd != -1) {
        close(_fd);
    }
    _fd = open(_device, O_RDWR);
}

void LinuxI2CDriver::end()
{
    if (_fd != -1) {
        ::close(_fd);
        _fd = -1;
    }
}
...
```
- Here we find, first, a constructor for the Semaphore method.



- Then `begin()`and `end()`methods are implemented.


```cpp
...
/*
  tell the I2C library what device we want to talk to
 */
bool LinuxI2CDriver::set_address(uint8_t addr)
{
    if (_fd == -1) {
        return false;
    }
    if (_addr != addr) {
        ioctl(_fd, I2C_SLAVE, addr);
        _addr = addr;
    }
    return true;
}

void LinuxI2CDriver::setTimeout(uint16_t ms)
{
    // unimplemented
}

void LinuxI2CDriver::setHighSpeed(bool active)
{
    // unimplemented
}
...
```
- With `set_address`method we decide which device we want to talk to (to read or write).


- Then we find two unimplemented functions.

```cpp
...
uint8_t LinuxI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    if (::write(_fd, data, len) != len) {
        return 1;
    }
    return 0; // success
}


uint8_t LinuxI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                       uint8_t len, uint8_t* data)
{
    uint8_t buf[len+1];
    buf[0] = reg;
    if (len != 0) {
        memcpy(&buf[1], data, len);
    }
    return write(addr, len+1, buf);
}

/*
  this is a copy of i2c_smbus_access() from i2c-dev.h. We need it for
  platforms with older headers
 */
static inline __s32 _i2c_smbus_access(int file, char read_write, __u8 command,
                                      int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;
	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}

uint8_t LinuxI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    if (!set_address(addr)) {
        return 1;
    }
    union i2c_smbus_data data;
    data.byte = val;
    if (_i2c_smbus_access(_fd,I2C_SMBUS_WRITE, reg,
                         I2C_SMBUS_BYTE_DATA, &data) == -1) {
        return 1;
    }
    return 0;
}
...
```

- This slice of code implements the writing methods:
 + First check if `set_address`.
 + Then write. Note the use of `void * memcpy ( void * destination, const void * source, size_t num );`. This function copies the values of num bytes from the location pointed by source directly to the memory block pointed by destination.


- Note that the arguments passed are the device address, the data ang the data length (and the register if necessary). Then the method write the data.


```
...

uint8_t LinuxI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    if (::read(_fd, data, len) != len) {
        return 1;
    }
    return 0;
}

uint8_t LinuxI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
    if (_fd == -1) {
        return 1;
    }
    struct i2c_msg msgs[] = {
        {
        addr  : addr,
        flags : 0,
        len   : 1,
        buf   : (typeof(msgs->buf))&reg
        },
        {
        addr  : addr,
        flags : I2C_M_RD,
        len   : len,
        buf   : (typeof(msgs->buf))data,
        }
    };
    struct i2c_rdwr_ioctl_data i2c_data = {
    msgs : msgs,
    nmsgs : 2
    };

    // prevent valgrind error
    memset(data, 0, len);

    if (ioctl(_fd, I2C_RDWR, &i2c_data) == -1) {
        return 1;
    }

    return 0;
}


uint8_t LinuxI2CDriver::readRegistersMultiple(uint8_t addr, uint8_t reg,
                                              uint8_t len,
                                              uint8_t count, uint8_t* data)
{
    if (_fd == -1) {
        return 1;
    }
    while (count > 0) {
        uint8_t n = count>8?8:count;
        struct i2c_msg msgs[2*n];
        struct i2c_rdwr_ioctl_data i2c_data = {
        msgs : msgs,
        nmsgs : (typeof(i2c_data.nmsgs))(2*n)
        };
        for (uint8_t i=0; i<n; i++) {
            msgs[i*2].addr = addr;
            msgs[i*2].flags = 0;
            msgs[i*2].len = 1;
            msgs[i*2].buf = (typeof(msgs->buf))&reg;
            msgs[i*2+1].addr = addr;
            msgs[i*2+1].flags = I2C_M_RD;
            msgs[i*2+1].len = len;
            msgs[i*2+1].buf = (typeof(msgs->buf))data;
            data += len;
        };
        if (ioctl(_fd, I2C_RDWR, &i2c_data) == -1) {
            return 1;
        }
        count -= n;
    }
    return 0;
}


uint8_t LinuxI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    union i2c_smbus_data v;
    memset(&v, 0, sizeof(v));
    if (_i2c_smbus_access(_fd,I2C_SMBUS_READ, reg,
                          I2C_SMBUS_BYTE_DATA, &v)) {
        return 1;
    }
    *data = v.byte;
    return 0;
}
...
```

-  This other slice implements reading methods. Reading is done in the same way as writing.

```
...
uint8_t LinuxI2CDriver::lockup_count()
{
    return 0;
}
#endif // CONFIG_HAL_BOARD
```
- Finally this method ends the proccess.
