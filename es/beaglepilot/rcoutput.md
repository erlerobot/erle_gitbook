# RCOutput


Class that takes care of **RC** (Radio Control) output signals processing, in **linux-based** systems, using Pulse-width modulation.

---
**Pulse-width modulation (PWM)**, or pulse-duration modulation (PDM), is a modulation technique that controls the width of the pulse, formally the pulse duration, based on modulator signal information. Although this modulation technique can be used to encode information for transmission, its main use is to allow the control of the power supplied to electrical devices, especially to inertial loads such as motors.

**It's important to hightlight that ardupilot uses a special PWM signals that take as inputs pulse with widths from 1000 to 2000 us.**

---

This class is divided into two files, **header** (`RCOutput.h`) and **source code** (`RCOutput.cpp`).

###RCOutput.h


Link to the code:[RCOutput.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/RCOutput.h)

`Linux::LinuxRCOutput`defines the methods inherited from the [AP_HAL::RCOutput](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/RCOutput.h) abstract class, for handling  **Radio Control (RC)** output signals generation like as the sent to the motors.

```cpp
#ifndef __AP_HAL_LINUX_RCOUTPUT_H__
#define __AP_HAL_LINUX_RCOUTPUT_H__

#include <AP_HAL_Linux.h>
#define PRUSS_SHAREDRAM_BASE     0x4a310000
#define MAX_PWMS                 12
#define PWM_CMD_MAGIC            0xf00fbaaf
#define PWM_REPLY_MAGIC          0xbaaff00f
#define TICK_PER_US              200
#define TICK_PER_S               200000000
#define PWM_CMD_CONFIG	         0	/* full configuration in one go */
#define PWM_CMD_ENABLE	         1	/* enable a pwm */
#define PWM_CMD_DISABLE	         2	/* disable a pwm */
#define PWM_CMD_MODIFY	         3	/* modify a pwm */
#define PWM_CMD_SET	         4	/* set a pwm output explicitly */
#define PWM_CMD_CLR	         5	/* clr a pwm output explicitly */
#define PWM_CMD_TEST	         6	/* various crap */

...
```

- The `AP_HAL_linux.h ` is imported and definitions for `PWM_CMD` are implemented.

```cpp
...

class Linux::LinuxRCOutput : public AP_HAL::RCOutput {
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    struct pwm_cmd {
        uint32_t magic;
        uint32_t enmask;     /* enable mask */
        uint32_t offmsk;     /* state when pwm is off */
        uint32_t periodhi[MAX_PWMS][2];
        uint32_t hilo_read[MAX_PWMS][2];
        uint32_t enmask_read;
    }*sharedMem_cmd;

};

#endif // __AP_HAL_LINUX_RCOUTPUT_H__
```
- The `LinuxRCOutput`inherits from [AP_HAL::RCOutput](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/RCOutput.h), where the channels names are defined.


- The methods defined here, will be implemented in `RCOutput.cpp`, we will comment them later.


- The private memebers include a struct containing some integer type with a minimum of 32 bits.The `pwd_cmd`struct of type `*sharedMem_cmd` is used *to save the readings of the channels in memory* .

###RCOutput.cpp


[RCOutput.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/RCOutput.cpp) implements the methods defined  in `RCOutput.h` header file for read/write channels, enable/disable channels and set/get frequency in linux-based systems.

```cpp
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <signal.h>
...
```
- This slice of code imports `AP_HAL.h` and defines the board.

Some functions and libraries are included:

+ `RCOutput.h`header file is included.


+ Header `sys/types.h` shall include definitions for types- [(sys/types.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/types.h.html)



+ The `sys/stat.h` header defines the structure of the data returned by the functions `fstat()`,` lstat()`, and `stat()`. - [(sys/stat.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html)



+ The `fcntl.h` header shall define some requests and arguments for use by the functions `fcntl()` and `open()`. - [(fcntl.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/fcntl.h.html)



+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+ The internal format of directories is unspecified.
The `dirent.h` header defines  data type through typedef.(e.g.: A type representing a directory stream.) - [(dirent.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/dirent.h.html)



+ This header defines several general purpose functions, including dynamic memory management, random number generation, communication with the environment, integer arithmetics, searching, sorting and converting.- [(stdlib.h)](http://www.cplusplus.com/reference/cstdlib/?kw=stdlib.h)


+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)


+ This header defines a set of integral type aliases with specific width requirements, along with macros specifying their limits and macro functions to create values of these types.- [(stdint.h)](http://www.cplusplus.com/reference/cstdint/?kw=stdint.h)


+ `ioctl() `function manipulates the underlying device parameters of special files.You can read more in `man ioctl()` - [(sys/ioctl.h)](http://unix.superglobalmegacorp.com/Net2/newsrc/sys/ioctl.h.html)


+ `linux/spi/spidev.h` offers user space versions of kernel symbols for SPI clocking modes(*see [SPIDriver](./spidriver.md)). - [(linux/spi/spidev.h)](http://www.cs.fsu.edu/~baker/devices/lxr/http/source/linux/include/linux/spi/spidev.h)


+ This header includes memory management declarations.-[(sys/mman.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/mman.h.html)


+ Some running environments use signals to inform running processes of certain events.This header is the C library to handle signals. - [(signals.h)](http://www.cplusplus.com/reference/csignal/?kw=signal.h)


```cpp
...
using namespace Linux;


#define PWM_CHAN_COUNT 12

static const uint8_t chan_pru_map[]= {10,8,11,9,7,6,5,4,3,2,1,0};                //chan_pru_map[CHANNEL_NUM] = PRU_REG_R30/31_NUM;
static const uint8_t pru_chan_map[]= {11,10,9,8,7,6,5,4,1,3,0,2};                //pru_chan_map[PRU_REG_R30/31_NUM] = CHANNEL_NUM;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
static void catch_sigbus(int sig)
{
    hal.scheduler->panic("RCOutput.cpp:SIGBUS error gernerated\n");
}
...
```
- Define two arrays of channels and a board.


- Try to catch a sigbus, SIGBUS is the signal sent by a program when an error has occurred on the bus. Print a message when caugth.

```cpp
...

void LinuxRCOutput::init(void* machtnicht)
{
    uint32_t mem_fd;
    signal(SIGBUS,catch_sigbus);
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, PRUSS_SHAREDRAM_BASE);
    close(mem_fd);
}
...
```
- Implementation of `init()` method: Open the memory directory `/dev/men/` and map the struct `sharedMem_cmd` into the memory.

```cpp
...
void LinuxRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    uint8_t i;
    unsigned long tick=TICK_PER_S/(unsigned long)freq_hz;
    for(i=0;i<12;i++){
        if(chmask&(1<<i)){
            sharedMem_cmd->periodhi[chan_pru_map[i]][0]=tick;
        }
    }

    uint16_t LinuxRCOutput::get_freq(uint8_t ch)
{
    return TICK_PER_S/sharedMem_cmd->periodhi[chan_pru_map[ch]][0];
}
...
```

- This methods handle output frequency (1/period) control.

```cpp
...
void LinuxRCOutput::enable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void LinuxRCOutput::disable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask &= !(1U<<chan_pru_map[ch]);
}
...
```
- This methods implements the output active/highZ control, either by single channel at a time or a mask of channels.They allow you to enable or disable channels.Note the use of 1U for the left shift, to make it unsigned.

```cpp
...

void LinuxRCOutput::write(uint8_t ch, uint16_t period_us)
{
    sharedMem_cmd->periodhi[chan_pru_map[ch]][1] = TICK_PER_US*period_us;
}

void LinuxRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        write(ch+i,period_us[i]);
    }
}
...
```
- Output, either single channel or bulk array of channels.

```cpp
...
uint16_t LinuxRCOutput::read(uint8_t ch)
{
    return (sharedMem_cmd->hilo_read[chan_pru_map[ch]][1]/TICK_PER_US);
}

void LinuxRCOutput::read(uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        period_us[i] = sharedMem_cmd->hilo_read[chan_pru_map[i]][1]/TICK_PER_US;
    }
}

#endif
```
- Read back current output state, as either single channel or array of channels.
