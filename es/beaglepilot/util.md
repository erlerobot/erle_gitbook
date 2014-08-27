# Util


Class that implements the methods in [/AP_HAL/utility](https://github.com/BeaglePilot/ardupilot/tree/master/libraries/AP_HAL/utility).

---
**Utility** software is system software designed to help analyze, configure, optimize or maintain the system.

In the [/AP_HAL/Util.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/Util.h) we find  Util member for string utilities, a system clock, a state of safety controller... but also we through here everything that doesn't have an specific place within the system. New modules might end up being here.

---

This class is divided into two files, **header** (`Util.h`) and **source** (`Util.cpp`).

### Util.h


Link to the code:[Util.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Util.h)

The `Linux::LinuxUtil` class inherits from [AP_HAL::Util](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/Util.h).

In the folder [AP_HAL/utility](https://github.com/BeaglePilot/ardupilot/tree/master/libraries/AP_HAL/utility) we can find  some utilities that are implemented in `Util.h` and `Util.cpp`.

```cpp
#ifndef __AP_HAL_LINUX_UTIL_H__
#define __AP_HAL_LINUX_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_Linux_Namespace.h"
...
```
- As usual imports `AP_HAL.h `and also `AP_HAL_Linux_Namespace.h`.

```cpp
...
class Linux::LinuxUtil : public AP_HAL::Util {
public:
    void init(int argc, char * const *argv) {
        saved_argc = argc;
        saved_argv = argv;
    }

    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }

    /**
       return commandline arguments, if available
     */
    void commandline_arguments(uint8_t &argc, char * const *&argv);

private:
    int saved_argc;
    char* const *saved_argv;
};

#endif // __AP_HAL_LINUX_UTIL_H__
```


- The `Linux::LinuxUtil`class is defined as class of [AP_HAL::Util](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/Util.h).


- The `run_debug_shell` takes as argument an ` AP_HAL::BetterStream` which is a pure virtual interface.` BetterStream` provided some implementations for AVR based on [_vprintf()](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/utility/print_vprintf.h).


- `run a debug shell` on the given stream if possible. This is used to support dropping into a debug shell to run firmware upgrade commands.


- The `commandline_arguments` returns commandline arguments, if available. This method should be implemented in `Util.cpp`.

### Util.cpp


[Util.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Util.cpp) implements the methods defined  in `Util.h`.

```cpp
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
using namespace Linux;
...
```
- Includes `AP_HAL.h` and `Util.h`. Defines the board.

Some functions and libraries are included:

+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)


+ This header defines macros to access the individual arguments of a list of unnamed arguments whose number and types are not known to the called function.- [(stdarg)](http://www.cplusplus.com/reference/cstdarg/)


+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+ This header defines several general purpose functions, including dynamic memory management, random number generation, communication with the environment, integer arithmetics, searching, sorting and converting.- [(stdlib.h)](http://www.cplusplus.com/reference/cstdlib/?kw=stdlib.h)



+ C Header that defines the following macro:
`errno->Last error number (macro )`; plus at least three additional macro constants: EDOM, ERANGE and EILSEQ .`errno`deals with errors (see [errno](http://www.cplusplus.com/reference/cerrno/errno/) for more details). - [(errno.h)](http://www.cplusplus.com/reference/cerrno/?kw=errno.h)

```cpp
...

/**
   return commandline arguments, if available
*/
void LinuxUtil::commandline_arguments(uint8_t &argc, char * const *&argv)
{
    argc = saved_argc;
    argv = saved_argv;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
```
- The `commandline_arguments` method is implemented.This method returns the commandline arguments in the variables: `argc` and `argv` passed by reference.

