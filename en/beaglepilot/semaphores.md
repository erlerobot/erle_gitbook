# Semaphores

 Class that take care of **Semaphores** in **linux-based** systems.

---
A **semaphore** is a variable or abstract data type that is used for controlling access, by multiple processes, to a common resource in a parallel programming or a multi user environment.

In computer science, **locks mutex** is a synchronization mechanism that limits access to a shared by multiple processes or threads in a concurrent execution environment resource, allowing the **mutual exclusion**.

A **mutex** is a **mutual exclusion semaphore**, a special variant of a semaphore that only allows one **locker** at a time and whose ownership restrictions may be more stringent than a normal semaphore.

When an element is shared by more than one thread, race conditions can occur if it is not properly protected. The simplest mechanism for protection is to close or **lock**. Generally when you should protect a set of elements, it is associated with a lock. Each process / thread, to access an element of the set, must block, which becomes its **owner**. That's the only way to gain access. Upon completion of use, the owner must unlock to allow another process / thread can take it in turn.

---
This class is divided into two files, **header** (`Semaphores.h`) and **source code** (`Semaphores.cpp`).

### Semaphores.h


Link to the code:[Semaphores.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Semaphores.h)

`Linux::LinuxSemaphore`class defines the methods inherited from the [AP_HAL::Semaphores](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Semaphores.h) abstract class.

```cpp

#ifndef __AP_HAL_LINUX_SEMAPHORE_H__
#define __AP_HAL_LINUX_SEMAPHORE_H__

#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux.h>
#include <pthread.h>
...
```
- The board is defined and the `AP_HAL_linux.h`is imported.


- The `pthread.h` header defines symbols, functions like `pthread_mutex_unlock()` and types like `pthread_mutex_t.` - [(pthread.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/pthread.h.html)
 + The `pthread_mutex_init()` function initialises the mutex referenced by mutex with attributes specified by attr.
 + The mutex object referenced by mutex shall be locked by calling `pthread_mutex_lock()`. If the mutex is already locked, the calling thread shall block until the mutex becomes available.On success, `pthread_mutex_trylock()` returns 0.
 + The `pthread_mutex_unlock()` function attempts to unlock the specified mutex. If there are threads blocked on the mutex object when `pthread_mutex_unlock()` is called, resulting in the mutex becoming available, the scheduling policy is used to determine which thread acquires the mutex.
 + *REMEMBER*: a **thread** of execution is the smallest sequence of programmed instructions that can be managed independently by a scheduler.

```cpp
...
class Linux::LinuxSemaphore : public AP_HAL::Semaphore {
public:
    LinuxSemaphore() {
        pthread_mutex_init(&_lock, NULL);
    }
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    pthread_mutex_t _lock;
};
#endif // CONFIG_HAL_BOARD

#endif // __AP_HAL_LINUX_SEMAPHORE_H__
```
- The `LinuxSemaphore `class inherits from [AP_HAL::Semaphore](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/Semaphores.cpp).


- The `pthread_mutex_init()` function called in the constructor initialises the mutex referenced by passing the address of `pthread_mutex_t type` object (`_lock`) with **attributes** specified by the second parameter. If the second parameter is `NULL`, the default mutex attributes are used. Upon successful initialisation, the state of the mutex becomes **initialised and unlocked**.


- The other methods will be implemented in `Semaphores.cpp`.

### Semaphores.cpp


[Semaphores.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Semaphores.h) implements the methods defined  in `Semaphores.h`.

```cpp
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;
...
```

- Imports `AP_HAL.h` and ` Semaphores.h `. Defines a board and a namespace. Imports the const ` AP_HAL::HAL& hal`.

```cpp
...
bool LinuxSemaphore::give()
{
    return pthread_mutex_unlock(&_lock) == 0;
}
...
```
- Implements the `give()`method. `pthread_mutex_unlock(&_lock)`unlocks the specified mutex, referred by the address object `&_lock`.

```cpp
...
bool LinuxSemaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        return pthread_mutex_lock(&_lock) == 0;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint32_t start = hal.scheduler->micros();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((hal.scheduler->micros() - start) < timeout_ms*1000);
    return false;
}
...
```
- Note that the argument passed to `take()`is `timeout_ms`, which will contain the ms when the semaphore will be unlock. The sequence of `take()` method is, more or less, the following:
 + If timeout is == 0 then lock the mutex.
 + If success on blocking (0 is returned) returns true.
 + If the timeout is not reached continue blocking.

```cpp
...

bool LinuxSemaphore::take_nonblocking()
{
    return pthread_mutex_trylock(&_lock) == 0;
}

#endif // CONFIG_HAL_BOARD
```
- The `pthread_mutex_trylock()` function shall return zero if a lock on the mutex object referenced by mutex is acquired. Otherwise, an error number is returned to indicate the error.

