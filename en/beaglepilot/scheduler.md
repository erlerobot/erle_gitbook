# Scheduler


Class that is designed to encapsulate **scheduling asynchronous processes** . Is responsible for managing the time of the processes and tasks.

---
**Scheduler** is an important functional component of multitasking and multiprocessing operating systems, and is essential in the real-time operating systems. Its function is to allocate the available time of a microprocessor between all processes that are available for execution.

A **real-time operating system** features to ensure that all programs will run at a maximum time limit. The Scheduler must behave in ways that this is true for any process.

In these cases, the purpose of the Scheduler is to **balance processor load**, preventing a process monopolize the processor or to be deprived of the resources of the machine. In real-time environments, such as devices for automatic control in industry (eg, robots), Scheduler also **prevents processes stop or interrupt others** who expect certain actions are performed. Their work is essential to keep the system stable and running.

---
This class is divided into two files, **header** (`Scheduler.h`) and **source code** (`Scheduler.cpp`).

For the case of Linux, the scheduler is implemented on top of the POSIX API, asumming pthreads and other nice interfaces that simplify the process of writting a real time system.



### Scheduler.h


Link to the code:[Scheduler.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Scheduler.h)

`Linux::LinuxScheduler`class defines the methods inherited from the [AP_HAL::Scheduler](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h) abstract class.

```cpp
#ifndef __AP_HAL_LINUX_SCHEDULER_H__
#define __AP_HAL_LINUX_SCHEDULER_H__

#include <AP_HAL_Linux.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <sys/time.h>
#include <pthread.h>

#define LINUX_SCHEDULER_MAX_TIMER_PROCS 10
...
```

- Imports `AP_HAL_Linux.h` and defines the board.


- The header`sys/time.h`defines time types, such as `timeval` structure that includes seconds and microsecons.- [(sys/time.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/systime.h.html)


- The `pthread.h` header defines symbols, functions like `pthread_mutex_unlock()` and types like `pthread_mutex_t.` - ([pthread.h](http://pubs.opengroup.org/onlinepubs/7908799/xsh/pthread.h.html))

```cpp
...
class Linux::LinuxScheduler : public AP_HAL::Scheduler {
public:
    LinuxScheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    uint32_t millis();
    uint32_t micros();
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void     begin_atomic();
    void     end_atomic();

    bool     system_initializing();
    void     system_initialized();

    void     panic(const prog_char_t *errormsg);
    void     reboot(bool hold_in_bootloader);

    void     stop_clock(uint64_t time_usec);

private:
    struct timespec _sketch_start_time;
    void _timer_handler(int signum);
    void _microsleep(uint32_t usec);

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;

    AP_HAL::Proc _failsafe;

    bool _initialized;
    volatile bool _timer_pending;

    volatile bool _timer_suspended;

    AP_HAL::MemberProc _timer_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    volatile bool _timer_event_missed;

    pthread_t _timer_thread_ctx;
    pthread_t _io_thread_ctx;
    pthread_t _uart_thread_ctx;

    void *_timer_thread(void);
    void *_io_thread(void);
    void *_uart_thread(void);

    void _run_timers(bool called_from_timer_thread);
    void _run_io(void);
    void _setup_realtime(uint32_t size);

    uint64_t stopped_clock_usec;
};

#endif // CONFIG_HAL_BOARD

#endif // __AP_HAL_LINUX_SCHEDULER_H__
```
- As we have seen ,the class `LinuxScheduler` is a class of class [AP_HAL::Scheduler](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h), that defines the methods and variables used is `Scheduler.cpp`.


### Scheduler.cpp

[Scheduler.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Scheduler.h) implements the methods defined  in `Scheduler.h` for managing time of processes.

```cpp
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "Scheduler.h"
#include "Storage.h"
#include "UARTDriver.h"
#include <sys/time.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/mman.h>

using namespace Linux;

extern const AP_HAL::HAL& hal;

#define APM_LINUX_TIMER_PRIORITY    13
#define APM_LINUX_UART_PRIORITY     12
#define APM_LINUX_MAIN_PRIORITY     11
#define APM_LINUX_IO_PRIORITY       10

LinuxScheduler::LinuxScheduler()
{}
...
```
- Imports `AP_HAL.h`and the `AP_HAL::HAL& hal` and defines the board.Also, imports `Scheduler.h`, `UARTDriver.h`(detailed in UARTDriver section) and `Storage.h`(explained in Storage section).


- Implements `LinuxScheduler()` and define priorities.

Some functions and libraries are included:



+ The header`sys/time.h`defines time types, such as `timeval` structure that includes seconds and microsecons.- [(sys/time.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/systime.h.html)


+ The `poll.h` header defines the `pollfd` structure that includes at least the following member:
int *fd*(the following descriptor being polled),
short int *events* (the input event flags) and
short int *revents * ( the output event flags) - [(poll.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/poll.h.html)


+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+ This header defines several general purpose functions, including dynamic memory management, random number generation, communication with the environment, integer arithmetics, searching, sorting and converting.- [(stdlib.h)](http://www.cplusplus.com/reference/cstdlib/?kw=stdlib.h)


+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)



+ C Header that defines the following macro:
`errno->Last error number (macro )`; plus at least three additional macro constants: EDOM, ERANGE and EILSEQ .`errno`deals with errors (see [errno](http://www.cplusplus.com/reference/cerrno/errno/) for more details). - [(errno.h)](http://www.cplusplus.com/reference/cerrno/?kw=errno.h)


+ This header includes memory management declarations.-[(sys/mman.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/mman.h.html)

```cpp
...
typedef void *(*pthread_startroutine_t)(void *);
...
```
-    `pthread_startroutine_t *routine` , stablish the routine to execute in a thread.

```cpp
...
/*
  setup for realtime. Lock all of memory in the thread and pre-fault
  the given stack size, so stack faults don't cause timing jitter
 */
void LinuxScheduler::_setup_realtime(uint32_t size)
{
        uint8_t dummy[size];
        mlockall(MCL_CURRENT|MCL_FUTURE);
        memset(dummy, 0, sizeof(dummy));
}
```

- `mlockall`belongs to `sys/mman.h`. The function `mlockall()` causes all of the pages mapped by the address space of a process to be memory resident until unlocked or until the process exits or execs another process image.


-  `void * memset ( void * ptr, int value, size_t num )` sets the first num bytes of the block of memory pointed by ptr to the specified value (interpreted as an unsigned char).

```cpp
...
void LinuxScheduler::init(void* machtnichts)
{
    clock_gettime(CLOCK_MONOTONIC, &_sketch_start_time);

    _setup_realtime(32768);

    pthread_attr_t thread_attr;
    struct sched_param param;

    memset(&param, 0, sizeof(param));

    param.sched_priority = APM_LINUX_MAIN_PRIORITY;
    sched_setscheduler(0, SCHED_FIFO, &param);

    param.sched_priority = APM_LINUX_TIMER_PRIORITY;
    pthread_attr_init(&thread_attr);
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_timer_thread_ctx, &thread_attr, (pthread_startroutine_t)&Linux::LinuxScheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
    pthread_attr_init(&thread_attr);
    param.sched_priority = APM_LINUX_UART_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_uart_thread_ctx, &thread_attr, (pthread_startroutine_t)&Linux::LinuxScheduler::_uart_thread, this);

    // the IO thread runs at lower priority
    pthread_attr_init(&thread_attr);
    param.sched_priority = APM_LINUX_IO_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

    pthread_create(&_io_thread_ctx, &thread_attr, (pthread_startroutine_t)&Linux::LinuxScheduler::_io_thread, this);
}
...
```
- From `time.h` the `int clock_gettime(clockid_t clk_id, struct timespec *tp);`funtion retrieve the time of the specified clock clk_id.

- The `pthread_attr_t type should be treated as opaque: any access to the object other than via pthreads functions is nonportable and produces undefined results.


- ` int sched_setscheduler(pid_t pid, int policy,const struct sched_param *param);`.
The `sched_setscheduler()` system call sets both the scheduling policy and parameters for the thread whose ID is specified in pid.  If pid equals zero, the scheduling policy and parameters of the calling thread will be set. This funtions is used with MAIN, to stablish the priority.


- Now, we attempt to stablish priority ans attributes for TIMER.The function `pthread_attr_init() `initialises a thread attributes object attr with the default value for all of the individual attributes used by a given implementation.


-  ` int pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param);` function sets the scheduling parameter attributes of the thread attributes object referred to by attr to the values specified in the buffer pointed to by param.Then the `int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);` function sets the scheduling policy
 attribute of the thread attributes object referred to by attr to the value specified in policy.


- The `pthread_create()` function is used to create a new thread, with attributes specified by attr, within a process.


- Then, threads with a determined priority and attributes are stablished for UART and IO proccess.


```cpp
...
void LinuxScheduler::_microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}
...
```
- `nanosleep()` suspends the execution of the calling thread until either at least the time specified.`nanosleep()` returns -1, sets errno to EINTR, and writes the remaining time into the structure pointed  by the second parameter `&ts`.

```cpp
...
void LinuxScheduler::delay(uint16_t ms)
{
    if (stopped_clock_usec) {
        stopped_clock_usec += 1000UL*ms;
        return;
    }
    uint32_t start = millis();

    while ((millis() - start) < ms) {
        // this yields the CPU to other apps
        _microsleep(1000);
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}
...
```
- Delay a process for the specified ms.

```cpp
...

uint32_t LinuxScheduler::millis()
{
    if (stopped_clock_usec) {
        return stopped_clock_usec/1000;
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
                  (_sketch_start_time.tv_sec +
                   (_sketch_start_time.tv_nsec*1.0e-9)));
}

uint32_t LinuxScheduler::micros()
{
    if (stopped_clock_usec) {
        return stopped_clock_usec;
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
                  (_sketch_start_time.tv_sec +
                   (_sketch_start_time.tv_nsec*1.0e-9)));
}
...
```
- This functions change units:
```
micro > 10^-6
mili > 10^-3
```

```cpp
...
void LinuxScheduler::delay_microseconds(uint16_t us)
{
    _microsleep(us);
}

void LinuxScheduler::register_delay_callback(AP_HAL::Proc proc,
                                             uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

...
```
- The `LinuxScheduler::delay_microseconds`calls `_microsleep`with a specified time in ms.


- The `LinuxScheduler::register_delay_callback` call `_delay`function with a specified time on specified procces.

```cpp
...
void LinuxScheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < LINUX_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}
...
```
- This funtion assigns the procces to the timer control till the max. number of timer processes (LINUX_SCHEDULER_MAX_TIMER_PROCS 10 from `Scheduler.h`).

- This function registers a high prioeity timer task.


```cpp
...
void LinuxScheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < LINUX_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}
...
```
- This function does the same as the previous one, but with the IO processes.IO task is registered as low priority task.

```cpp
...

void LinuxScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void LinuxScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
    while (_in_timer_proc) {
        usleep(1);
    }
}
void LinuxScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
    while (_in_timer_proc) {
        usleep(1);
    }
}

void LinuxScheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void LinuxScheduler::_run_timers(bool called_from_timer_thread)
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i] != NULL) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != NULL) {
        _failsafe();
    }
    in_timer_proc = false;
}

void *LinuxScheduler::_timer_thread(void)
{
    _setup_realtime(32768);
    while (system_initializing()) {
        poll(NULL, 0, 1);
    }
    while (true) {
        _microsleep(1000);

        // run registered timers
        _run_timers(true);

    }
    return NULL;
}
...
```
 - This piece of code, implements the function for dealing with timers.


- Note that `suspend_timer_procs()` involve suspending and resuming both IO and timer.

```cpp
...

void LinuxScheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i] != NULL) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void *LinuxScheduler::_uart_thread(void)
{
    _setup_realtime(32768);
    while (system_initializing()) {
        poll(NULL, 0, 1);
    }
    while (true) {
        _microsleep(10000);

        // process any pending serial bytes
        ((LinuxUARTDriver *)hal.uartA)->_timer_tick();
        ((LinuxUARTDriver *)hal.uartB)->_timer_tick();
        ((LinuxUARTDriver *)hal.uartC)->_timer_tick();
    }
    return NULL;
}

void *LinuxScheduler::_io_thread(void)
{
    _setup_realtime(32768);
    while (system_initializing()) {
        poll(NULL, 0, 1);
    }
    while (true) {
        _microsleep(20000);

        // process any pending storage writes
        ((LinuxStorage *)hal.storage)->_timer_tick();

        // run registered IO processes
        _run_io();
    }
    return NULL;
}
...
```
- Define the threads for UART and IO processes.


- `poll()`  waits for one of a set of file descriptors to become ready to perform I/O.


```cpp
...
void LinuxScheduler::panic(const prog_char_t *errormsg)
{
    write(1, errormsg, strlen(errormsg));
    write(1, "\n", 1);
    hal.scheduler->delay_microseconds(10000);
    exit(1);
}
...
```
- This funtion prints a error-messge and delay the process on going.

```cpp
...

bool LinuxScheduler::in_timerprocess()
{
    return _in_timer_proc;
}

void LinuxScheduler::begin_atomic()
{}

void LinuxScheduler::end_atomic()
{}

bool LinuxScheduler::system_initializing() {
    return !_initialized;
}

void LinuxScheduler::system_initialized()
{
    if (_initialized) {
        panic("PANIC: scheduler::system_initialized called more than once");
    }
    _initialized = true;
}

void LinuxScheduler::reboot(bool hold_in_bootloader)
{
    for(;;);
}

void LinuxScheduler::stop_clock(uint64_t time_usec)
{
    stopped_clock_usec = time_usec;
}

#endif // CONFIG_HAL_BOARD
```
- The missing function appearing in `Scheluder.h`are implemented here.


- The `stop_clock`function can optionally stop the clock at a given time.
