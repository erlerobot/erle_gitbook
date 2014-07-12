# AP_HAL::Scheduler

The `AP_HAL::Scheduler` class is pure virtual and can be found in `/libraries/AP_HAL/Scheduler.h`. The `AP_HAL::Scheduler` interface is designed to encapsulate both the timing utilities previously provided by the Arduino core (i.e. `millis` and `delay`), as well as scheduling asynchronous processes as a replacment to the ArduPilot `AP_PeriodicProcess` driver.

The following methods are exposed by the `AP_HAL::Scheduler` interface:

- **`void init(void* implspecific)`** : Initializer should setup all hardware. This should be the very first initializer called by `AP_HAL::HAL::init`.
- **`void delay(uint32_t ms)`** : Duplicates Arduino core `delay`. Will call delay callback, if registered.
- **`uint32_t millis()`** : Duplicates Arduino core `millis`
- **`uint32_t micros()`** : Duplicates Arduino core `micros`
- **`void delay_microseconds(uint16_t us)`** : Duplicates Arduino core `delayMicros`.
- **`void register_delay_callback(AP_HAL::Proc, uint16_t min_ms)`**: Register a callback to be used during calls to `delay`. Callback will be called at a 1ms period. Second argument is the minimum length of time expected to delay - set this to the ceiling of the runtime for the callback.
- **`void register_timer_process(AP_HAL::TimedProc)`** : Duplicates `AP_PeriodicProcess::register_process`
- **`void register_timer_failsafe(AP_HAL::TimedProc, uint32_t period_us)`** : Duplicates `AP_PeriodicProcess::set_failsafe`
- **`void suspend_timer_procs()`** : Duplicates `AP_PeriodicProcess::suspend_timer`
- **`void resume_timer_procs()`** : Duplicates `AP_PeriodicProcess:resume_timer`
