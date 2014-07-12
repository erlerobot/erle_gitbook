# AP_HAL::AnalogIn

The `AP_HAL::AnalogIn` class is pure virtual and can be found in `/libraries/AP_HAL/AnalogIn.h`. The pure virtual `AP_HAL::AnalogSource` class is also defined in that class.

The `AP_HAL::AnalogSource` interface is based loosely on the `AP_AnalogSource` interface in the existing AP_AnalogSource library. At this time, an `AP_HAL::AnalogSource` has a single method `float read()` which returns the latest analog measurment. There are no methods for flow control - currently you must assume the method will block until a measurment is ready; the timing is entirely implementation defined.

The `AP_HAL::AnalogIn` interface does not have a close analog in the existing libraries. Consider it to be, loosely, a factory for `AnalogSource` objects. There are only two methods: the standard `void init(void*)` initializer and a numbered interface to the available analog channels `AP_HAL::AnalogSource* channel(int n)`. At this time the significance of each numbered channel is determined by the implementation.

Extensions will be made to these interfaces as required to meet the needs of other platforms. We will also have to consider making named channels, as opposed to numbered channels, available from the `AP_HAL::AnalogIn` interface.
