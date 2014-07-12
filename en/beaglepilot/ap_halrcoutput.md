# AP_HAL::RCOutput

The `AP_HAL::RCOutput` class is pure virtual and can be found in `/libraries/AP_HAL/RCOutput.h`. The RCOutput interface is based on the input related methods of the existing ArduPilot `APM_RC` class. RCOutput methods were separated from RCInput methods for clarity.

The output methods from `APM_RC` were reproduced here faithfully, with minor differences to naming. As an extension, the `read` and `write` methods are overloaded to also have versions which take or give arrays of values, as the most common use case is to read or write all channels sequentially, and bulk reads and writes may be more efficient in a threaded environment.
