# AP_HAL::RCInput

The `AP_HAL::RCInput` class is pure virtual and can be found in `/libraries/AP_HAL/RCInput.h`. The RCInput interface is based on the input related methods of the existing ArduPilot `APM_RC` class. RCInput methods were separated from RCOutput methods for clarity.

The methods `uint8_t valid()` and `uint16_t read(uint8_t)` carry over the exact interface found in `APM_RC`, which is to specify the number of valid channels, and then read each channel out by number.

Based on the most common use case, which is to read the valid flag and then all input channels sequentially, a new interface `uint8_t read(uint16_t* periods, uint8_t len)` makes the valid flag available as the return value, and writes the periods as an array to the memory specified by the first argument. This bulk read interface may be more efficient in a threaded environment.
