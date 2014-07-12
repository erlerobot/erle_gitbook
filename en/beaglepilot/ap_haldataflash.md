# AP_HAL::Dataflash

The `AP_HAL::Dataflash` class is pure virtual and can be found in `/libraries/AP_HAL/Dataflash.h`. It is based on the existing ArduPilot `DataFlash` library found in `/libraries/DataFlash`. The `AP_HAL::Dataflash` interface is a cleaned up version of that library's interface. Public member variables have been replaced with getter methods, unused public interfaces have been removed, and all methods have had their names translated to lowercase and underscores for style.

Similar to the problems with the `SPI` library, the existing `DataFlash` library interface is oriented for byte-at-a-time writes to the hardware device. This interface may have to be revised to support bulk transfers for efficient use in a threaded environment.
