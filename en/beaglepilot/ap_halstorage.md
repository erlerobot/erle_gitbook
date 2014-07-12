# AP_HAL::Storage

The `AP_HAL::Storage` class is pure virtual and can be found in `/libraries/AP_HAL/Storage.h`. It is the `AP_HAL` interface to take the place of the **AVR EEPROM**, possibly with other nonvolatile storage.

The `AP_HAL::Storage` interface very closely resembles the avr-libc eeprom interface. The use of `uint8_t*` projections into storage space are subject to change - it seems to make more sense to use integer types to designate these locations, as there is no valid dereference of a pointer value.
