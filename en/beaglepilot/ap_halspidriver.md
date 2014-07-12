# AP_HAL::SPIDriver

The `AP_HAL::SPIDriver` class is pure virtual and can be found in `/libraries/AP_HAL/SPIDriver.h`.

The `AP_HAL::SPIDriver` class is derived from, but only slightly resembles the Arduino core's `SPI` library. Methods we don't use have been removed, as has the export of a global `SPI` object. The `begin` method has been swapped for a more idiomatic `init` method, and `setClockDivider` has been replaced with a more general `setSpeed` method. In addition, it is not possible to operate the `SPI` device as a *slave*. The `uint8_t transfer(uint8_t data)` method remains in common.

The `SPIDriver` interface (and the existing AVR implementation) will require significant changes to support efficient implementations in a threaded environment. Transfers will have to be possible in bulk, and somehow incorporate the notion of the target device so that specialized hardware can manage the slave device select lines. Currently, device select signals are sent using the `AP_HAL::GPIO::write` method, which is the AP_HAL analog of Arduino's `digitalWrite`.
