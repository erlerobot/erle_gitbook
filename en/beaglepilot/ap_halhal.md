# AP_HAL::HAL

The `AP_HAL::HAL` class is a container for the a complete set of device drivers. The class is defined in `/libraries/AP_HAL/HAL.h`. It also has a virtual (i.e. overridable) method to handle driver initialization. Each device driver is exposed as a pointer an `AP_HAL` driver class, (e.g. each serial driver is exposed as a public `UARTDriver* uartN`).

The following drivers are public members of `AP_HAL::HAL`. (Each class is in the `AP_HAL namespace`, left off for brevity.)

- **`UARTDriver* uart0`** : Corresponds to ArduPilot FastSerial library override of the Arduino core Serial object
- **`UARTDriver* uart1`** : Corresponds to Serial1 object (as above)
- **`UARTDriver* uart2`** : Corresponds to Serial2 object (as above)
- **`UARTDriver* uart3`** : Corresponds to Serial3 object (as above)
- **`I2CDriver* i2c`** : Corresponds to ArduPilot `/libraries/I2C` driver
- **`SPIDriver* spi`** : Corresponds to Arduino library SPI object
- **`AnalogIn* analogin`** : Corresponds to `/libraries/AP_AnalogSource/AP_AnalogSource_Arduino` driver
- **`Storage* storage`** : Corresponds to avr-libc's `<avr/eeprom.h>` driver
- **`Dataflash* dataflash`** : Corresponds to ArduPilot `/libraries/DataFlash` driver
- **`ConsoleDriver* console`** : New utility for warning and error reporting
- **`GPIO* gpio`** : Corresponds to Arduino core `pinMode`, `digitalRead`, and `digitalWrite` functionality
- **`RCInput* rcin`** : Corresponds to PPM input side of ArduPilot `/libraries/APM_RC` library
- **`RCOutput* rcout`** : Corresponds to PPM output side of ArduPilot `/libraries/APM_RC` library
- **`Scheduler* scheduler`** : Encompasses both Arduino core timing functions such as millis and delay and the ArduPilot `/library/AP_PeriodicProcess/` driver.

`AP_HAL` also has an unimplemented `virtual void init(void* opts) const` method. This method should initialize each driver before the call to a sketch's setup method.
