# AP_HAL::I2CDriver

The `AP_HAL::I2CDriver` class is the `AP_HAL` replacment for ArduPilot's `I2C` library. (ArduPilot's `I2C` library is a replacment for the Arduino provided `Wire` library. The `Wire` library is so bad I could cry.)

The `AP_HAL::I2CDriver` class is a pure virtual interface, found in `/libraries/AP_HAL/I2CDriver.h`. The methods resemble those in defined by the `I2C` class in `/libraries/I2C/I2C.h`, but to ease future implementations, all of the old Arduino `Wire` class compatibility methods have been dropped.

The `I2CDriver` interface supports the timeout features we require to assure safe timing properties when polling the hardware I2C peripeheral. It also only exposes whole-transaction interfaces to the user, to support more efficient implementations in a threaded environment.
