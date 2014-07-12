# AP_HAL
### Overview



`AP_HAL` is hardware abstraction layer for the `ArduPilot` project. The goal is to separate AVR (via avr-libc) and Arduino (via the Arduino core and included libraries like SPI and Wire) dependencies from the ArduPilot programs and libraries.

This will make it possible to port the ArduPlane and ArduCopter programs (and their associated libraries) to a new platform without changing any of the ArduPlane or ArduCopter code, only implementing the `AP_HAL` interface for the new platform.

Currently, the `AP_HAL_AVR` library , found in `/libraries/AP_HAL_AVR/`, is an implementation of `AP_HAL` for the `ArduPilot` hardware platforms.

`AP_HAL_AVR` exports two HAL instances, `AP_HAL_AVR_APM1` and `AP_HAL_AVR_APM2`, which provide the instances for APM1 and APM2 hardware.

###Requirments

`AP_HAL` is designed to work with the `Arduino 1.0.1 IDE` with a special "Coreless" modification, or with the `ArduPilot Arduino.mk makefiles` with a similar Coreless extension. The Arduino.mk Makefile in the `AP_HAL` development branch also implements the coreless modification. You will not need to upgrade to a coreless patched IDE if you are just using Makefiles.

The Coreless modification to the `Arduino IDE` will be made available as a patch and as compiled IDEs on the ArduPilot project's Downloads section.

###Why Coreless Arduino

todo

###Consequences of Coreless Arduino

- Need to provide an empty "Arduino.h" somewhere in the include path of each sketch. The line #include <Arduino.h> is added to each sketch by the Arduino IDE's sketch preprocessor, as well as by the Makefile build.

- Need to provide an entry point int main(void) for the application. Previously, a trivial implementation existed in the Arduino core:

      int main(void) {
          init(); /* Initialized the Arduino core functionality */
          setup();
          while(1) loop();
          return 0; /* Never reached */
      }

    Each program built with the coreless Arduino build will need to provide its own main function. The following implementation may be used in at the bottom of the sketch PDE/INO file:

      extern "C" {
          int main(void) {
              hal.init(NULL);
              setup();
              while(1) loop();
              return 0;
          }
      }

    The `extern "C"` wrapper is required because a PDE/INO file is compiled as C++.


- Global objects which were previously available in all Arduino sketches, such as Serial, Serial1, etc. no longer exist. This is by design - all of those hardware interfaces should be accessed through the HAL.

###Using The AP_HAL Library

`AP_HAL`, found in `/libraries/AP_HAL/`, is a library of purely virtual classes: there is no concrete code in the `AP_HAL` library, only interfaces. All code in the core `ArduPlane & ArduCopter` programs, `ArduPilot` libraries, and example sketches should depend only on the interfaces exposed by `AP_HAL`.

The collection of classes in the `AP_HAL library` exist in the `AP_HAL C++ namespace`. The convention is for a program to instantiate a single instance of the `AP_HAL::HAL class`, under a reference to the name hal. `#include <AP_HAL.h> const AP_HAL::HAL& hal = specific_hal_implementation;` This instance should be made in a single object file. All other object files, including libraries (even those inside an `AP_HAL implementation`, should use the `AP_HAL interface` by declaring an extern reference to hal. `#include <AP_HAL.h> extern const AP_HAL::HAL& hal;`

###Using The AP_HAL_AVR library

The `AP_HAL_AVR library` exports `AP_HAL::HAL instances` for the `APM1` and `APM2`. These instances are made of a number of concrete classes which implement the `AP_HAL interfaces` for the `AVR platform`. These implementations depend only on `avr-libc` and not the `Arduino core`.

Some of the code in `AP_HAL_AVR`, such as the the `GPIO class's pinMode`, `read`, and `write`, has been derived directly from the source code of the `Arduino core pinMode`, `digitalRead`, and `digitalWrite`.

When using the coreless Arduino IDE to build for AVR, you will need the following three libraries included in the top level of your sketch:
```
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
```
and then declare one of the following hal lines depending on your platform:
```
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
```
or
```
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
```
###AP_HAL Library Contents

The `AP_HAL library` is organized as follows:

`AP_HAL.h`exports all other headers for the library.

`AP_HAL_Namespace.h` : exposes the `C++ namespace AP_HAL`. The namespace declaration declares each class by name (not implementation) and some useful typedefs.

The `AP_HAL interface classes` are each defined in a header file bearing their name.
