# AP_HAL_Linux

The `AP_HAL_Linux` is a subclass of `AP_HAL` that implements a set of classes to be able to run `ardupilot` in Linux-based systems. The code from the `AP_HAL_Linux` can be found at [/libraries/AP_HAL_Linux/](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/AP_HAL_Linux.h),

According to the code, the this HAL is described as:

> Umbrella header for `AP_HAL_Linux` module.
> The module header exports singleton instances which must conform the `AP_HAL::HAL` interface. It may only expose implementation details (class
> names, headers) via the Linux namespace.
> The class implementing `AP_HAL::HAL` should be called `HAL_Linux` and exist
> in the global namespace. There should be a single const instance of the
> `HAL_Linux` class called `AP_HAL_Linux`, instantiated in the HAL_Linux_Class.cpp
> and exported as `extern const HAL_Linux AP_HAL_Linux;` in `HAL_Linux_Class.h`
>
> All declaration and compilation should be guarded by `CONFIG_HAL_BOARD` macros.
> In this case, we're using `CONFIG_HAL_BOARD == HAL_BOARD_LINUX`.
 When creating a new HAL, declare a new `HAL_BOARD_*` in `AP_HAL/AP_HAL_Boards.h`
>
>  The module should also export an appropriate AP_HAL_MAIN() macro if the appropriate `CONFIG_HAL_BOARD` value is set.
  The `AP_HAL_MAIN` macro expands to a main function (either an `int main (void)`
  or `int main (int argc, const char * argv[]), depending on platform) of an
  ArduPilot application, whose entry points are the c++ functions
  `void setup()` and `void loop()`, ala Arduino.


The following abstractions are implemented:
```bash
tree ardupilot/libraries/AP_HAL_Linux
.
├── AP_HAL_Linux.h
├── AP_HAL_Linux_Main.h
├── AP_HAL_Linux_Namespace.h
├── AP_HAL_Linux_Private.h
├── AnalogIn.cpp
├── AnalogIn.h
├── GPIO.cpp
├── GPIO.h
├── HAL_Linux_Class.cpp
├── HAL_Linux_Class.h
├── I2CDriver.cpp
├── I2CDriver.h
├── RCInput.cpp
├── RCInput.h
├── RCOutput.cpp
├── RCOutput.h
├── SPIDriver.cpp
├── SPIDriver.h
├── Scheduler.cpp
├── Scheduler.h
├── Semaphores.cpp
├── Semaphores.h
├── Storage.cpp
├── Storage.h
├── UARTDriver.cpp
├── UARTDriver.h
├── Util.cpp
└── Util.h

0 directories, 28 files
```
When running ardupilot (either ArduCopter, ArduPlane or APMRover2) in a Linux machine, these classes will be used for tasks such as SPI or I2C handling, Radio Control (RC) input processing and output generation and so on.

As mentioned before, a HAL is usually quite close to the hardware,thus it might be difficult sometimes to code a generic HAL for a big range of systems. In the case of Linux, the development platform has been the BeagleBone Black however an effort has been done to use standart system calls and generic Linux mechanisms. Still, some parts remain platform dependent. This is the case of `RCInput`,  `RCOuput` or the `GPIO` modules that use the PRUs (Programmable RealTime Units) to offload the main processor.

----

*Note*: The HAL has been developed with Debian, for major compatibility, this File System (FS) is recommended.

Other FS could be used (e.g.: Ubuntu, ArchLinux, etc.) but some modifications in the code might be needed.

----

Below, we describe some abstractions implemented in this layer briefly:

#####AP_HAL_Linux_Namespace.h

This [header](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/AP_HAL_Linux_Namespace.h) put all the abstractions under the same namespace.

#####AP_HAL_Linux_Private.h

[AP_HAL_Linux_Private.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/AP_HAL_Linux_Private.h) defines a private umbrella for the `AP_HAL_Linux`.

##### AP_HAL_Linux.h
[AP_HAL_Linux.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/AP_HAL_Linux.h) represents an umbrella header. Boards definition for the different hardware autopilots supported are collected at [AP_HAL_Boards.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/AP_HAL_Boards.h)

##### HAL_Linux_Class.h

[HAL_Linux_Class.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/HAL_Linux_Class.h) defines the class `HAL_linux`.

- **include NOTE**. When using include, the difference between using `<>`and `" "`is the following:
 + `<>` Search in all directories specified to the compiler.
 + `" "`Search , first, in the current directory and after that in the others.



#####HAL_Linux_Class.cpp

[HAL_Linux_Class.cpp](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/HAL_Linux_Class.cpp) is probably one of the most relevant files of the HAL. Defines and instantiates all the abstractions used in the Linux HAL.

You can find some code comments [here](./hal_linux_classcpp.md).

#####AP_HAL_Linux_Main.h

[AP_HAL_Main.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL_Linux/AP_HAL_Linux_Main.h) is used for compatibility with previous HAL code (AVRs, etc.). Launches the system.


The following subsections will describe the most relevant componentes of this HAL.
