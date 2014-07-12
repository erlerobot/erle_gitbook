# AP_HAL::UARTDriver

The `AP_HAL::UARTDriver` class is the `AP_HAL` replacment for ArduPilot's `FastSerial` library.

The `AP_HAL::UARTDriver` class is a pure virtual interface. The code is derived directly from the `FastSerial` class. It provides the methods `begin()`, `end()`, `flush()`, `is_initialized()`, `set_blocking_writes`, and `tx_pending`. The class hierchary for `AP_HAL::UARTDriver` is also derived directly from the `FastSerial` class's hierarchy . `AP_HAL::UARTDriver` is a public `AP_HAL::BetterStream`, which is a public `AP_HAL::Stream`, which is a public `AP_HAL::Print`.

The `utility/` directory contains the definitions of the `AP_HAL::Print`, `AP_HAL::Stream`, and `AP_HAL::BetterStream` classes, as well as default implementations for the `AP_HAL::Print class`.

`AP_HAL::Print` and `AP_HAL::Stream` are derived directly from the classes of the same name in the Arduino core. Some methods dealing with the Arduino core's C++ `String`s, and the `Printable` class, have been left out.

The `AP_HAL::Print` class has default implementations of all of the print methods. Each of these methods is implemented in terms of `virtual size_t write(uint8_t char)`, which is expected to be implemented by a class deriving from `AP_HAL::Print`. This is the same structure as used in the Arduino core.

The `AP_HAL::Stream` class is based on the Arduino core's Stream, but reduced to just the methods we actually use in ArduPilot - `int available()`, `int read()`, and `int peek()`. These methods are all pure virtual

The `AP_HAL::BetterStream` class is a pure virtual version of the class by the same name from Mike Smith's `FastSerial` library. It exposes the methods int `txspace()`, `void print_P()`, `void println_P`, `void printf()`, and `void printf_P()`. As in FastSerial's BetterStream library, function names postfixed with `_P` take a string in program space as their first argument. To use the AVR program space types, the `AP_HAL::BetterStream` class depends on the `<avr/pgmspace.h>` and `AP_Common.h` header files. This is the only part of the `AP_HAL` library which still depends on an AVR specific library. We will find a way to make this dependency modular by isolating the parts of the `AP_Common` and `<avr/pgmspace.h>` headers which BetterStream depends upon into a single header, and then conditionally defining those typedefs and macros to innocuous implementations when compiling for other platforms.
