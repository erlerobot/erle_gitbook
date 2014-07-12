# AP_HAL::ConsoleDriver

The `AP_HAL::ConsoleDriver` class is pure virtual and can be found in `/libraries/AP_HAL/Console.h` . It is derived from the `AP_HAL::BetterStream` class.

In the existing ArduPilot code, there is no unified way to send debugging messages, warnings, and errors to the user. A dedicated Console driver, is envisioned as a better way to communicate that sort of information to the user.

In addition to the `AP_HAL::BetterStream` interface, `AP_HAL::ConsoleDriver` exposes the following methods for implementing a user defined backend. (I envision this backend will be piped over a mavlink connection by application, but right now I'm leaving it open ended.)

- **`void init(void*implspecific)`** : Standard init code. Should be called after the UARTDrivers, but before all other drivers, in AP_HAL::HAL::init.
- **`void backend_open()`** : Start buffering reads and writes to user backend
- **`void backend_close()`** : Stop buffering reads and writes to user backend
- **`int backend_read(uint8_t *data, int len)`** : Read from user backend buffer. Data sent by `write` through the `BetterStream` interface will be available to backend_read, modulo oveflowing the internal buffers (undefined behavior).
- **`int backend_write(const uint8_t *data, int len)`** : Write to user backend buffer. Written data will be available by `read` through the `BetterStream` interface, modulo overflowing the internal buffers (undefined behavior).

A few implementation guidelines:

- This is a low assurance data interface: it is more important to maintain the timing properties of the system than deliver data properly. Assume the user backend to the BetterStream interface will be operated synchronously, and take care not to create deadlocks.
- Behavior while the user backend is not open is undefined. It may be appropriate to either implement a trivial interface (drop all writes, return -1 on all reads) or proxy to a UARTDriver, depending on the platform & developer's needs.
- Behavior while the user backend is open: activity on the `BetterStream` interface should never block. Internal buffer sizes are undefined. Data that doesn't fit into the internal buffers should be dropped.
