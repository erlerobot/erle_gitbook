# UARTDriver


Class that take care of **UART** (Universal Asynchronous Receiver-Transmitter) in **linux-based** systems.

---
The [**Universal Asynchronous Receiver/Transmitter**](http://en.wikipedia.org/wiki/Universal_asynchronous_receiver/transmitter) (UART) takes bytes of data and transmits the individual bits in a sequential fashion. At the destination, a second UART re-assembles the bits into complete bytes. Each UART contains a shift register, which is the fundamental method of conversion between serial and parallel forms. Serial transmission of digital information (bits) through a single wire or other medium is less costly than parallel transmission through multiple wires.

**Asynchronous transmission** allows data to be transmitted without the sender having to send a clock signal to the receiver. Instead, the sender and receiver must agree on timing parameters in advance and special bits are added to each word which are used to synchronize the sending and receiving units.

A **UART** usually contains the following **components**:

- a clock generator, usually a multiple of the bit rate to allow sampling in the middle of a bit period.
- input and output shift registers
- transmit/receive control
- read/write control logic
- transmit/receive buffers (optional)
- parallel data bus buffer (optional)
- First-in, first-out (FIFO) buffer memory (optional)

---



![UART](../en/img/beaglepilot//UART.png)

This class is divided into two files, **header** (`UARTDriver.h`) and **source code** (`UARTDriver.cpp`).

### UARTDriver.h

Link to the code:[UARTDriver.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/UARTDriver.h)

`Linux::LinuxUARTDriver`class defines the methods inherited from the [AP_HAL::UARTDriver](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.h) abstract class, for handling **asynchronous serial** ports and peripherals.

Define methods of **AP_HAL::UARTDriver**, **AP_HAL::Stream** and **AP_HAL::Print** abstract classes. **Stream** and **Print** classes are defined in [/libraries/AP_HAL/utility](https://github.com/diydrones/ardupilot/tree/master/libraries/AP_HAL/utility) directory.


```cpp


#ifndef __AP_HAL_LINUX_UARTDRIVER_H__
#define __AP_HAL_LINUX_UARTDRIVER_H__

#include <AP_HAL_Linux.h>

class Linux::LinuxUARTDriver : public AP_HAL::UARTDriver {
public:
    LinuxUARTDriver(bool default_console);
    /* Linux implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* Linux implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* Linux implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    void set_device_path(char *path);

    void _timer_tick(void);

private:
    char *device_path;
    int _rd_fd;
    int _wr_fd;
    bool _nonblocking_writes;
    bool _console;
    volatile bool _initialised;
    volatile bool _in_timer;
    uint16_t _base_port;
    char *_ip;
    char *_flag;
    bool _connected; // true if a client has connected

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
    uint8_t *_readbuf;
    uint16_t _readbuf_size;

    // _head is where the next available data is. _tail is where new
    // data is put
    volatile uint16_t _readbuf_head;
    volatile uint16_t _readbuf_tail;

    uint8_t *_writebuf;
    uint16_t _writebuf_size;
    volatile uint16_t _writebuf_head;
    volatile uint16_t _writebuf_tail;

    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);
    void _tcp_start_connection(bool wait_for_connection);
    int _parseDevicePath(char* arg);
    uint64_t _last_write_time;
};

#endif // __AP_HAL_LINUX_UARTDRIVER_H__
```
- The `Linux::UARTDriver`inherit from [AP_HAL::UARTDriver](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.h)


- This class provides extended port open method
that allows for both opening with specified buffer sizes, and re-opening to adjust a subset of the port's settings.


- Here are defined variables and functions for later implementation in `UARTDRiver.cpp`.


- Note the use of methods defined in `Stream.h` and `Print.h`.

---

###Print.h

Link to the code: [Print.h](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/utility/Print.h).

`Print.h `header belongs to `AP_HAL/ utility folder.
As you can see, this class implements methods for printing different types.

###Stream.h


Link to the code:[Stream.h](https://github.com/BeaglePilot/ardupilot/tree/master/libraries/AP_HAL/utility)

`Stream.h`belongs to `AP_HAL/utility` folder.This file defines some methods.


### UARTDriver.cpp


[UARTDriver.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/UARTDriver.cpp) implements the methods defined  in `UARTDriver.h`.

```cpp

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "UARTDriver.h"

#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string.h>
#include <arpa/inet.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

#define DEVICE_TCP 0
#define DEVICE_SERIAL 1
#define DEVICE_UNKNOWN 99
...
```
- Imports `AP_HAL.h` and `UARTDriver.h`. Defines the board and three device types: DEVICE_TCP,
 DEVICE_SERIAL, DEVICE_UNKNOWN.


Some functions and libraries are included:





+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)



+ C Header that defines the following macro:
`errno->Last error number (macro )`; plus at least three additional macro constants: EDOM, ERANGE and EILSEQ .`errno`deals with errors (see [errno](http://www.cplusplus.com/reference/cerrno/errno/) for more details). - [(errno.h)](http://www.cplusplus.com/reference/cerrno/?kw=errno.h)



+ The `<termios.h>` header contains the definitions used by the terminal I/O interfaces (see the XBD specification, General Terminal Interface  for the structures and names defined). - [(termios.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html)


+ This header defines several general purpose functions, including dynamic memory management, random number generation, communication with the environment, integer arithmetics, searching, sorting and converting.- [(stdlib.h)](http://www.cplusplus.com/reference/cstdlib/?kw=stdlib.h)


+ Header `sys/types.h` shall include definitions for types- [(sys/types.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/types.h.html)


+ The `sys/stat.h` header defines the structure of the data returned by the functions `fstat()`,` lstat()`, and `stat()`. - [(sys/stat.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html)

+ The `fcntl.h` header shall define some requests and arguments for use by the functions `fcntl()` and `open()`. - [(fcntl.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/fcntl.h.html)


+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+ `assert.h` defines one macro function that can be used as a standard debugging tool.- [(assert.h)](http://www.cplusplus.com/reference/cassert/)


+ `ioctl() `function manipulates the underlying device parameters of special files.You can read more in `man ioctl()` - [(sys/ioctl.h)](http://unix.superglobalmegacorp.com/Net2/newsrc/sys/ioctl.h.html)


+ `sys/socket.h` should define types and structures for communication between different entities on a network.- [(sys/socket.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/socket.h.html)

**NOTE**: Sockets are an abstract concept that designates the end point of a connection. The programs use sockets to communicate with other programs, which may be located on different computers. A socket is defined by the IP address of the machine, the port on which it listens, and the protocol used.

+ This header defines types and structs for dealing with IP address and port addresses - [(netinet.in.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/netinet/in.h.html)

+ `netinet/tcp.h`  header shall define the TCP_NODELAY macro for use as a socket option at the IPPROTO_TCP level and avoid coalescing of small segments. -[(netinet/tcp.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/netinet/tcp.h.html)


+  The `string.h` header file defines several functions to manipulate C strings and arrays. - [(string.h)](http://www.cplusplus.com/reference/cstring/)


+ This header makes available some of the types defined in `netinet.in.h`  -[(arpa/inet.h)](http://pubs.opengroup.org/onlinepubs/7908799/xns/arpainet.h.html)

```cpp
...
LinuxUARTDriver::LinuxUARTDriver(bool default_console) :
    device_path(NULL),
    _rd_fd(-1),
    _wr_fd(-1)
{
    if (default_console) {
        _rd_fd = 0;
        _wr_fd = 1;
        _console = true;
    }
}
...
```

- This method initialized some variables.


```cpp
...
/*
  set the tty device to use for this UART
 */
void LinuxUARTDriver::set_device_path(char *path)
{
    device_path = path;
}
...
```
- Set the `device_path` value.


```cpp
...

/*
  open the tty
 */
void LinuxUARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}
...
```
- This method call the following `begin()`method passing (b,0,0)- (b,rxs,txs) as parameters.

```cpp
...
void LinuxUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (device_path == NULL && _console) {
        _rd_fd = 0;
        _wr_fd = 1;
        rxS = 512;
        txS = 512;
        fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);
        fcntl(_wr_fd, F_SETFL, fcntl(_wr_fd, F_GETFL, 0) | O_NONBLOCK);
    } else if (!_initialised) {
        if (device_path == NULL) {
            return;
        }

        switch (_parseDevicePath(device_path)){
        case DEVICE_TCP:
            {
                _connected = false;
                if (_flag != NULL){
                    if (!strcmp(_flag, "wait")){
                        _tcp_start_connection(true);
                    } else {
                        _tcp_start_connection(false);
                    }
                } else {
                    _tcp_start_connection(false);
                }

                if (_connected) {
                    if (rxS < 1024) {
                        rxS = 1024;
                    }
                    if (txS < 1024) {
                        txS = 1024;
                    }
                } else {
                    printf("LinuxUARTDriver TCP connection not stablished\n");
                    exit(1);
                }
                break;
            }
            case DEVICE_SERIAL:
            {
                uint8_t retries = 0;
                while (retries < 5) {
                    _rd_fd = open(device_path, O_RDWR);
                    if (_rd_fd != -1) {
                        break;
                    }
                    // sleep a bit and retry. There seems to be a NuttX bug
                    // that can cause ttyACM0 to not be available immediately,
                    // but a small delay can fix it
                    hal.scheduler->delay(100);
                    retries++;
                }
                _wr_fd = _rd_fd;
                if (_rd_fd == -1) {
                    fprintf(stdout, "Failed to open UART device %s - %s\n",
                            device_path, strerror(errno));
                    return;
                }
                if (retries != 0) {
                    fprintf(stdout, "WARNING: took %u retries to open UART %s\n",
                            (unsigned)retries, device_path);
                    return;
                }

                // always run the file descriptor non-blocking, and deal with
                // blocking IO in the higher level calls
                fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);

                if (rxS < 1024) {
                    rxS = 1024;
                }

                // we have enough memory to have a larger transmit buffer for
                // all ports. This means we don't get delays while waiting to
                // write GPS config packets
                if (txS < 1024) {
                    txS = 1024;
                }
                break;
            }
            default:
            {
                // Notify that the option is not valid and select standart input and output
                printf("LinuxUARTDriver parsing failed, using default\n");

                _rd_fd = 0;
                _wr_fd = 1;
                rxS = 512;
                txS = 512;
                fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);
                fcntl(_wr_fd, F_SETFL, fcntl(_wr_fd, F_GETFL, 0) | O_NONBLOCK);
                break;
            }
        }
    }

    _initialised = false;
    while (_in_timer) hal.scheduler->delay(1);

    if (b != 0 && _rd_fd == _wr_fd) {
        // set the baud rate
        struct termios t;
        memset(&t, 0, sizeof(t));
        tcgetattr(_rd_fd, &t);
        cfsetspeed(&t, b);
        // disable LF -> CR/LF
        t.c_oflag &= ~ONLCR;
        tcsetattr(_rd_fd, TCSANOW, &t);
    }

    /*
      allocate the read buffer
    */
    if (rxS != 0 && rxS != _readbuf_size) {
        _readbuf_size = rxS;
        if (_readbuf != NULL) {
            free(_readbuf);
        }
        _readbuf = (uint8_t *)malloc(_readbuf_size);
        _readbuf_head = 0;
        _readbuf_tail = 0;
    }

    /*
      allocate the write buffer
    */
    if (txS != 0 && txS != _writebuf_size) {
        _writebuf_size = txS;
        if (_writebuf != NULL) {
            free(_writebuf);
        }
        _writebuf = (uint8_t *)malloc(_writebuf_size+16);
        _writebuf_head = 0;
        _writebuf_tail = 0;
    }

    if (_writebuf_size != 0 && _readbuf_size != 0) {
        _initialised = true;
    }
}

...

```

- In this slice of code all the methods and variables are initialized to their corresponding.


- About the parameters passed to `begin()`:
 + Buffer sizes greater than ::_max_buffer_size will be rounded
 + param.	baud (b) :Selects the speed that the port will be configured to.  If zero, the port speed is left unchanged.
 +param. rxSpace (rxs):	Sets the receive buffer size for the port.  If zero then the buffer size is left unchanged if the portvis open, or set to `::_default_rx_buffer_size` if it is currently closed.
 + param. txSpace(txs):	Sets the transmit buffer size for the port.  If zero hen the buffer size is left unchanged if the port is open, or set to `::_default_tx_buffer_size` if it is currently closed.

```cpp
...
/* Device path accepts the following syntaxes:
        - /dev/ttyO1
        - tcp:192.168.2.15:1243:wait
*/

int LinuxUARTDriver::_parseDevicePath(char* arg)
{
    const char *serial_string = "/dev/tty";
    const char *tcp_string = "tcp";
    _flag = NULL; // init flag

    if(strstr(arg, tcp_string) != NULL){
        // Parse the TCP string
        char *protocol, *ip, *port, *flag;
        protocol = strtok ( arg, ":" );
        ip = strtok ( NULL, ":" );
        port = strtok ( NULL, ":" );
        flag = strtok ( NULL, ":" );

        _base_port = (uint16_t) atoi(port);
        _ip = ip;
        _flag = flag;
        return DEVICE_TCP;
    } else if (strstr(arg, serial_string) != NULL){
        return DEVICE_SERIAL;
    } else {
        return DEVICE_UNKNOWN;
    }
}


 ...
 ```
- `(const) char * strstr ( const char * str1, const char * str2 );` returns a pointer to the first occurrence of str2 in str1, or a null pointer if str2 is not part of str1.


-  `char * strtok ( char * str, const char * delimiters );`split string into tokens.(A **token** is a character string that has a consistent meaning in a programming language.)A sequence of calls to this function split str into tokens, which are sequences of contiguous characters separated by any of the characters that are part of delimiters.

```cpp
...
/*
  start a TCP connection for the serial port. If wait_for_connection
  is true then block until a client connects
 */
void LinuxUARTDriver::_tcp_start_connection(bool wait_for_connection)
{
    int one=1;
    struct sockaddr_in sockaddr;
    int ret;
    int listen_fd = -1;  // socket we are listening on
    int net_fd = -1; // network file descriptor, will be linked to wr_fd and rd_fd
    uint8_t portNumber = 0; // connecto to _base_port + portNumber

    // if (_console) {
    //         // hack for console access
    //         connected = true;
    //         listen_fd = -1;
    //         fd = 1;
    //         return;
    // }

    if (net_fd != -1) {
        close(net_fd);
    }

    if (listen_fd == -1) {
        memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
        sockaddr.sin_len = sizeof(sockaddr);
#endif
        sockaddr.sin_port = htons(_base_port + portNumber);
        sockaddr.sin_family = AF_INET;
        // sockaddr.sin_addr.s_addr = inet_addr(_base_ip);

        // Bind to all interfaces
        sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

        listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd == -1) {
            printf("socket failed - %s\n", strerror(errno));
            exit(1);
        }

        /* we want to be able to re-use ports quickly */
        setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        printf("bind port %u for %u\n",
                (unsigned)ntohs(sockaddr.sin_port),
                (unsigned)portNumber),

        ret = bind(listen_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
        if (ret == -1) {
            printf("bind failed on port %u - %s\n",
                        (unsigned)ntohs(sockaddr.sin_port),
                        strerror(errno));
            exit(1);
        }

        ret = listen(listen_fd, 5);
        if (ret == -1) {
            printf("listen failed - %s\n", strerror(errno));
            exit(1);
        }

        printf("Serial port %u on TCP port %u\n", portNumber,
                _base_port + portNumber);
        fflush(stdout);
    }

    if (wait_for_connection) {
        printf("Waiting for connection ....\n");
        fflush(stdout);
        net_fd = accept(listen_fd, NULL, NULL);
        if (net_fd == -1) {
            printf("accept() error - %s", strerror(errno));
            exit(1);
        }
        setsockopt(net_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        setsockopt(net_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        // always run the file descriptor non-blocking, and deal with                                         |
        // blocking IO in the higher level calls
        fcntl(net_fd, F_SETFL, fcntl(net_fd, F_GETFL, 0) | O_NONBLOCK);

        _connected = true;
        _rd_fd = net_fd;
        _wr_fd = net_fd;
    }
}


 ...
 ```
 - Here networking processes are implemented using sockets.


 - Follow the notes provided with the code for a further understanding.


 ```cpp

...
/*
  shutdown a UART
 */

void LinuxUARTDriver::end()
{
    _initialised = false;
    _connected = false;
    while (_in_timer) hal.scheduler->delay(1);
    if (_rd_fd == _wr_fd && _rd_fd != -1) {
        close(_rd_fd);
    }
    _rd_fd = -1;
    _wr_fd = -1;
    if (_readbuf) {
        free(_readbuf);
        _readbuf = NULL;
    }
    if (_writebuf) {
        free(_writebuf);
        _writebuf = NULL;
    }
    _readbuf_size = _writebuf_size = 0;
    _writebuf_head = 0;
    _writebuf_tail = 0;
    _readbuf_head = 0;
    _readbuf_tail = 0;
}

...
```
- This codes ends the processes of UART.

```cpp
...

void LinuxUARTDriver::flush()
{
    // we are not doing any buffering, so flush is a no-op
}


/*
  return true if the UART is initialised
 */
bool LinuxUARTDriver::is_initialized()
{
    return _initialised;
}

/*
  enable or disable blocking writes
 */

 void LinuxUARTDriver::set_blocking_writes(bool blocking)
{
    _nonblocking_writes = !blocking;
}
...
```
- Here `flush()`, `set_blocking_writes`and `is_initialized()` methods are implemented.

```cpp
...


/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size

...
```
- Defines buffering handling macros.

```cpp
...
/*
  do we have any bytes pending transmission?
 */
bool LinuxUARTDriver::tx_pending()
{
    return !BUF_EMPTY(_writebuf);
}

/*
  return the number of bytes available to be read
 */
int16_t LinuxUARTDriver::available()
{
    if (!_initialised) {
        return 0;
    }
    uint16_t _tail;
    return BUF_AVAILABLE(_readbuf);
}

/*
  how many bytes are available in the output buffer?
 */
int16_t LinuxUARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    uint16_t _head;
    return BUF_SPACE(_writebuf);
}
...
```
-  `txSpace` Sets the transmit buffer size for the port.  If zero	then the buffer size is left unchanged if the port	is open, or set to `::_default_tx_buffer_size` if it is currently closed.

```cpp
...

int16_t LinuxUARTDriver::read()
{
    uint8_t c;
    if (!_initialised || _readbuf == NULL) {
        return -1;
    }
    if (BUF_EMPTY(_readbuf)) {
        return -1;
    }
    c = _readbuf[_readbuf_head];
    BUF_ADVANCEHEAD(_readbuf, 1);
    return c;
}

/* Linux implementations of Print virtual methods */
size_t LinuxUARTDriver::write(uint8_t c)
{
    if (!_initialised) {
        return 0;
    }
    uint16_t _head;

    while (BUF_SPACE(_writebuf) == 0) {
        if (_nonblocking_writes) {
            return 0;
        }
        hal.scheduler->delay(1);
    }
    _writebuf[_writebuf_tail] = c;
    BUF_ADVANCETAIL(_writebuf, 1);
    return 1;
}

/*
  write size bytes to the write buffer
 */
size_t LinuxUARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return 0;
    }
    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    uint16_t _head, space;
    space = BUF_SPACE(_writebuf);
    if (space == 0) {
        return 0;
    }
    if (size > space) {
        size = space;
    }
    if (_writebuf_tail < _head) {
        // perform as single memcpy
        assert(_writebuf_tail+size <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, size);
        BUF_ADVANCETAIL(_writebuf, size);
        return size;
    }

    // perform as two memcpy calls
    uint16_t n = _writebuf_size - _writebuf_tail;
    if (n > size) n = size;
    assert(_writebuf_tail+n <= _writebuf_size);
    memcpy(&_writebuf[_writebuf_tail], buffer, n);
    BUF_ADVANCETAIL(_writebuf, n);
    buffer += n;
    n = size - n;
    if (n > 0) {
        assert(_writebuf_tail+n <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], buffer, n);
        BUF_ADVANCETAIL(_writebuf, n);
    }
    return size;
}

/*
  try writing n bytes, handling an unresponsive port
 */
int LinuxUARTDriver::_write_fd(const uint8_t *buf, uint16_t n)
{
    int ret = 0;

    struct pollfd fds;
    fds.fd = _wr_fd;
    fds.events = POLLOUT;
    fds.revents = 0;

    if (poll(&fds, 1, 0) == 1) {
        ret = ::write(_wr_fd, buf, n);
    }

    if (ret > 0) {
        BUF_ADVANCEHEAD(_writebuf, ret);
        return ret;
    }

    return ret;
}

/*
  try reading n bytes, handling an unresponsive port
 */
int LinuxUARTDriver::_read_fd(uint8_t *buf, uint16_t n)
{
    int ret;
    ret = ::read(_rd_fd, buf, n);
    if (ret > 0) {
        BUF_ADVANCETAIL(_readbuf, ret);
    }
    return ret;
}

...
```
- This are reading and writing methods.

```cpp
...
/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void LinuxUARTDriver::_timer_tick(void)
{
    uint16_t n;

    if (!_initialised) return;

    _in_timer = true;

    // write any pending bytes
    uint16_t _tail;
    n = BUF_AVAILABLE(_writebuf);
    if (n > 0) {
        if (_tail > _writebuf_head) {
            // do as a single write
            _write_fd(&_writebuf[_writebuf_head], n);
        } else {
            // split into two writes
            uint16_t n1 = _writebuf_size - _writebuf_head;
            int ret = _write_fd(&_writebuf[_writebuf_head], n1);
            if (ret == n1 && n != n1) {
                _write_fd(&_writebuf[_writebuf_head], n - n1);
            }
        }
    }

    // try to fill the read buffer
    uint16_t _head;
    n = BUF_SPACE(_readbuf);
    if (n > 0) {
        if (_readbuf_tail < _head) {
            // one read will do
            assert(_readbuf_tail+n <= _readbuf_size);
            _read_fd(&_readbuf[_readbuf_tail], n);
        } else {
            uint16_t n1 = _readbuf_size - _readbuf_tail;
            assert(_readbuf_tail+n1 <= _readbuf_size);
            int ret = _read_fd(&_readbuf[_readbuf_tail], n1);
            if (ret == n1 && n != n1) {
                assert(_readbuf_tail+(n-n1) <= _readbuf_size);
                _read_fd(&_readbuf[_readbuf_tail], n - n1);
            }
        }
    }

    _in_timer = false;
}

#endif // CONFIG_HAL_BOARD
```
- Checks if there are pending bytes and sends them.
