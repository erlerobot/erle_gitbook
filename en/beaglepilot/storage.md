# Storage



Class that takes care of **Data Storage Media** in linux-based systems
( its aim is  to read and write data storage media like a SDCard).

 ---

`Storage` stores 'eeprom' data on the SD card, with a 4k size, and a in-memory buffer (this keeps the latency down).

**EEPROM** stands for **Electrically Erasable Programmable Read-Only Memory** .This memory is a type of ROM(ead-only memory) that can be programmed, erased and reprogrammed electrically.Definitely EEPROM is a type of non-volatile memory used in computers and other electronic devices to store small amounts of data( in `Storage` class case normally for storing flight/ride data) that must be saved when power is removed.

---

Storage when running ardupilot through the `AP_HAL_Linux` is at `/var/APM`.Notice that **you should manually create this folder**.

This class is divided into two files, **header** (`Storage.h`) and **source code** (`Storage.cpp`).

### Storage.h


Link to the code:[Scheduler.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Storage.h)

`Linux::LinuxStorage`class defines the methods inherited from the [AP_HAL::Storage](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Storage.h) abstract class.

```cpp
#ifndef __AP_HAL_LINUX_STORAGE_H__
#define __AP_HAL_LINUX_STORAGE_H__

#include <AP_HAL.h>
#include "AP_HAL_Linux_Namespace.h"

#define LINUX_STORAGE_SIZE 4096
#define LINUX_STORAGE_MAX_WRITE 512
#define LINUX_STORAGE_LINE_SHIFT 9
#define LINUX_STORAGE_LINE_SIZE (1<<LINUX_STORAGE_LINE_SHIFT)
#define LINUX_STORAGE_NUM_LINES (LINUX_STORAGE_SIZE/LINUX_STORAGE_LINE_SIZE)
...
```

- Imports `AP_HAL.h`and `AP_HAL_Linux_Namespace.h`.


- Define some STORAGE values: size, line size ...

```cpp
...
class Linux::LinuxStorage : public AP_HAL::Storage
{
public:
    LinuxStorage() :
	_fd(-1),
	_dirty_mask(0)
	{}
    void init(void* machtnichts) {}
    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n);

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, const void* src, size_t n);

    void _timer_tick(void);

private:
    int _fd;
    volatile bool _initialised;
    void _storage_create(void);
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[LINUX_STORAGE_SIZE];
    volatile uint32_t _dirty_mask;
};

#endif // __AP_HAL_LINUX_STORAGE_H__
```
- The methods are defined for later implementation in `Storage.cpp`


- As mentioned `Linux::LinuxStorage`class inherits from [AP_HAL::Storage](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Storage.h) class.

### Storage.cpp


[Storage.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/Storage.h) implements the methods defined  in `Storage.h` for
**read and write** data storage media.

```cpp
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

#include "Storage.h"
using namespace Linux;

/*
  This stores 'eeprom' data on the SD card, with a 4k size, and a
  in-memory buffer. This keeps the latency down.
 */

// name the storage file after the sketch so you can use the same board
// card for ArduCopter and ArduPlane
#define STORAGE_DIR "/var/APM"
#define STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"

extern const AP_HAL::HAL& hal;
...
```
- Include `AP_HAL.h`and `Storage.h`.Also defines the board.


- Note that, as indicated in the code:
 + First, defines the storage directory at `/var/APM`.
 + Names the storage file after the sketch. This means that you can use the same board card for ArduCopter and ArduPlane.


Some functions and libraries are included:

+ `assert.h` defines one macro function that can be used as a standard debugging tool.- [(assert.h)](http://www.cplusplus.com/reference/cassert/)



+ Header `sys/types.h` shall include definitions for types- [(sys/types.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/types.h.html)



+ The `sys/stat.h` header defines the structure of the data returned by the functions `fstat()`,` lstat()`, and `stat()`. - [(sys/stat.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html)


+ The `fcntl.h` header shall define some requests and arguments for use by the functions `fcntl()` and `open()`. - [(fcntl.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/fcntl.h.html)



+  This header defines miscellaneous symbolic constants and types, and declares miscellaneous functions.It is provided by POSIX(Portable Operating System Interface-calls to the OS)-compatible systems.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)


+ C Header that defines the following macro:
`errno->Last error number (macro )`; plus at least three additional macro constants: EDOM, ERANGE and EILSEQ .`errno`deals with errors (see [errno](http://www.cplusplus.com/reference/cerrno/errno/) for more details). - [(errno.h)](http://www.cplusplus.com/reference/cerrno/?kw=errno.h)


+  Deal with Input and Output operations: manage files,read and write... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)


```cpp
...

void LinuxStorage::_storage_create(void)
{
	mkdir(STORAGE_DIR, 0777);
	unlink(STORAGE_FILE);
	int fd = open(STORAGE_FILE, O_RDWR|O_CREAT, 0666);
	if (fd == -1) {
		hal.scheduler->panic("Failed to create " STORAGE_FILE);
	}
	for (uint16_t loc=0; loc<sizeof(_buffer); loc += LINUX_STORAGE_MAX_WRITE) {
		if (write(fd, &_buffer[loc], LINUX_STORAGE_MAX_WRITE) != LINUX_STORAGE_MAX_WRITE) {
			hal.scheduler->panic("Error filling " STORAGE_FILE);
		}
	}
	// ensure the directory is updated with the new size
	fsync(fd);
	close(fd);
}
...
```
- This method creates a file if not exists.Then update the new-created or exixsting file to the actual buffer size.


- `mkdir` creates a directory and the `unlink()` function removes the link named by path from its directory.


- `open()` **returns -1 when fails**, so in that case a error message is printed. There are two cases referred with the or operator:
 + `O_CREAT` will fail if the file exists, if not creats it.
 + `O_RDWR` opens a existing file for reading and writing.


- Ensures the directory is updated with the new size (due to the content of the `_buffer[loc]`, that means the same as writing on the file).Then `close()`the file. Is very important to close after reading or writing ( when changes are done), if not there will be problems.

```cpp
...

void LinuxStorage::_storage_open(void)
{
	if (_initialised) {
		return;
	}

	_dirty_mask = 0;
	int fd = open(STORAGE_FILE, O_RDONLY);
	if (fd == -1) {
		_storage_create();
		fd = open(STORAGE_FILE, O_RDONLY);
		if (fd == -1) {
			hal.scheduler->panic("Failed to open " STORAGE_FILE);
		}
	}
	if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
		close(fd);
		_storage_create();
		fd = open(STORAGE_FILE, O_RDONLY);
		if (fd == -1) {
			hal.scheduler->panic("Failed to open " STORAGE_FILE);
		}
		if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
			hal.scheduler->panic("Failed to read " STORAGE_FILE);
		}
	}
	close(fd);
	_initialised = true;
}
...
```
- The `_storage_open`method  is implemented in order to be able to open a file for later writing/reading.


- The first thing is trying to open for reading only (`O_RDONLY`). If `open()` fails the `_storage_create`method is called and opening is retried.


- If read buffer size is not coincident with the actual one calls `_storage_create`for updating it.


- Then `close()`and the `_initialised`value is updated to true.

```cpp
...
/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
 */
void LinuxStorage::_mark_dirty(uint16_t loc, uint16_t length)
{
	uint16_t end = loc + length;
	while (loc < end) {
		uint8_t line = (loc >> LINUX_STORAGE_LINE_SHIFT);
		_dirty_mask |= 1 << line;
		loc += LINUX_STORAGE_LINE_SIZE;
	}
}
...
```
- As specified above, this method marks some lines as dirty.

```cpp
...
uint8_t LinuxStorage::read_byte(uint16_t loc)
{
	if (loc >= sizeof(_buffer)) {
		return 0;
	}
	_storage_open();
	return _buffer[loc];
}
...
```

- This is the first reading method implemented. It read a byte.


- First checks the buffer size and then open the file.


- Then the desired value is returned.

```cpp
...
uint16_t LinuxStorage::read_word(uint16_t loc)
{
	uint16_t value;
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return 0;
	}
	_storage_open();
	memcpy(&value, &_buffer[loc], sizeof(value));
	return value;
}

uint32_t LinuxStorage::read_dword(uint16_t loc)
{
	uint32_t value;
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return 0;
	}
	_storage_open();
	memcpy(&value, &_buffer[loc], sizeof(value));
	return value;
}

void LinuxStorage::read_block(void *dst, uint16_t loc, size_t n)
{
	if (loc >= sizeof(_buffer)-(n-1)) {
		return;
	}
	_storage_open();
	memcpy(dst, &_buffer[loc], n);
}
...
```
 - The `read_word`, `read_dword` , `read_block` for reading one, a dword (a 32-bit unsigned integer ) and a block of words are implemented in the same way.

- Note the use of  `void * memcpy ( void * destination, const void * source, size_t num );`. This function copies the values of num bytes from the location pointed by source directly to the memory block pointed by destination.

```cpp
...
void LinuxStorage::write_byte(uint16_t loc, uint8_t value)
{
	if (loc >= sizeof(_buffer)) {
		return;
	}
	if (_buffer[loc] != value) {
		_storage_open();
		_buffer[loc] = value;
		_mark_dirty(loc, sizeof(value));
	}
}

void LinuxStorage::write_word(uint16_t loc, uint16_t value)
{
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return;
	}
	if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], &value, sizeof(value));
		_mark_dirty(loc, sizeof(value));
	}
}

void LinuxStorage::write_dword(uint16_t loc, uint32_t value)
{
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return;
	}
	if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], &value, sizeof(value));
		_mark_dirty(loc, sizeof(value));
	}
}

void LinuxStorage::write_block(uint16_t loc, const void *src, size_t n)
{
	if (loc >= sizeof(_buffer)-(n-1)) {
		return;
	}
	if (memcmp(src, &_buffer[loc], n) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], src, n);
		_mark_dirty(loc, n);
	}
}
...
```
- Next, the writing methods are implemented following the same scheme.`write_byte`, `write_word`, `write_dword`and`write_block` each method for writing what it name indicates.


- `int memcmp ( const void * ptr1, const void * ptr2, size_t num )` compares the first num bytes of the block of memory pointed by ptr1 to the first num bytes pointed by ptr2, returning zero if they all match or a value different from zero representing which is greater if they do not.


- The write method calls `_storage_open`which calls `_storage_create`. This last method update the STORAGE_FILE

```cpp
...
void LinuxStorage::_timer_tick(void)
{
	if (!_initialised || _dirty_mask == 0) {
		return;
	}

	if (_fd == -1) {
		_fd = open(STORAGE_FILE, O_WRONLY);
		if (_fd == -1) {
			return;
		}
	}

	// write out the first dirty set of lines. We don't write more
	// than one to keep the latency of this call to a minimum
	uint8_t i, n;
	for (i=0; i<LINUX_STORAGE_NUM_LINES; i++) {
		if (_dirty_mask & (1<<i)) {
			break;
		}
	}
	if (i == LINUX_STORAGE_NUM_LINES) {
		// this shouldn't be possible
		return;
	}
	uint32_t write_mask = (1U<<i);
	// see how many lines to write
	for (n=1; (i+n) < LINUX_STORAGE_NUM_LINES &&
		     n < (LINUX_STORAGE_MAX_WRITE>>LINUX_STORAGE_LINE_SHIFT); n++) {
		if (!(_dirty_mask & (1<<(n+i)))) {
			break;
		}
		// mark that line clean
		write_mask |= (1<<(n+i));
	}

	/*
	  write the lines. This also updates _dirty_mask. Note that
	  because this is a SCHED_FIFO thread it will not be preempted
	  by the main task except during blocking calls. This means we
	  don't need a semaphore around the _dirty_mask updates.
	 */
	if (lseek(_fd, i<<LINUX_STORAGE_LINE_SHIFT, SEEK_SET) == (i<<LINUX_STORAGE_LINE_SHIFT)) {
		_dirty_mask &= ~write_mask;
		if (write(_fd, &_buffer[i<<LINUX_STORAGE_LINE_SHIFT], n<<LINUX_STORAGE_LINE_SHIFT) != n<<LINUX_STORAGE_LINE_SHIFT) {
			// write error - likely EINTR
			_dirty_mask |= write_mask;
			close(_fd);
			_fd = -1;
		}
		if (_dirty_mask == 0) {
			if (fsync(_fd) != 0) {
				close(_fd);
				_fd = -1;
			}
		}
	}
}

#endif // CONFIG_HAL_BOARD
```
- This method print out the first dirty set of lines and ceck how many dirty lines to write.


- Then updates the `_dirty_mask`and updates the directory using `fsync`.

