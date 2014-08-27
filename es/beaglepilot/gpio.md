# GPIO

Esta clase se encarga de **GPIO** (General-purpose input/output).

---
**GPIO** es un pon genérico en un circuito integrado cuyo comportamiento, incluyendo si es un pin de entrada o de saldida, puede ser controlado por el usuario en tiempo de ejecución.

Los **pines GPIO** no tienen ningún proposito general definido, y no se utilizan de forma predeterminada. La idea es que a veces el un sistema completo que utiliza un chip podría encontrar útil tener algunas lineas de control digital adicionales, y tenerlas a disposición del chip puede ahorrar la molestia de tener que diseñar un circuito adicional para proporcionarselo

---
Esta clase esta dividad en dos ficheros, **cabecera** (`GPIO.h`) and **código fuente** (`GPIO.cpp`).

###GPIO.h

Enlace al código: [GPIO.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/GPIO.h)

La clase `Linux::LinuxGPIO ` define los métodos heradados de la clase abstracta [AP_HAL::GPIO](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/GPIO.h). La clase Linux::LinuxDigitalSource` también esta definida en este fichero y hereda de `AP_HAL::DigitalSource`.

*For more clarity in the code: The GPIO files define C preproccesor Hexadecimal addresses and BeagleBone Black GPIO mappings in linux.*

```cpp

#ifndef __AP_HAL_LINUX_GPIO_H__
#define __AP_HAL_LINUX_GPIO_H__

#include <AP_HAL_Linux.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"

#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000

#define GPIO_SIZE  0x00000FFF

// OE: 0 is output, 1 is input
#define GPIO_OE    0x14d
#define GPIO_IN    0x14e
#define GPIO_OUT   0x14f

#define LED_AMBER       117
#define LED_BLUE        48
#define LED_SAFETY      61
#define SAFETY_SWITCH   116
#define LOW             0
#define HIGH            1

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
#define LINUX_GPIO_NUM_BANKS 4
#else
// disable GPIO
#define LINUX_GPIO_NUM_BANKS 0
#endif

// BeagleBone Black GPIO mappings
#define BBB_USR0 53
#define BBB_USR1 54
#define BBB_USR2 55
#define BBB_USR3 56
#define BBB_P8_3 38
#define BBB_P8_4 39
#define BBB_P8_5 34
...
#define BBB_P9_42 7
...

```
- A raíz de la declaración:` #define identifier replacement`, cuando el preprocesador se encuentra con esta directiva, se sustituye cualquier ocurrencia de `indetifier` en el resto de código por `replacement`. Esto es lo que se hace para los identificadores en el código de abajo.

- Tambien contiene una declaración con el din de activar o desactivar el GPIO, dependiendo de la tarjeta especificada

```cpp
...

class Linux::LinuxGPIO : public AP_HAL::GPIO {
private:
    struct GPIO {
        volatile uint32_t *base;
        volatile uint32_t *oe;
        volatile uint32_t *in;
        volatile uint32_t *out;
     } gpio_bank[LINUX_GPIO_NUM_BANKS];

public:
    LinuxGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};
...
```

- La clase `LinuxGPIO` hereda de [AP_HAL::GPIO](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/GPIO.h).


- La palabra clave `volatile` es utilizado para declarar que un objeto puede ser modificado en el programa por algo como el sistema operativo, el hardware, o un *thread*. Aquí se definen algunos indicadores volátiles.

- También se define una estructura GPIO llamada  `gpio_bank[]`, como su nombre indica, se utiliza para agrupar los pins en bancos. Puede acceder al banco por el indice.

- En el ámbito público: hay definido un método `init()`, `read()` and `write()`. (estas funciones están implementadas en `GPIO.cpp`).

- Los métodos definidos dentro de `GPIO.h` en la clase `Linux::LinuxGPIO` como `LinuxGPIO::pinMode()`,`LinuxGPIO::read()`,`LinuxGPIO::write()` y así sucesivamente, son implementados para manejar los bancos de pines GPIO de la placa.

```cpp
...
class Linux::LinuxDigitalSource : public AP_HAL::DigitalSource {
public:
    LinuxDigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value);
    void    toggle();
private:
    uint8_t _v;

};

#endif // __AP_HAL_LINUX_GPIO_H__
```
- Define el `LinuxDigitalSource` que hereda de `DigitalSource`, que es parte de [AP_HAL::GPIO](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/GPIO.h)


- Los métodos definidos dentro de `gpio.h` en la clase `Linux :: LinuxDigitalSource`, se han aplicado aquí para administrar fuentes digitales. 

- Algunos métodos se definen aquí, para posteriormente implementarlos en `GPIO.cpp`.


###GPIO.cpp

[GPIO.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/GPIO.cpp) implementa los métodos definidos en `GPIO.h`.

El método más importante es `init()` que habilita todos los bancos GPIO, abra el directorio `/sys/class/gpio/export`, y mapeo los bancos GPIO en `/dev/mem`.

```cpp
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "GPIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
...
```
- En este fragmento de código `AP_HAL.h` y `GPIO.h` están incluidos y la placa esta definida.

Algunas funciones y librerias están incluidas:

+  Trata con las operaciones de entrada y salida: gestión de archivos, lectura, escritura... - [ (stdio.h)](http://www.cplusplus.com/reference/cstdio/)

+ Esta cabecera define varias funciones de propósito general, incluyendo la gestión dinámica de memoria, generación de numeros aleatorios, la comunicación con el entorno, aritmética de enteros, búsqueda, clasificación...- [(stdlib.h)](http://www.cplusplus.com/reference/cstdlib/?kw=stdlib.h)

+  El archico de cabeceras `string.h` define varias funciones para manipular string de C y arrays. - [(string.h)](http://www.cplusplus.com/reference/cstring/)


+ Cabecera de C que define la siguiente macro:

`errno->Last error number (macro )`; al menos 3 macros constantes adicionales: EDOM, ERANGE and EILSEQ. (see [errno](http://www.cplusplus.com/reference/cerrno/errno/) para más detalles). - [(errno.h)](http://www.cplusplus.com/reference/cerrno/?kw=errno.h)

+  Esta cabecera denife constantes y tipos y declara funciones auxiliares. Es proporcionado por POSIX (Portable Operating System Interface-calls to the OS) - Sistemas compatibles.- [(unistd.h)](http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/unistd.h.html)

+ La cabecera `fcntl.h` define varias solicitudes y argumentos para el uso de las funciones `fcntl()` and `open()`. - [(fcntl.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/fcntl.h.html)

+ La cabecera `poll.h` define la estructura `pollfd` que incluye al menos los siguientes miembros:
int *fd*(the following descriptor being polled),
short int *events* (the input event flags) and
short int *revents * ( the output event flags) - [(poll.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/poll.h.html)

+ La cabecera `sys/mman.h` define las opciones de protección cuando hay consultas, archivos asignados en memoria, o objetos en memoria compartida compatibles con opciones son soportados- [(sys/mman.h)](http://pubs.opengroup.org/onlinepubs/009695399/basedefs/sys/mman.h.html)

+ La cabecera `sys/stat.h` define la esructura de los datos devueltos por las funciones `fstat()`,` lstat()`, and `stat()`. - [(sys/stat.h)](http://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html)

```cpp
...
using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
LinuxGPIO::LinuxGPIO()
{}
...
```
- Begins a namespace definition, takes the `AP_HAL::HAL& hal` value and call `LinuxGPIO()` funtion that manage pin banks.

```cpp
...
void LinuxGPIO::init()
{
#if LINUX_GPIO_NUM_BANKS == 4
    int mem_fd;
    // Enable all GPIO banks
    // Without this, access to deactivated banks (i.e. those with no clock source set up) will (logically) fail with SIGBUS
    // Idea taken from https://groups.google.com/forum/#!msg/beagleboard/OYFp4EXawiI/Mq6s3sg14HoJ

    uint8_t bank_enable[3] = { 5, 65, 105 };
    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
    if (export_fd == -1) {
        hal.scheduler->panic("unable to open /sys/class/gpio/export");
    }
    for (uint8_t i=0; i<3; i++) {
        dprintf(export_fd, "%u\n", (unsigned)bank_enable[i]);
    }
    close(export_fd);


    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
            printf("can't open /dev/mem \n");
            exit (-1);
    }

    /* mmap GPIO */
    off_t offsets[LINUX_GPIO_NUM_BANKS] = { GPIO0_BASE, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE };
    for (uint8_t i=0; i<LINUX_GPIO_NUM_BANKS; i++) {
        gpio_bank[i].base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, offsets[i]);
        if ((char *)gpio_bank[i].base == MAP_FAILED) {
            hal.scheduler->panic("unable to map GPIO bank");
        }
        gpio_bank[i].oe = gpio_bank[i].base + GPIO_OE;
        gpio_bank[i].in = gpio_bank[i].base + GPIO_IN;
        gpio_bank[i].out = gpio_bank[i].base + GPIO_OUT;
    }

    close(mem_fd);
#endif // LINUX_GPIO_NUM_BANKS
}
...
```
Habilitar todos bancos GPIO (creación de los bancos):

+ En primer lugar, intenta abrir el archivo `/sys/class/gpio/export`: si falla imprimir un mensahe, si no imprime el contenido.

+ Abre `/dev/mem`.


+ `off_t`  es un tipo usado para *offset* a varios funciones de archivos relacionados. Con esto activamos los bancos de GPIO, asignandolos en `/dev/mem`.

```cpp

...

void LinuxGPIO::pinMode(uint8_t pin, uint8_t output)
{
    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (output == HAL_GPIO_INPUT) {
        *gpio_bank[bank].oe |= (1U<<bankpin);
    } else {
        *gpio_bank[bank].oe &= ~(1U<<bankpin);
    }
}

int8_t LinuxGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}
...
```
- Check the bank numbers and the output. Note the use of 1U for the left shift, to make it unsigned and the allocation using binary asigments.


- The return -1 is like a return boolean=False.

```cpp
...
uint8_t LinuxGPIO::read(uint8_t pin) {

    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return 0;
    }
    return *gpio_bank[bank].in & (1U<<bankpin) ? HIGH : LOW;

}


void LinuxGPIO::write(uint8_t pin, uint8_t value)
{
    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (value == LOW) {
        *gpio_bank[bank].out &= ~(1U<<bankpin);
    } else {
        *gpio_bank[bank].out |= 1U<<bankpin;
    }
}
...
```
- Similarly to `pinmode()` acts `read()` and `write()`.

```
...
void LinuxGPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* LinuxGPIO::channel(uint16_t n) {
    return new LinuxDigitalSource(n);
}

/* Interrupt interface: */
bool LinuxGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}

bool LinuxGPIO::usb_connected(void)
{
    return false;
}

LinuxDigitalSource::LinuxDigitalSource(uint8_t v) :
    _v(v)
{

}

void LinuxDigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_v, output);
}

uint8_t LinuxDigitalSource::read()
{
    return hal.gpio->read(_v);
}

void LinuxDigitalSource::write(uint8_t value)
{
    return hal.gpio->write(_v,value);
}

void LinuxDigitalSource::toggle()
{
    write(!read());
}

#endif // CONFIG_HAL_BOARD
```
- Aquí las funciones que faltan se implementan.
