# AnalogIn

----

**En el momento de escritura (29-07-2014) esta clase no se ha implementado todavía**

----

Esta clase se encarga de **ADC**  (Analogic to Digital Conversion). Define e implementa médotos para la **medición de la tensión** de las **señales analógicas** en **sistemas basados en linux**.

Esta clase esta dividida en dos archivos, **cabecera** (`AnalogIn.h`) y **código fuente**  (`AnalogIn.cpp`).

###AnalogIn.h

Enlace al código: [AnalogIn.h](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/AnalogIn.h).

`La clase Linux::AnalogIn` define los métodos heredados de la clase abstracta[AP_HAL::AnalogIn](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/AnalogIn.h). La clase `Linux::LinuxAnalogSource` esta definida en esta cabecera.

```cpp

#ifndef __AP_HAL_LINUX_ANALOGIN_H__
#define __AP_HAL_LINUX_ANALOGIN_H__

#include <AP_HAL_Linux.h>

class Linux::LinuxAnalogSource : public AP_HAL::AnalogSource {
public:
```
Define la clase `LinuxAnalogSource` (clase de [AP_HAL::AnalogSource](https://github.com/BeaglePilot/ardupilot/blob/master/libraries/AP_HAL/AnalogIn.h))

```cpp
    LinuxAnalogSource(float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
```
`void set_stop_pin(uint8_t p);`: opcionalmente permite el establecimiento de un pin que detiene el dispositivo de lectura. Esto es necesario para los dispotivos sonar, cuando tienes más de un sonar, y desea evitar que interfieran entre sí. Se supone que si se mantiene bajo el dispositivo se detiene, si esta alto el dispotivo comiento a leer.

---

*Nota*: uint8_t, uint16_t, uint32_t, uint64_t son de tipo entero con un anchura de exactamente 8, 16, 32, or 64 bits. Son parte de `<cstdint> (stdint.h)`. Estos tipos son imporatados cuandos se importa `AP_HAL.h`

---

```cpp
    void set_settle_time(uint16_t settle_time_ms);
```
`virtual void set_settle_time(uint16_t settle_time_ms);`: opcionalmente permite un periodo para asentarse en milisegundos. Esto sólo se utiliza si el pin de parada se utiliza. Si el periodo no es cero entonces la entrada analógica esperará para obtener un lectura para ese número de milisegundos. Tenga en cuenta que esto ralentizará la lectura de las entradas analógicas.

```cpp
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }
private:
    float _v;
};
...
```

El `float` devuleto es un valor de `voltaje` entre 0.0-5.0 V.

```
...
class Linux::LinuxAnalogIn : public AP_HAL::AnalogIn {
public:
    LinuxAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);

    // we don't yet know how to get the board voltage
    float board_voltage(void) { return 0.0f; }

};
#endif // __AP_HAL_LINUX_ANALOGIN_H__
```
Este pedazo de código:

- Define la clase `LinuxAnalogIn` de `AP_HAL::AnalogIN`.


- Declara el método init() method con un puntero como argumento.


- El puntero es inicializado a un canal.


- `board_voltage` debe leer el voltaje de la placa. Aún no está implementado.

###AnalogIn.cpp


[AnalogIn.cpp](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL_Linux/AnalogIn.cpp) implements the methods defined  in `AnalogIn.h` for read/write analog signals.  **Not implemented yet**.

```cpp
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn.h"

using namespace Linux;
...
```
Dentro del **espacio de nombres (namespace)** se incluyen todas las funciones apropiadas que cumplean con un determinado objetivo. A continuación puede hacer referencia a las funciones que forman parte de un espacio de nombres prefijando el nombre de la función con el nombre de espacio seguido del operador `::`.

```cpp

...
LinuxAnalogSource::LinuxAnalogSource(float v) :
    _v(v)
{}

float LinuxAnalogSource::read_average() {
    return _v;
}

float LinuxAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float LinuxAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float LinuxAnalogSource::read_latest() {
    return _v;
}

void LinuxAnalogSource::set_pin(uint8_t p)
{}

void LinuxAnalogSource::set_stop_pin(uint8_t p)
{}

void LinuxAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

LinuxAnalogIn::LinuxAnalogIn()
{}

void LinuxAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* LinuxAnalogIn::channel(int16_t n) {
    return new LinuxAnalogSource(1.11);
}

#endif // CONFIG_HAL_BOARD
```
