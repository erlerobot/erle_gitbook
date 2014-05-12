(I²C)
=====


I²C es un bus de comunicaciones en serie. Su nombre viene de Inter-Integrated Circuit (Inter-Circuitos Integrados). La versión 1.0 data del año 1992 y la versión 2.1 del año 2000, su diseñador es Philips. La velocidad es de 100 kbit/s en el modo estándar, aunque también permite velocidades de 3.4 Mbit/s. Es un bus muy usado en la industria, principalmente para comunicar microcontroladores y sus periféricos en sistemas integrados (Embedded Systems) y generalizando más para comunicar circuitos integrados entre si que normalmente residen en un mismo circuito impreso.

La principal característica de I²C es que utiliza dos líneas para transmitir la información: una para los datos y por otra la señal de reloj. También es necesaria una tercera línea, pero esta sólo es la referencia (masa). Como suelen comunicarse circuitos en una misma placa que comparten una misma masa esta tercera línea no suele ser necesaria.

Varios competidores, tales como Siemens AG (posteriormente Infineon Technologies AG, empresa de comunicaciones móviles de Intel), NEC, Texas Instruments, STMicroelectronics (antes SGS-Thomson), Motorola (más tarde Freescale), y Intersil, introdujeron productos compatible I²C al mercado a mediados de 1990.

Desde el 10 de octubre del 2006, no hay obligación de aplicar derechos de licencia al protocolo I²C. Sin embargo, aún se requieren cuotas para obtener direcciones esclavas I²C asignadas por NXP.


####Observación interesante
```
SMBus, definido por Intel en 1995, es un subconjunto de I²C que define los protocolos de forma más estricta. Uno de los propósitos de SMBus es promover la robustez e interoperabilidad. En consecuencia, en la actualidad I²C incorpora sistemas de políticas y normas de SMBus, a veces apoyando tanto I²C y SMBus, que sólo requiere la reconfiguración mínima.
```

I²C en el [robot Erle](http://erlerobot.com)
--------
El robot educacional [Erle](http://erlerobot.com) incluye 3 I²C buses disponible bajo `/dev`: `/dev/i2c-0`, `/dev/i2c-1` and `/dev/i2c-2`.

----

`/dev/i2c-2` por defecto no está activada. Para activarlo debe escribir:

``` bash
echo BB-I2C1 > $SLOTS
```

Usted puede verificar que ha sido activado al escribir:
``` bash
cat $SLOTS
```
---

Inspeccionando I²C
------
Una buena herramienta para inspeccionar el bus I²C es el comando de bash `i2cdetect`:
```
i2cdetect -r 1
WARNING! This program can confuse your I2C bus, cause data loss and worse!
I will probe file /dev/i2c-1 using read byte commands.
I will probe address range 0x03-0x77.
Continue? [Y/n] 
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- 13 -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- UU UU UU UU -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         

```

Este comando envía un ping al rango `0x03` - `0x77` (en hexadecimal) y registra las respuestas en todas las direcciones por lo tanto es capaz de averiguar si hay algún dispositivo. En particular, podemos ver que las respuestas de las direcciones `0x13` y `0x68` que corresponden a la IR y el sensor IMU respectivamente.


Crear instancias de dispositivos I²C desde el espacio de usuario
------------

El kernel de Linux también permite configurar manualmente los dispositivos I²C a través de una interfaz de sysfs:

> En general, el núcleo debe saber qué dispositivos I2C están conectados y 
> en qué dirección están. Sin embargo, en ciertos casos no lo hace,






I2C
> ¿Qué aborda viven en . Sin embargo , en ciertos casos , no lo hace, por lo que un
Se añadió > interfaz sysfs para que el usuario proporcione la información . este
> Interfaz es de 2 archivos de atributos que se crean en cada bus I2C
> Directorio : new_device y delete_device . Ambos archivos son de sólo escritura y
> Deben escribir los parámetros correctos para ellos con el fin de crear una instancia adecuada,
> Respectivamente borrar, un dispositivo I2C .
>
> New_device archivo tarda 2 parámetros : el nombre del dispositivo I2C (una cadena)
> Y la dirección del dispositivo I2C ( un número , típicamente expresada en
> Hexadecimal empezando por 0x , pero también puede ser expresado en decimal . )
>
> Delete_device archivo toma un único parámetro : la dirección del I2C
> Dispositivo. Como no hay dos dispositivos pueden vivir en la misma dirección en un I2C dado
> Segmento , la dirección es suficiente para identificar de forma exclusiva el dispositivo para que sea
> Borrada .


> In general, the kernel should know which I2C devices are connected and          
> what addresses they live at. However, in certain cases, it does not, so a       
> sysfs interface was added to let the user provide the information. This         
> interface is made of 2 attribute files which are created in every I2C bus       
> directory: new_device and delete_device. Both files are write only and you      
> must write the right parameters to them in order to properly instantiate,       
> respectively delete, an I2C device.                                             
>                                                                                 
> File new_device takes 2 parameters: the name of the I2C device (a string)       
> and the address of the I2C device (a number, typically expressed in             
> hexadecimal starting with 0x, but can also be expressed in decimal.)            
>                                                                                 
> File delete_device takes a single parameter: the address of the I2C             
> device. As no two devices can live at the same address on a given I2C           
> segment, the address is sufficient to uniquely identify the device to be        
> deleted.                                                                        
>                                                                                 
> Ejemplo:                                                                        
> ```echo eeprom 0x50 > /sys/bus/i2c/devices/i2c-3/new_device```
>                                                                                 
> While this interface should only be used when in-kernel device declaration      
> can't be done, there is a variety of cases where it can be helpful:             
> * The I2C driver usually detects devices (method 3 above) but the bus           
>   segment your device lives on doesn't have the proper class bit set and        
>   thus detection doesn't trigger.                                               
> * The I2C driver usually detects devices, but your device lives at an           
>   unexpected address.                                                           
> * The I2C driver usually detects devices, but your device is not detected,      
>   either because the detection routine is too strict, or because your           
>   device is not officially supported yet but you know it is compatible.         
> * You are developing a driver on a test board, where you soldered the I2C       
>   device yourself.                                                              
>                                                                                 
> This interface is a replacement for the force_* module parameters some I2C      
> drivers implement. Being implemented in i2c-core rather than in each            
> device driver individually, it is much more efficient, and also has the         
> advantage that you do not have to reload the driver to change a setting.        
> You can also instantiate the device before the driver is loaded or even         
> available, and you don't need to know what driver the device needs.       


Considering this, we could instantiate a sensor (hih6130) connected to `/dev/i2c-1` and with address `0x27` doing:

```
echo hih6130 0x27 > /sys/bus/i2c/devices/i2c-1/new_device 
```

After that, the device will be available under `/sys/bus/i2c/drivers/hih6130/1-0027`.

To remove the device you can use:
```
echo 0x27 > /sys/bus/i2c/devices/i2c-1/delete_device
```

Sources
------
- [Wikipedia](http://en.wikipedia.org/wiki/I%C2%B2C)
- [Linux kernel documentation](http://lxr.free-electrons.com/source/Documentation/i2c/instantiating-devices)
