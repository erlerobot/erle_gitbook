# USB

### Puerto USB0
El HUB se conecta directamente al puerto USB0 en el procesador. Esto permite que el puerto sea accesible desde el mismo conector USB como los puertos serie y JTAG.

#### Puerto USB Cliente
El acceso a USB0 se proporciona a través del HUB USB integrado. Se mostrará en un PC como un dispositivo serie estándar USB (generalmente `/dev/tty[algo]`)

### Puerto USB1
En la plca hay un solo conector USB tipo A con compatibilidad LS/FS/HS con el host que se conecta a USB1 en el procesador. El puerto puede proporcionar energía de control de encencido o apagado y **hasta 500mA** de corriente a 5V.

```
Bajo la alimientación del USB, la placa no será de capaz de suministrar los 500mA completos, pero debe ser suficiente para suministrar corriente para un dispositivo de alimentación USB inferior.
Si se necesita más corriente se recomienda el uso de una batería.

Se pueden usar un teclado inalámbrico, un ratón o una pantalla o puede agregar un HUB para la interconexión con los dispositivos en caso necesario.
```
