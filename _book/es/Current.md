Medición de corriente
===========

El BEAGLEBONE tiene un método según el cual se puede medir el consumo de corriente de la junta, sin contar el puerto Host USB y tarjetas de expansión. La caída de tensión en una resistencia de 0,1 ohmios se mide para determinar el consumo de corriente.

! [actual] ( img / currentmeas.png )

Conexión `SYS_5V`
----------
El carril `SYS_5V` se mide para determinar la parte alta de la resistencia en serie . El carril ` SYS_5V ` está conectado a la ` pasador ` MUX_OUT . Antes de ser conectado al segundo multiplexor interno, la tensión se divide por 3 . Una señal de ` 5V ` dará lugar a una tensión de 1.66V ` ` en el ` pasador ` MUX_OUT .


Conexión ` SYS_VOLT `
-------
El carril ` SYS_VOLT ` se mide para determinar la parte alta de la resistencia en serie . El carril ` SYS_VOLT ` está conectado a la MUX_OUT mediante el establecimiento de los registros dentro de la TPS65217B . Las resistencias R2 ` ` y ` R1 ` se proporcionan para mantener la misma configuración de divisor de tensión como se encuentra en el carril de ` SYS_5V ` situado interna a la TPS65217B . Sin embargo , un carril de 5V le dará ` 1.41V ` a diferencia de la ` 1.66V ` encontraron interna a la TPS65217B . Esto se resuelve a un divisor de 2,8. Asegúrese de trabajar esto en sus cálculos finales .

Conexión ` MUX_OUT `
-------
La conexión ` MUX_OUT ` se divide por 2 antes de ser conectado al procesador . La razón de esto es que si está conectado el voltaje de la batería , no tiene divisor de tensión internamente . Si está conectado podría dañar el procesador . Cuando el cálculo de las tensiones para cada lado de las resistencias , que la tensión se divide por 2 . Asegúrese e incluir esto en sus cálculos .

Cálculo actual
---------
El cálculo para el actual se basa en 0,1 mV es igual a 1 mA . Puede utilizar la siguiente fórmula para calcular la corriente utilizando las lecturas de voltaje como leído por el procesador.

`` `
( ( ( SYS_5V * 2 ) * 3,3 ) - ( ( SYS_VOLT * 2 ) * 3,54 ) ) ) / 0,1 = total ( mA )
`` `