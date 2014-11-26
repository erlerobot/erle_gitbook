# BBB+PXF

![](../img/hardware/PXF.jpg)

El [PixHawk Fire Cape](https://github.com/diydrones/PXF) (PXF) es una *capa* para el BeagleBone Black desarrollado por Philip Rowse. La placa se usa para apoyar el `AP_HAL_Linux` de ardupilot.


### Instrucciones de configuración para el PXF v1.0
#### Preparando el PXF

La revisión `1.0` tiene un problema en el arranque que hace que el BB o el BBB no arranque. Una solución rápida para este asunto es doblar los pins *P8* `46` a `31` para que no entren.

![](../../en/img/hardware/PXF_bended.jpg)

#### Preparando the BBB

----

*No conecte todavía el al BeagleBone Black.*

----

Obtener la [imagen de la microSD]() que proporcionamos y verific¡que que la BeagleBone Black funciona. Recomendamos utilizar la BB para propositos de desarrollo (ya que tiene disponibilidad de un puerto serie del miniUSB que permite lanzar una consola serie a través de `minicom` o `screen`).

El BB (o el BBB) debe arrancar correctamente. En este punto estaremos dispuestos a poner la capa sobre la BB (o BBB).


