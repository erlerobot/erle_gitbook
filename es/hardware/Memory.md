#Memoria

Se utiliza un único dispositivo de memoria DDR2 (16 bits). El diseño actual soporta 128 MB o 256 MB.

El diseño utiliza un chip MT47H128M16RT-25E:C 400MHZ de Micron que viene en un encapsulado de 84-Ball 9.0mm x 12.5mm FBGA. La configuración de la memoria se describe en la siguiente tabla:

| **Parameter** | **128 Meg x 16** |
|---------------|------------------|
| Configuration | 16 Meg x 16 x 8 banks|
| Refresh Count | 8K |
| Row Address | A[13:0] (16K)|
| Bank Address | BA[2:0] (8)|
| Column Address | A[9:0] (1K)|

La memoria DDR2 se conecta directamente al procesador sin necesidad de ninguna otra interfaz. La alimentación es a través del rail de 1.8V del *Power Management Integrated Circuit* (PMIC).

Licencia
--------
Parte del contenido es un derivado del *BeagleBone System Reference Manual Rev A6.0.0* con licencia Creative Commons Attribution-Share Alike 3.0 Unported License. Para ver una copia de esta licencia visite [http://creativecommons.org/licenses/by-sa/3.0/](http://creativecommons.org/licenses/by-sa/3.0/) o envíe una carta a *Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA*.

Todo trabajo derivado ha de ser atribuído a Gerald Coley de BeagleBoard.org. 

Preguntas, dudas y/o problemas pueden ser enviados a gerald@BeagleBoard.org.
