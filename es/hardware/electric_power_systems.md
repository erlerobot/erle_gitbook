# Electric Power Systems
El siguiente contenido presenta un modelo de motor simplificado.

### El motor ideal
Imaginemos un motor eléctrico sencillo que llamaremos *Motor Ideal*. Lo llamaremos *ideal* porque tiene unas características extremadamente simples y es 100% eficiente. Nuestro motor tiene una sola cualidad: por cada voltio de energía girará exactamente 1000 *revoluciones por minuto* (RPM). Si aplicamos 5V el motor girará 5000 RPM o 25V el motor girará a 25000 RPM. Las revolusiones por minuto serán igual a la cantidad de voltios multiplicado por una constande de 1000.

|Voltios |	RPM|
|------|------------|
|1 voltios|	1000 RPM|
|2 voltios|	2000 RPM|
|3 voltios|	3000 RPM|
|4 voltios|	4000 RPM|

Ahora necesitamos una fuente de energía para nuestro motor. Imaginemos que tenemos una *celda ideal* que se conecta al *motor ideal*. Por ahora, nuestra *celda ideal* puede ser definida con una característica: La cantidad de tensión que produce. Puesto que somos un amantes de las cosas simples (algunos dirán que es porque parecen atractivas), vamos a elegir un voltaje facil de entender para nuestra *celda ideal*. Cada *celda ideal* produce exactamente 1 voltio de electricidad.

Así que ahora podemos hacer un paquete de baterías ideal y conectarlas a nuestro a motor. Vamos a ver que los resultados podrían ser:

| # de celdas ideales |	RPM |
|---------|--------|
| 1	| 1000 RPM |
| 2	|2000 RPM |
| 3 |3000 RPM |
| 4	|4000 RPM |

---

Si observa detenidadmente la tabla superior se parece mucho a la primera tabla. Nuestro *motor ideal* y *celda ideal* hacen las cosas muy sencillas para nosotros. Casi odemos usar las palabras *celda* y *coltios* indistintamente a cause de nuestros valores artificiales. Naturalmente el mundo real no es tan sencillo, pero no es del todo diferente a nuestro mundo ideal. Tenga esto en mente a medida que avanzamos en la explicación.

----

La relación entre voltio (celdas) y RPM para nuestro sistema ideal puede funcionar tanto en un sentido como en el otro. Así que si medidos que el motor está girando a 4000 RPM puedo apostar lo que sea a que la tensión de entrada es precisamente 4 voltios. Esto a su vez significa que se están usando 4 celdas. Simple por el momento.

Pero ten en cuento que nuesto motor da vueltas pero no hace nada! Necesitamos añadir algo al eje de salida de manera que al girar pueda mover una gran cantidad de aire. Así que vamos a poner una hélice a nuestro motor y ver que pasa con las RPM. De hech, vamos a comparar dos hélices diferentes con el mismo motor y distinto número de celdas.

Si eres nuevo en el mundo del radiocontrol, entonces necesitas saber como de define un hélice. Cada hélice tiene un *diámetro* y un *pitch*. La palabra *pitch* puede ser un poco confusa. Se refiere a la distancia que la hélice puede moverse hacia delante en un revolusión en un medio perfecto. Cuanto más alto sea el *pitch* más alto será el ángulo de las palas y más rápido se moverá en un sola revolución. Las hélices con un alto *pitch* se utilizan generalmente en aviones rápidos mientras que las hélices con bajo *pitch* son usadas en aviones lentos.

En Estados Unidos las hélices se especifican en pulgadas. Vamos a utilizar dos hélices para nuestro ejemplo.
- Una que tenga 5 pulgadas de diámetro y un *pitch* de 5 pulgadas, que vamos a designar como 5x5.
- La otra tendrá un diámetro de 12 pulgadas y un *pitch* de 8 pulgadas, que vamos a designar como 12x8.

|# ideal de celdas|	hélices|	RPM|
|-----|-----|
|1	|5x5	|1000 RPM|
|2|	5x5|	2000 RPM|
|3|	5x5|	3000 RPM|
|4|	5x5|	4000 RPM|

|# ideal de celdas|	hélices|	RPM|
|-----|-----|
|1|	12x8|	1000 RPM|
|2|	12x8|	2000 RPM|
|3|	12x8|	3000 RPM|
|4|	12x8|	4000 RPM|

Nada en la tabla anterior debería sorprender porque nuestro motor ideal siempre gira a `1000 RPM` por cada voltio. Independientemente del tipo de carga que se coloque en el eje de salida. Este es un punto crucial y su analogía con el mundo real es la permite a mucho principiantes comprender la electricidad completamente.

Por supuesto también hayq eu darse cuenta de que se necesit mucha más energía para girar una hélice de `12x8` a 4000 RPM que hacer girar una hélice de 5x5 a la misma velocidad. Tiene que haber algo que falta en nuestro modelo de motor simplificado.

De hecho, lo que nos falta se denomina **corriente**. La corriente es la otra mitad de la ecuación de la energía. Lamentablemente, no podemos ir más lejos sin la introducción de una formula en la discusión:
```
Watios = Voltios x Amperios
```
---

Nota esta formula se expresa en sus unidades, la fórmula física adecuada es `P = I x V`

---

Vamos a funcionar con watios a partir de ahora. ¿Recuerda la última table que mostraba el motor ideal acoplada a un número variable de celdas ideales para girar dos hélices diferentes? Recuerda cómo las RPM siempre dependen exclusivamente d ela canitdad de voltios que se aplican al motor, a pesar de que se necesita mucho más esfuerzo a para mover una hélice más grande que una pequeña. Vamos a utilizar watios para mostrar cuanto esfuerzo está involucrado.

---

Nota: Estos valores no son necesariamente reales.

---

| |	5x5	|12x8|
|---|----|----|
|1000 RPM|	1 watios|	10 watios|
|2000 RPM|	4 watios|	40 watios|
|3000 RPM|	10 watios|	100 watios|
|4000 RPM|	25 watios|	250 watios|

Tenga en cuenta que se necesita mucha más potencia (energía) para girar una hélice de 12x8 a 4000 RPM que para girar una de 5x5. También se necesita más del doble de potencia para hacer girar una hélice al doble de RPM.

Puesto que ahora sabemos cómo expresar la potencia `watios = voltios x amperios`, podemos poner un ejemplo de la tabala anterior y ver lo que está pasando con nuestro *motor ideal*. Vamos a concentrarnos en la entrada de la tabla que muestra que se necesutan `100 watios` para hacer girar una hélice de `12x8` a `3000 RPM`. Ya que sabemos que los watios es el producto de los voltios por los amperios esto significa que necesitaremos una combinación como los siguiente ejemplos:

- `100 watios = 1 voltio x 100 amperios`
- `100 watios = 2 voltios x 50 amperios`
- `100 watios = 3 voltios x 33 amperios`
-    ...

Así que podemos conseguir relamente 100 watios de muchas maneras diferentes. Nuestro motor, sin embargo, está cableado para darnos exactamente 1000 RPM por voltio independientemente de cualquier otra cosa. Dado que estamos tratando de hacer girar nuestra hélice a 3000 RPM, esto significa que tenemos 3 voltios. Y, por lo tanto, nuestro motor deber necesita 33 amperios de corriente de nuestras celdas ideales. Esta es una conclusión ineludible. 

Vamos a echar un vistazo a una de nuestras tables y completar con la corriente y watios apropiados:

|# ideal de celdas|	Corriente	|Hélices|	RPM|	Potencia|
|----|----|----|----|----|
|1|	1|	5x5|	1000|	1 watt|
|2|	2|	5x5|	2000|	4 watios|
|3|	3|	5x5|	3000|	10 watios|
|4|	6|	5x5|	4000|	25 watios|

|# ideal de celdas|	Corriente	|Hélices|	RPM|	Potencia|
|----|----|----|----|----|
|1|	10|	12x8|	1000|	10 watios|
|2|	20|	12x8|	2000|	40 watios|
|3|	33|	12x8|	3000|	100 watios|
|4|	63|	12x8|	4000|	250 watios|

Así que esto es lo que hemos aprendido hasta el momento

- Nuestro motor gira a 1000 RPM por cada voltio.
- Nuestri motor consume la corriente necesaria para hacer que los watios de energía eléctrica sea igual a la potencia que necesita para girar la hélice velocidad demandad por el voltaje.
- Los watios son el prodecto de los voltios por los amperios.
- Una hélice grande requiere mucha más energía para girar a unos revoluciones por minuto concreo que una hélice más pequeña.
- Dado un motor y una hélice, el aumento de tensión incrementará la corriente de forma exponencial

### Fuentes

- http://www.rebelpeon.com/quadcopter-research-and-education/
- http://www.rcgroups.com/forums/showthread.php?t=333326
