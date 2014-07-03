# ADC

El procesador tiene 8 entradas ADC (conversor analogíco a digital). Las señales son de 1.8V. Uno de ellos, `AD7`, esta conectado a la PMIC (Circuito integrado de administración de energía) TPS65217B y utilizado para medir los voltajes y corriente a través de la TPS65217B.

### Entradas ADC
El proposito principal de las entradas ADC fue diseñado para usarlo como un controlador de pantalla táctil, pero puede ser utilizado como un ADC de propósito general. Cada señal es un registro de aproximación (SAR) de 12 bits. La frecuencia de muestreo es de 100K muestras por segundo. Sólo hay un ADC en el procesador y que se puede conectar a cualquier de los 8 pines ADC.

### Interfaz VDD_ADC
La señal `VDD_ADC` se proporciona a través de la cabecera de expansión, pero *no es una pista que sea usada para dar corriente en la placa de expansión*. Se suministra desde la pista de 1.8V del TPS65217B y se ejecuta a través de un inductor para el aislamiento del ruido. Es allí, donde si hay necesidad de circuiteria externa se tiene acceso a la pista VREF del ADC o para añadir si fuera necesario un filtado adicional a través de un condensador.

###Licencia

Parte del contenido es un derivado de la  *BeagleBone System Reference Manual Rev A6.0.0* bajo licencia *Creative Commons Attribution-Share Alike 3.0 Unported License*. Para ver una copia de esta licencia, visite [http://creativecommons.org/licenses/by-sa/3.0/] (http://creativecommons.org/licenses/by-sa/3.0/) o envíe una carta a * Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, EE.UU. *. 

Todos los trabajos derivados deben ser atribuidos a Gerald Coley de BeagleBoard.org. 

Para cualquier pregunta, preocupaciones o cuestiones que se someten a gerald [at] BeagleBoard.org.
