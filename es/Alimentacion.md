Alimentación
============

La selección de cualquiera de los 5 VDC o el USB como fuente de alimentación se maneja internamente en el TPS65217B y cambia automáticamente a la energía 5VDC aunque ambas estén conectadas. SW puede cambiar la configuración de la corriente a través de la interfaz I2C del procesador. Además, el SO puede leer el TPS65217B y determinar si la placa se está ejecutando en la entrada de 5 VDC o la entrada USB. Esto puede ser interesante para conocer la capacidad de la Junta para suministrar corriente para cosas como la frecuencia de funcionamiento y las tarjetas de expansión.

Administración de energía del circuito integrado (PMIC)
--------

La administración de energía del circuito integrado ( PMIC ) en el sistema la gestiona [ TPS65217B ] ( http://www.ti.com/product/tps65217b ) . El * TPS65217B * es un chip que consiste en un circuito de alimentación de doble entrada lineal , tres convertidores reductores , cuatro LDO y un impulso convertidor de alta eficiencia para soportar dos cadenas de hasta 10 LEDs en serie . El sistema se suministra por un puerto USB o adaptador de DC . Tres convertidores de alta eficiencia de 2.25MHz están dirigidos a proporcionar el voltaje del núcleo , MPU, y el voltaje de la memoria para la tarjeta .

Los convertidores reductores entran en un modo de baja potencia con cargas ligeras para una máxima eficiencia en la gama más amplia posible de las corrientes de carga . Para aplicaciones de bajo nivel de ruido de los dispositivos pueden ser forzados a PWM de frecuencia fija utilizando la interfaz I2C . Los convertidores reductores permiten el uso de pequeños inductores y condensadores para lograr un pequeño tamaño de solución .

LDO1 y LDO2 tienen como objetivo apoyar el modo de espera del sistema . En el estado SLEEP la corriente de salida se limita a 100uA para reducir la corriente de reposo , mientras que en el funcionamiento normal pueden apoyar * hasta 100 mA cada archivo *. LDO3 y LDO4 pueden soportar hasta * 285 mA cada uno * .
Por defecto ** sólo LDO1 está siempre encendido ** pero cualquier carril puede ser configurado para permanecer en estado de suspensión . Especialmente los convertidores DCDC pueden permanecer en un modo de PFM de baja potencia para soportar procesador en modo de suspensión . El TPS65217B ofrece secuencias flexibles de encendido y apagado y varias funciones de mantenimiento de la casa , tales como salida de una buena potencia , monitor de botón , la función de reinicio del hardware y el sensor de temperatura para proteger la batería .

La selección de cualquiera de los 5VDC o el USB como fuente de alimentación se maneja internamente en el TPS65217B y cambia automáticamente a la energía 5VDC aunque ambos estén conectadas . El software puede cambiar la configuración de la corriente a través de la interfaz I2C del procesador. Además, el software puede leer ** el TPS65217B y determinar si la placa se está ejecutando en la entrada de 5VDC o la entrada USB . ** Esto puede ser beneficioso para conocer la capacidad de la Junta para suministrar corriente para cosas como la frecuencia de funcionamiento y las tarjetas de expansión .

