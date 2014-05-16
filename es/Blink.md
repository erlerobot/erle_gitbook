Hola mundo en robótica: parpadeo de un LED(hola)
======

Objetivo
-----
En este tutorial empezaremos con un ejemplo práctico que demostrará *cómo hacer que uno de los LEDs del robot parpadeen*.

Material
-----
Para seguir el tutorial necesitas:
- El robot Erle.
- Una tarjeta microSD de 8 GB con la *Distrución Ubuntu Linux* que proporcionamos.
- un conector (macho) USB-to-miniUSB
- Un ordenador con un terminal serial instalado (en Windows puedes usar PuTTy in en *Unixes* `minicom`).



Tutorial
-----

Erle tiene cuatro LEDs, usr0 - usr3 accesibles a través de la interfaz sysfs en `/sys/class/leds`. Esta interfaz permite abstraer los LEDs a través de ficheros, esto significa que **es posible controlar los LEDs escribiendo en un fichero**. Genial, ¿verdad?.

Nuestro objetivo será hacer que uno de estos user LEDs parpadeen utilizando sencillas instrucciones de la shell de Unix.

![leds](img/leds.jpg)

En primer lugar nos movemos al directorio sysfs correspondiente:
```
cd /sys/class/leds
```

Si revisas el contenido de este directorio (`ls`) deberías ver algo como esto:
```
erlerobot:green:usr0@
erlerobot:green:usr1@
erlerobot:green:usr2@
erlerobot:green:usr3@
```
Cada uno de estos directorios te permite modificar el comportamiento del LED correspondiente. Dentro de cada uno de estos directorios hay ficheros que se encargan de activar los LEDs de diferentes formas. Vamos a probar el `usr0`:
```
cd erlerobot:green:usr0@
ls
```
El contenido de listar (escribiendo `ls`) este directorio debe ser:
```
brightness  device@  max_brightness  power/  subsystem@  trigger  uevent
```
Como hemos dicho anteriormente, cada uno de estos ficheros te permite controlar el comportamiento del LED correspondiente y para ello basta con escribir en ellos *el contenido adecuado*. También es posible conocer el estado de los LEDs leyendo estos ficheros.
Para conseguir que el LED se encienda o apague basta con escribir en el fichero `brightness`.
Para encender el LED `usr0` escribe:
```
echo 1 > brightness
```
y para apagarlo:
```
echo 0 > brightness
```

---

Pon el LED `usr0` en modo `heartbeat` (latido de corazón):

```js
# escribe aquí

```

```js
# la solución (asumiendo que estás en el directorio usr0)
echo "heartbeat" > trigger
```

```js
assert(x == 10);
```

---


