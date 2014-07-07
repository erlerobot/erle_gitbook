# Installing and Configuring BeaglePilot

Como se dijo antes, [BeaglePilot](https://github.com/BeaglePilot/ardupilot) se basa en `AP_HAL_Linux` de [ardupilot](https://github.com/diydrones/ardupilot).

Instalando BeaglePilot es tan fácil como clonar el código:

```
git clone https://github.com/BeaglePilot/ardupilot
cd ardupilot
```
[ardupilot](https://github.com/diydrones/ardupilot) soporta por el momento, tres tipos de vehículos *copters, *aviones*, y *rovers*. Nos centraremos en los *copters* por ahora:
```
cd ArduCopter
```
Ahora generaremos algunos archivos de configuración (solo se necesita una vez):
```
make configure
```
Y finalmente se compila el código:
```
make erle
```

----

En un BeagleBone (white) que ejecutaa 720 MHz (perf. mode) la compilación tardas aproximadamente 10 minutos.

----

Después de la compilación, debes tener `ArduCopter.build/ArduCopter.elf` en el directorio `/tmp`.

----

Si desea almacenar los ejecutables en otro lugar debes definir la variable de entorno `TMPDIR`.

----
