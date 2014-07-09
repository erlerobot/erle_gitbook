# Navegando por el sistema de archicos de ROS

Este tutorial introduce los conceptos del sistema de fichero de ROS, y cubre el uso de `roscd`, `rosls` y `rospack`, herramientas de la línea de comandos usando [Erle board](http://erlerobot.com) durante todo el proceso.

### Breve resumen de los conceptos de un sistema de ficheros
- *Paquetes*: los paquetes son la unidad de organización del código de ROS. Cada paquete contiene librerias, ejecutables, scripts u otras artefactos.
- *Manifest* (package.xml): Un manifiesto es la descripción de un paquete. Sirve para definir las dependencias entre los paquetes y para capturar la información de los metadatos sobre el paquete como la versión, licencia, etc.

### Herramientas del sistema de ficheros

El código se extiende por mucho paquetes ROS. Navegar con herramientas de linea de comandos como `ls` o `cd` puede ser muy tedioso y por eso ROS proporciona herramientas para ayudarte.

#### Utilizando rospack

`rospack` permite obtener información acerca de los paquetes. Es este tutorial, sólo vamos a cubrir la opción de búsqueda, que devuelve la ruta de acceso al paquete

Uso:
``` bash
# rospack find [package_name]
```

---

Si obtiene el siguiente error:
``` bash
rospack find roscpp
terminate called after throwing an instance of 'std::runtime_error'
  what():  locale::facet::_S_create_c_locale name not valid
Aborted

```
Puedes resolverlo configurando la siguiente variable de entorno:
``` bash
export LC_ALL="en_US.UTF-8"
```
---

Por ejemplo:

``` bash
root@erlerobot:~# rospack find roscpp
/opt/ros/groovy/share/roscpp
```

#### Usando roscd

`roscd` es parte de la *suite* `rosbash`. Permite cambiar el directorio.

Uso:
``` bash
# roscd [locationname[/subdir]]
```
Ejecuta este ejemplo:
``` bash
root@erlerobot:~# roscd roscpp
root@erlerobot:/opt/ros/groovy/share/roscpp# pwd
/opt/ros/groovy/share/roscpp
```

Tu puedes ver que `/opt/ros/groovy/share/roscpp` tiene la misma ruta que `rospack` gracias al ejemplo anteriors.

NOTA: `roscd`, como otras herramientas de ROS, solo puede encontrar paquetes ROS que estén bajo el directorio definido por la variable de entorno `ROS_PACKAGE_PATH`. Para ver que contiene la variable de entorno `ROS_PACKAGE_PATH`, introduce:

``` bash
root@erlerobot:~# echo $ROS_PACKAGE_PATH
/root/catkin_ws/src:/opt/ros/groovy/share:/opt/ros/groovy/stacks
```

`ROS_PACKAGE_PATH` debería contener una lista de directorios donde están los paquetes ROS separados por dos puntos.

Al igual que otras rutas de entorno, tu puedes añadir adicionalmente directorios a `ROS_PACKAGE_PATH`, que estén seprados por ':'.

#####Subdirectorios

`roscd` puede moverse por los subdirectorios de un paquete

Prueba:

``` bash
root@erlerobot:~# roscd roscpp/cmake
root@erlerobot:/opt/ros/groovy/share/roscpp/cmake# pwd
/opt/ros/groovy/share/roscpp/cmake

```

#### roscd log

`roscd log`  te llevará donde se almacenan los archicos de registro de ROS. Tenga en cuenta que si no ha ejecutado ningún programa ROS devolverá un error indicando que todavía no existe.

``` bash
root@erlerobot:~# roscd log
No active roscore
```
Si lanzamos  `roscore` y luego ejecutamos el mismo comando en otro ventana (o lanza `roscore` en segundo plano añadiendo al final `&` ):
``` bash
root@erlerobot:~/.ros/log# roscore
... logging to /root/.ros/log/53689ab4-bfe7-11d3-af71-1e658b3e1294/roslaunch-erlerobot-1104.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://erlerobot:55088/
ros_comm version 1.9.44

SUMMARY
========

PARAMETERS
 * /rosdistro
 * /rosversion

NODES

auto-starting new master
process[master]: started with pid [1117]
ROS_MASTER_URI=http://erlerobot:11311/

setting /run_id to 53689ab4-bfe7-11d3-af71-1e658b3e1294
process[rosout-1]: started with pid [1130]
started core service [/rosout]

```

``` bash
root@erlerobot:~# roscd log
root@erlerobot:~/.ros/log/53689ab4-bfe7-11d3-af71-1e658b3e1294# ls
master.log  roslaunch-erlerobot-1104.log  rosout-1-stdout.log  rosout.log
```


#### Usando rosls

`rosls` es parte de [rosbash](http://wiki.ros.org/rosbash). Permite ir directamente a un direcorio en un paquete por el nombre en lugar del path absoluto.

Uso:
``` bash
# rosls [locationname[/subdir]]
```

Ejemplo:

``` bash
root@erlerobot:~/catkin_ws/devel# rosls catkin
cmake  package.xml
```

#### Completando con el tabulador

Puede resultar tedioso escribir un nombre de todo el paquete. En el ejemplo anterior, roscpp_tutorials es un nombre bastante largo. Por suerte, algunas herramientas de ROS permiten autocompletar el nombre con el tabulador.

Empieza escribiendo:
``` bash
# roscd catk<<< now push the TAB key >>>
```
Después de pulsar el tabulador, la línea de comandos debería rellanar el resto:
``` bash
$ roscd catkin/
```
Esto funciona porque `catkin` porque actualmente es el único paquete de ROS que empieza por `catkin`.

### Revisión
Debes de haber notado que hay un patrón con el nombramiento de las herramientas de ROS:

- `rospack` = ros + pack(age)
- `roscd` = ros + cd
- `rosls` = ros + ls

Este patrón de nombramiento se mantiene para muchas herramientas de ROS.

### Variables de entorno
ROS depende en gran medida de variables de entorno. Asegurate de que todo está configurado como se espera:

``` bash
root@erlerobot:~/catkin_ws/devel# export|grep ROS
declare -x ROS_DISTRO="groovy"
declare -x ROS_ETC_DIR="/opt/ros/groovy/etc/ros"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/root/catkin_ws/src:/opt/ros/groovy/share:/opt/ros/groovy/stacks"
declare -x ROS_ROOT="/opt/ros/groovy/share/ros"
declare -x ROS_TEST_RESULTS_DIR="/root/catkin_ws/build/test_results"

```

Ahora [crearemos un paquete](creating_a_ros_package.md).
