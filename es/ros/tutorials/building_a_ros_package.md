# Constuyendo un paquete de ROS

Este tutorial cubre la cadena de herramientas para construir un paquete.

###Construyendo paquetes
Mientras todas las dependencias estén instaladas en su sistema, podemos construir el nuevo paquete.

Antes de continuar recuerda configurar tu entorno si no lo has hecho aún (en las imagenes proporcionadas esto se hace en el fichero `.bashrc`).

``` bash
source /root/catkin_ws/devel/setup.bash
```

####Usando catkin_make
`catkin_make` es una herramienta de la línea de comandos que añade algunas cosas al flujo de trabajo de `catkin`. Tu puedes imaginar que [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) combina las llamadas `cmake` y `make` en el _standard CMake workflow_.

Uso:

```
# In a catkin workspace
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

Para las personas que no están familiarizadas con el flujo de trabajo de CMake:

---

NOTA: Si ejecutas el siguiente comando no funcionará, ya que esto es solo un ejemplo de como funciona CMake

---

``` bash
# In a CMake project
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  # (optionally)
```

Este proceso se ejecuta para proyecto CMake. En comparación con los proyectos catkin que se pueden construir juntos en el mismo espacio de trabajo.  

``` bash
# In a catkin workspace
$ catkin_make
$ catkin_make install  # (optionally)
```

Los comandos anteriores construirán los poryectos catkin que se encuentran en la carpeta src. Esto sigue las recomendaciones formuladas en [REP128](http://www.ros.org/reps/rep-0128.html). Si el código fuente está en un lugar diferente, por ejemplo `my_src`, entonces se llamaría a `catkin_make`:

---

NOTA: Si ejecuta esta comando no funcionará, ya que el directorio `my_src` no existe. 

---

```bash
# In a catkin workspace
$ catkin_make --source my_src
$ catkin_make install --source my_src  # (optionally)
```

Para usos más avanzados de `catkin_make+  revise las documentación: [catkin/commands/catkin_make](http://wiki.ros.org/catkin/commands/catkin_make).

####Construyendo el paquete

Debes de estar en el espacio de trabajo de catkin y con paquete llamado  `erle_beginner_tutorials`. Ve al espacio de trabajo de catkin y mira la carpeta `src`:

```bash
root@erlerobot:~# cd ~/catkin_ws/src
root@erlerobot:~/catkin_ws/src# ls
CMakeLists.txt  erle_beginner_tutorials
````

Ahora podemos construir el paquete usando `catkin_make`:

```bash
root@erlerobot:~# cd ~/catkin_ws
root@erlerobot:~/catkin_ws# catkin_make
Base path: /root/catkin_ws
Source space: /root/catkin_ws/src
Build space: /root/catkin_ws/build
Devel space: /root/catkin_ws/devel
Install space: /root/catkin_ws/install
####
#### Running command: "cmake /root/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/root/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/root/catkin_ws/install" in "/root/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /root/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /root/catkin_ws/devel;/opt/ros/groovy
-- This workspace overlays: /root/catkin_ws/devel;/opt/ros/groovy
-- Found gtest sources under '/usr/src/gtest': gtests will be built
-- Using CATKIN_TEST_RESULTS_DIR: /root/catkin_ws/build/test_results
-- catkin 0.5.65
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - erle_beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws/build
####
#### Running command: "make -j1 -l1" in "/root/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /root/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /root/catkin_ws/devel;/opt/ros/groovy
-- This workspace overlays: /root/catkin_ws/devel;/opt/ros/groovy
-- Found gtest sources under '/usr/src/gtest': gtests will be built
-- Using CATKIN_TEST_RESULTS_DIR: /root/catkin_ws/build/test_results
-- catkin 0.5.65
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - erle_beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws/build

```

Tenga en cuenta que `catkin_make` primero muestra cuales son las rutas que se están utilizando para uno sde los 'espacios'. Estos espacios son descritos en [REP128](http://www.ros.org/reps/rep-0128.html) y documentado en la wiki de catkin [catkin/workspaces](http://wiki.ros.org/catkin/workspaces). Lo importante es darse cuenta de que se han creado varias carpetas en el espacio de trabajo de catkin. Echa un vistazo a esto:


```bash
root@erlerobot:~/catkin_ws# ls
build  devel  install  src
```
La carpeta de compilación está en la ubicación predetermianda y es donde `cmake`se llama para configurar y contruir el paquete. La carpeta `devel` es la ubicación por defecto donde los ejecutables y librerías van antes de ser instaladas. 
