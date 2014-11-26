# Creado paquetes de ROS

Este tutotial explica el uso de `roscreate-pkg` o `catkin` para crear nuevos paquetes, y `rospack` para listar las dependenciar. El paquete que se creará en este tutorial esta disponible [aquí](https://github.com/erlerobot/erle_beginner_tutorials).


### ¿Qué hace que un paquete sea catkin?
PAra que un paquete sea considerado *catkin* debe cumplir una serie de requisitos:

- El paquete tiene que contener un archico complatible `package.xml`. Este fichero `package.xml` proporciona meta informacion acerca del paquete.
- El paquete debe contener un `CMakeLists.txt` que usará `catkin`..
- No debe haber más de un paquete en cada carpeta. Esto significa que no hay paquete anidados ni varios paquetes que compartan el mismo directorio.

El paquete más simple tendría el siguiente aspecto:
```
my_package/
  CMakeLists.txt
  package.xml
```

### Paquete en el espacio de trabajo de catkin
El método recomendado para trabajar con paquetes catkin packages es usar el [espacio de trabajo de catkin](http://wiki.ros.org/catkin/workspaces), pero también se puede construir paquetes catkin de manera independiente. Un espacio de trabajo trivial tendría el siguiente aspecto:
```
workspace_folder/         -- CATKIN WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file
    package_1/
      CMakeLists.txt
      package.xml
      ...
    package_n/
      CMakeLists.txt
      package.xml
      ...
  build/                  -- BUILD SPACE
    CATKIN_IGNORE         -- Keeps catkin from walking this directory
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
```

En la placa [Erle](http://erlerobot.com), el espacio de trabajo catkinesta en `/root/catkin_ws`.

Si quieres crear otro espacio de trabajo catkin, sigue las siguientes [instrucciones](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

### Creado un paquete catkin
Esta sección demuestra como usar el script `catkin_create_pkg` para crear un nuevo paquete catkin, y que hacer después de haber sido creado.

Primero cambia el espacio de trabajo al espacio de trabajo de catkin:

``` bash
# You should have created this in the Creating a Workspace Tutorial
$ cd ~/catkin_ws/src
```
Ahora utiliza el script `catkin_create_pkg` para crear un nuevo paquete denominado 'erle_beginner_tutorials' que depende de `std_msgs`, `roscpp`, y `rospy`:

``` bash
root@erlerobot:~/catkin_ws/src# catkin_create_pkg erle_beginner_tutorials std_msgs rospy roscpp
Created file erle_beginner_tutorials/package.xml
Created file erle_beginner_tutorials/CMakeLists.txt
Created folder erle_beginner_tutorials/include
Created folder erle_beginner_tutorials/src
Successfully created files in /root/catkin_ws/src/erle_beginner_tutorials. Please adjust the values in package.xml.
```

Esto crea una carpeta `erle_beginner_tutorials` que contiene un `package.xml` y un `CMakeLists.txt`, que se ha completado con al información que has entregado a `catkin_create_pkg`.

```
root@erlerobot:~/catkin_ws/src# tree erle_beginner_tutorials/
erle_beginner_tutorials/
├── CMakeLists.txt
├── include
├── package.xml
└── src

2 directories, 2 files

```

`catkin_create_pkg` requiere que des un `<package_name>` y opcionalmente un lista de dependencias de las cuales depende tu paquete:

``` bash
# This is an example, do not try to run this
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

`catkin_create_pkg` también tiene funcionalidades más avanzadas que se describen en [catkin/commands/catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg).

### Dependencia de paquetes
#### Dependencias de primer orden
Cuando utilizas `catkin_create_pkg` se proporcionan una serie de dependencias, estas dependencias se pueden revisar con la herramienta `rospack`.

``` bash
root@erlerobot:~# rospack depends1 erle_beginner_tutorials
roscpp
rospy
std_msgs
```
Como puede ver, `rospack` lista las algunas dependencias que se han introducido como argumento cuando se ejecutó `catkin_create_pkg`. Estas dependencias del paquete se almacenan en el archivo `package.xml`:

```
root@erlerobot:~# roscd erle_beginner_tutorials/
root@erlerobot:~/catkin_ws/src/erle_beginner_tutorials# cat package.xml
```
```xml
<?xml version="1.0"?>
<package>
  <name>erle_beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The erle_beginner_tutorials package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="root@todo.todo">root</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but mutiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://ros.org/wiki/erle_beginner_tutorials</url> -->


  <!-- Author tags are optional, mutiple are allowed, one per tag -->
  <!-- Authors do not have to be maintianers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *_depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use run_depend for packages you need at runtime: -->
  <!--   <run_depend>message_runtime</run_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- You can specify that this package is a metapackage here: -->
    <!-- <metapackage/> -->

    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

####Dependencias indirectas
En muchos casos, una dependencia puede tener sus propias dependencias. Por ejemplo, `rospy` tiene otras dependencias:

``` bash
root@erlerobot:~/catkin_ws/src/erle_beginner_tutorials# rospack depends1 rospy
genpy
rosgraph
rosgraph_msgs
roslib
std_msgs
```
Un paquete puede tener varias dependencias indirectas. Afortunadamente `rospack` puede determinar de forma recursiva todas las dependencias:

``` bash
root@erlerobot:~/catkin_ws/src/erle_beginner_tutorials# rospack depends erle_beginner_tutorials
cpp_common
rostime
roscpp_traits
roscpp_serialization
genmsg
genpy
message_runtime
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
catkin
rospack
roslib
rospy

```

###Personalizando el paquete 
En esta parte del tutorial verá cada archico generado por `catkin_create_pkg` y describirá, línea por línea, cada componente de esos archivos y cómo se puede personalizar el paquete.

####Personalizando el archico package.xml
El archivo generado `package.xml` debería estar en tu nuevo paquete. Ahora vamos a ir a través de `package.xml` y revisando cada elemento.

####Etiqueta *description*
Primero actulizado la etiqueta `description`:
```xml
  <description>The erle_beginner_tutorials package</description>
```
Cambia la descripción como quieras, pero por convención, la frase tiene que ser corta, cubriendo el objetivo del paquete. Si es difícil describir el paquete en un sola frase, entonces puede ser que necesite replantearse el paquete.

####Etiqueta *maintainer* 

Ahora viene la etiqueta *maintainer*

```xml
  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <maintainer email="victor@erlerobot.com">vmayoral</maintainer>

```
Se trato de un etiqueta obligatoria e importante para `package.xml` porque permite a otros saber con quién contactar acerca de este paquete. Al menos un persona es necesaria, pero se pueden añadir más si quieres. El nombre del maintainer tiene que ir en el cuerpo del tag, pero también hay un atributo para el email que debe ser rellenado:

####Etiqueta *license*

La siguiente etiqueta es *license*, que requiere: 

```xml
  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>BSD</license>
```

Debes escoger una licencia e indicarla aquí. Algunas licencias de código abierto comunes son BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, and LGPLv3. Puedes leer más acerca de esto en [Open Source Initiative](http://opensource.org/licenses/alphabetical). Para este tutorial se usa la licencia BSD porque el resto de los componentes de ROS la utilizan:

---

**Nosotros recomendamos compartir tu código bajo un licencia de la Open Source Initiative**

![](../../img/ros/opensource.png)

La [Open Source Initiative](http://opensource.org/about) (OSI) es una organización sin ánimo de lucro con ámbito global para educar y abogar sobre los beneficios de código abierto y construir puentes entre los diferentes grupos de interés en la comunidad de código abierto.

---

####Etiqueta *dependencies*

El siguiente conjunto de etiquetas describen las dependencias del paquete. Las dependencias se dividen en `build_depend`, `buildtool_depend`, `run_depend`, `test_depend`. Para más detalles sobre esta etiqueta revise la documentación [Catkin Dependencies](http://wiki.ros.org/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies). Desde que pasamos `std_msgs`, `roscpp`, and `rospy` como argumentos a `catkin_create_pkg`, las dependencias tendrán el siguiente aspecto:

```xml
  <!-- The *_depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use run_depend for packages you need at runtime: -->
  <!--   <run_depend>message_runtime</run_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>

```

####Final package.xml

Como puedes ver el aspecto final de `package.xml`, sin comentarios ni etiquetas vacias:

```xml
<?xml version="1.0"?>
<package>
  <name>erle_beginner_tutorials</name>
  <version>0.0.1</version>
  <description>The erle_beginner_tutorials package</description>

  <maintainer email="victor@erlerobot.com">vmayoral</maintainer>

  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>

  <export>
  </export>
</package>

```
