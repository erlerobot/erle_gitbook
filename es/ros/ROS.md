El sistema operativo de Robots (ROS): Powering the world's robots
=========

![](../img/ros/rosorg-nb.png)

¿Qué es ROS?
-----
[Robot Operative System](http://www.ros.org/) (ROS) es un  ** meta sistema operativo de códido abierto** para robots que es mantenido por la [Open Source Robotics Foundation](http://www.osrfoundation.org/) (OSRF). Proporciona los servicios que caben esperar de un sistema operativo, incluyendo abstracción de hardware, control de dispositivos a bajo nivel, la implementación de la funcionalidad de uso común, paso de mensajes entre procesos y gestión de paquetes. También proporciona herramientas y bibliotecas para la obtención, construcción, escritutra y ejecución de código en varios equipos. ROS es similar en algunos aspectos a otros "framework" robóticos como: Player, YARP, Orocos, CARMEN, Orca, MOOS o Microsoft Robotics Studio. 

ROS ejecuta como una *red de procesos peer-to-peer* que están acoplados utilizando la infraestructura de de comunicación de ROS. Implementa varios estilos diferentes de comuncicación, incluyendo la transmición síncrona de servicios través de RPC,  la transmición síncrona de datos a través de *topics* y el almacenamiento de datos a través de *parámetros en el servidor*.

**ROS no es un framework de tiempo real**, aunque es posible intregrar ROS con el código en tiempo real. ROS tiene un perfecta integración con *Orocos Real-time Toolkit*.

Objetivos
-----

El objetivo princpila de ROS es ** dar soporto y reutilizar código para el desarrollo e investigación de la robótica**. ROS es un framework distribuido de procesos (nodos) que permite a los ejecutables ser diseñados de manera individual y se acoplan de manera flexible en tiempo de ejecución. Estos procesos se pueden agrupar en paquetes y pilas, que peuden ser fácilmente compartidos y distribuidos. ROS también es compatible con un sistema federado de repositorios de código que permite la colaborar a distribuirlo. Este diseño desde el nivel del sistema de archivos hasta el nivel de la comunidad de usuarios, permite tomar decisiones independientes sobre el desarrollo e implementación, pero esto solo es posible con las infraestructura de herramientas de ROS.

En apoyo de este objetivo de compartir y colaborar, hay otros objeticos en el framework de ROS:

- **Thin**: ROS esta diseñado para ser lo más ligero posible -- no vamos a eliminar el main() -- para que el códifo escrito para ROS se pueda utilizar con otros frameworks de desarrollo robótico. Un colorario de esto es que ROS se puede integrar facilmente con otros frameworks robóticos : ROS esta integrado con  OpenRAVE, Orocos, and Player.
- Librerias **ROS-agnostic**: el modelo de desarrollo perferido es escribir bibliotecas con interfaces funcionales limpias.
- **Independencia del lenguaje*: ROS es fácil de implementar en cualquier lenguaje de programación modero. Ya esta implementado en  *Python*, *C++*, y *Lisp*, y existen *librerias experimentales en Jaba y Lua*.
- **Fácil de testear**: ROS tiene un framework de test llamado `rostest`que permite realizar pruebas de manera sencilla.
- **Escalable**: ROS es paropiado para sistema de tiempo de ejecución grandes y para grandes procesos de desarrollo.

Sistemas operativos
-------
En la actulidad ROS sólo ejecuta en las **plataformas basadas en Unix**. El software de ROS se testea en *Ubuntu* y *Mac OS X*.

Distribuciones
----------
LAs siguientes distribuciones han sido testeadas en [Erle](http://erlerobot.com).

----

**Por defecto las imágenes que proporciona Erle vienen con ROS preinstalado y completamente funcional.**

----

| Distro | Release date | Poster | Instuctions |
|--------|--------------|--------|-------------|
| [ROS Hydro Medusa](http://wiki.ros.org/hydro) | September 4th, 2013 | ![medusa](../../en/img/ros/hydro.png) | [Installation](http://wiki.ros.org/hydro/Installation/UbuntuARM) |
| [ROS Groovy Galapagos](http://wiki.ros.org/groovy) | December 31, 2012 | ![medusa](../../en/img/ros/galapagos.jpg) | [Installation](http://wiki.ros.org/groovy/Installation/UbuntuARM) |


Licencia
-------
Parte del material ha sido tomado de la [wiki de ROS](http://wiki.ros.org/). Excepto donde se indique lo contrario. El wiki de ROS esta bajo licencia *Creative Commons Attribution 3.0*.
