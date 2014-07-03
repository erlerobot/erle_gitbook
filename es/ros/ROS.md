#El Sistema Operativo de Robot ( ROS ): Encender robots del mundo


![ros](../img/rosorg-nb.png)

##¿Qué es ROS ?

El [Sistema Operativo de Robot](http://www.ros.org/) (ROS) es un **sistema operativo de código abierto** para su robot mantenida por el [Open Source Robótica Fundación](http://www.osrfoundation.org/) (OSRF). Proporciona los servicios que cabe esperar de un sistema operativo, incluyendo abstracción de hardware , control de dispositivos de bajo nivel , la implementación de la funcionalidad de uso común , entre los procesos de paso de mensajes y gestión de paquetes. También proporciona herramientas y bibliotecas para la obtención , la construcción , la escritura y la ejecución de código en varios equipos. ROS es similar en algunos aspectos a los "marcos de robots, 'como jugador , YARP , Orocos , CARMEN , Orca, MOOS , y Microsoft Robotics Studio.

El tiempo de ejecución "graph" ROS es una * red de procesos de peer -to-peer * que están débilmente acoplados utilizando la infraestructura de comunicación ROS . ROS implementa varios estilos diferentes de comunicación , incluida la comunicación sincrónica de estilo RPC a través de los servicios , la transmisión asíncrona de datos a través de los temas , y el almacenamiento de datos en un servidor * Parámetro * .

** ROS no es un marco de tiempo real ** , aunque es posible integrar de ROS con el código de tiempo real . ROS también tiene una perfecta integración con el Orocos Toolkit en tiempo real .

Objetivos
-----
El objetivo principal de ROS es ** código de soporte reutilización en la investigación y el desarrollo de la robótica . ** ROS es un marco distribuido de procesos (también conocido como nodos ) que permite a los ejecutables que ser diseñado de forma individual y de acoplamiento flexible en tiempo de ejecución . Estos procesos se pueden agrupar en paquetes y pilas , que puede ser fácilmente compartida y distribuida . ROS también es compatible con un sistema federado de repositorios de código que permite la colaboración a distribuir también. Este diseño , desde el nivel del sistema de archivos a nivel de la comunidad , permite tomar decisiones independientes sobre el desarrollo y puesta en práctica, pero todo puede ser llevado junto con herramientas de infraestructura ROS .

En apoyo de este objetivo primordial de compartir y colaborar , hay varios otros objetivos del marco de ROS:

- ** Thin **: ROS está diseñado para ser lo más fina posible - no vamos a envolver su main () - para que el código escrito para ROS se puede utilizar con otros marcos de software robot. Un corolario de esto es que los ROS es fácil de integrar con otros marcos de software robot : ROS ya ha sido integrada con OpenRAVE , Orocos y jugador.
- ** Bibliotecas ** ROS- agnósticos : el modelo de desarrollo preferido es escribir bibliotecas ROS- agnóstico con interfaces funcionales limpias .
- ** Independencia Idioma **: el marco ROS es fácil de implementar en cualquier lenguaje de programación moderno . Ya hemos implementado en Python * *, * C + + * , y * Lisp * , y tenemos bibliotecas experimentales en Java * y * Lua .
- ** Pruebas ** Fácil : ROS tiene un marco de prueba de unidad / integración incorporado llamado ROSTEST que hace que sea fácil de llevar y derribar accesorios de la prueba .
- ** ** Escala : ROS es apropiado para sistemas de tiempo de ejecución de grandes y para grandes procesos de desarrollo.

Sistemas Operativos
-------
ROS en la actualidad sólo se ejecuta en las plataformas basadas en Unix ** . ** Software de ROS se prueba principalmente en * Ubuntu * y * sistemas * X Mac OS.

Distribuciones
----------
Los siguientes distribuciones han sido probados en el robot [ Erle ]( http://erlerobot.com ) .

----

** Por defecto las imágenes proporcionadas por Erle vienen con ROS preinstalado y completamente funcional. **

----

| Distro | Fecha de salida | Foros | INSTRUCTIONS |
| -------- | -------------- | -------- | ------------- |
| [ ROS Hydro Medusa ]( http://wiki.ros.org/hydro ) | 04 de septiembre 2013 | ![Medusa](../img/hydro.png) | [Instalación](http://wiki.ros.org/hidro/Instalación/UbuntuARM ) |
| [ ROS Groovy Galápagos ]( http://wiki.ros.org/groovy ) | 31 de diciembre 2012 | ![Medusa](http://www.ros.org/images/groovygalapagos-320w.jpg) | [ Instalación]( http://wiki.ros.org/groovy/Installation/UbuntuARM ) |


licencia
-------
Parte de este material ha sido tomado de la [ ROS wiki]( http://wiki.ros.org/ ) . Excepto donde se indique lo contrario, el wiki de ROS está licenciado bajo Creative Commons Attribution 3.0.









