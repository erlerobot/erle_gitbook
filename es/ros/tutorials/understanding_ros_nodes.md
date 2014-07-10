# Understanding ROS Nodes

Este tutorial los conceptos de ROS y discute el uso de las herramientas de línea de comando `roscore`, `rosnode` y `rosrun`.

###Resumen
- [Nodos](http://wiki.ros.org/Nodes): Un nodo es un ejecutable que usa ROS para comunicarse con otros nodos.
- [Mensajes](http://wiki.ros.org/Messages): Tipo de dato de ROS que es utilizando durante la suscripción y publicación de un *topic*.
- [Topics](http://wiki.ros.org/Topics): Los nodos pueden publicar mensajes a través de un *topic*. Se puede suscribirse a ellos para recibir mensajes.
- [Master](http://wiki.ros.org/Master): Nombre de servicio para ROS(por ejemplo ayuda a encontrar otros nodos)
- [rosout](http://wiki.ros.org/rosout): La salida equivalente de ROS a stdout/stderr
- [roscore](http://wiki.ros.org/roscore): Master + rosout + parameter server (parameter server será introcudido más adelante)


###Nodos
Un nodo en realidad no es más que un archico ejecutable dentro de un paquete de ROS. Los nodos de ROS utilizan una biblioteca cliente para comunicarse con otros nodos. Los nodos pueden publicar o suscribirse a un *topic*. Los nodos pueden utilizar o proporcionar algún servicio.

###Libraría cliente
La biblioteca cliente de ROS permite a escribir los nodos de ROS en diferentes lenguajes de programación:

- rospy = python client library
- roscpp = c++ client library

### `roscore`
`roscore` es la primero que debe ejecutar cuando usas ROS.

Por favor ejecuta:

```bash
roscore
... logging to /root/.ros/log/e8b674a4-bff3-11d3-af73-4ed34f3752c7/roslaunch-erlerobot-2990.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://erlerobot:59324/
ros_comm version 1.9.44

SUMMARY
========

PARAMETERS
 * /rosdistro
 * /rosversion

NODES

auto-starting new master
process[master]: started with pid [3003]
ROS_MASTER_URI=http://erlerobot:11311/

setting /run_id to e8b674a4-bff3-11d3-af73-4ed34f3752c7
process[rosout-1]: started with pid [3016]
started core service [/rosout]


```

### `rosnode`
`rosnode` muestra información sobre los nodos de ROS que se están ejecutando. El comando `rosnode list` lista los nodos que están activos.

```bash
root@erlerobot:~# rosnode list
/rosout
```
Esto nos demostró que sólo hay un  nodo en ejecución: *rosout*. Este siempre se está ejecutando, ya que recoge y registra información de duración de los nodos.

El comando `rosnode info` devuelve información sobre un nodo específico.

```bash
root@erlerobot:~#  rosnode info /rosout
--------------------------------------------------------------------------------
Node [/rosout]
Publications:
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions:
 * /rosout [unknown type]

Services:
 * /rosout/set_logger_level
 * /rosout/get_loggers


contacting node http://erlerobot:49274/ ...
Pid: 3016


```

Ahora , vamos a ver algunos nodos más. PAra ellos, vamos a utilizar `rosrun` para que aparezca otro nodo.

### `rosrun`

`rosrun` permite usar el nombre del paquete para ejecutar directamente un nodo (sin necesidad de conocer la ruta del paquete).

Uso:

```
$ rosrun [package_name] [node_name]
```

En un nuevo terminal:
``` bash
root@erlerobot:~# rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200
Starting mavlink <-> ROS interface over the following link:
  device:		/dev/ttyO4
  baudrate:		115200

Waiting for AP heartbeat
```

En otro terminal:

```bash
root@erlerobot:~# rosnode list
/autopilot
/rosout
```
Una característica poderosa de ROS es que se pueden volver a asignar nombres desde la línea de comandos.

Volvemos al terminal de `rosrun autopilot_bridge` y usa ctrl-C ( parace que hay un problema con este nodo por lo que puede ser que necesite abrir un nuevo terminal). Ahora vaoms a volver a ejecutarlo, pero esta vez utiliza un argumento de vambio de nombre de nodo:

```bash
root@erlerobot:~# rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200 __name:=my_autopilot
Starting mavlink <-> ROS interface over the following link:
  device:		/dev/ttyO4
  baudrate:		115200

Waiting for AP heartbeat
```
Ahora, Si volvemos atras y usarmos el comando `rosnode list`:

```bash
root@erlerobot:~# rosnode list
/my_autopilot
/rosout
```

Vamos a usar otro comando de `rosnode`, `ping`, para probarlo:

```bash
root@erlerobot:~# rosnode ping my_autopilot
rosnode: node is [/my_autopilot]
pinging /my_autopilot with a timeout of 3.0s
xmlrpc reply from http://erlerobot:39540/	time=15.213966ms
xmlrpc reply from http://erlerobot:39540/	time=13.329983ms
xmlrpc reply from http://erlerobot:39540/	time=12.695074ms
xmlrpc reply from http://erlerobot:39540/	time=13.059974ms
xmlrpc reply from http://erlerobot:39540/	time=12.760997ms
xmlrpc reply from http://erlerobot:39540/	time=12.681007ms
^Cping average: 13.290167ms

```

### Comentarios
¿Qué se ha cubierto?:

- `roscore` = ros+core : master (proporciona un nombre de servicio para ROS) + rosout (stdout/stderr) + parameter server (parameter server será introducido más tarde)
- `rosnode` = ros+node : Herramienta de ROS que obtiene información sobre un nodo.
- `rosrun` = ros+run : ejecuta un nodo de un paquete.
Ahora que usted entiene cómo hacen su trabajo los nodos de ROS. PAsaremos a ver cómo funcionan los *topics* de ROS.