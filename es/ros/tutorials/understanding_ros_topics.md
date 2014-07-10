# Understanding ROS Topics

Este tutorial introcude los *topics* de ROS además del uso de la herramienta de línea de comandos `rostopic`.


----
Antes de seguir, asegurate de tener ejecutando `roscore`, en un nuevo terminal:

```
$ roscore
If you left roscore running from the last tutorial, you may get the error message:

roscore cannot run as another roscore/master is already running.
Please kill other roscore/master processes before relaunching
This is fine. Only one roscore needs to be running.
```

----


###ROS *Topics*

La herramienta `rostopic` permite obtener información sobre un *topic* de ROS

Puedes usar la opcion de ayuda para ver los subcomandos disponible es `rostopic`:

```
root@erlerobot:~# rostopic -h
rostopic is a command-line tool for printing information about ROS Topics.

Commands:
	rostopic bw	display bandwidth used by topic
	rostopic echo	print messages to screen
	rostopic find	find topics by type
	rostopic hz	display publishing rate of topic
	rostopic info	print information about active topic
	rostopic list	list active topics
	rostopic pub	publish data to topic
	rostopic type	print topic type

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
```