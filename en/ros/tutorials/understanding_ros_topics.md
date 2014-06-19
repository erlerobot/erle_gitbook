# Understanding ROS Topics

This tutorial introduces ROS topics as well as using the `rostopic` commandline tools.


----

Before we move forward, let's start by making sure that we have roscore running, in a new terminal:

```
$ roscore
If you left roscore running from the last tutorial, you may get the error message:

roscore cannot run as another roscore/master is already running.
Please kill other roscore/master processes before relaunching
This is fine. Only one roscore needs to be running.
```

----


###ROS Topics

The `rostopic` tool allows you to get information about ROS topics.

You can use the help option to get the available sub-commands for `rostopic`

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

