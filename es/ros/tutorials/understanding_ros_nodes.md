# Understanding ROS Nodes

This tutorial introduces ROS graph concepts and discusses the use of `roscore`, `rosnode`, and `rosrun` commandline tools.

###Quick Overview of Graph Concepts
- [Nodes](http://wiki.ros.org/Nodes): A node is an executable that uses ROS to communicate with other nodes.
- [Messages](http://wiki.ros.org/Messages): ROS data type used when subscribing or publishing to a topic.
- [Topics](http://wiki.ros.org/Topics): Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
- [Master](http://wiki.ros.org/Master): Name service for ROS (i.e. helps nodes find each other)
- [rosout](http://wiki.ros.org/rosout): ROS equivalent of stdout/stderr
- [roscore](http://wiki.ros.org/roscore): Master + rosout + parameter server (parameter server will be introduced later)


###Nodes
A node really isn't much more than an executable file within a ROS package. ROS nodes use a ROS client library to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a Service.

###Client Libraries
ROS client libraries allow nodes written in different programming languages to communicate:

- rospy = python client library
- roscpp = c++ client library

### `roscore`
`roscore` is the first thing you should run when using ROS.

Please run:

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
`rosnode` displays information about the ROS nodes that are currently running. The `rosnode list` command lists these active nodes:

```bash
root@erlerobot:~# rosnode list
/rosout
```

This showed us that there is only one node running: rosout. This is always running as it collects and logs nodes' debugging output.

The `rosnode info` command returns information about a specific node.

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

Now, let's see some more nodes. For this, we're going to use rosrun to bring up another node.

### `rosrun`

`rosrun` allows you to use the package name to directly run a node within a package (without having to know the package path).

Usage:

```
$ rosrun [package_name] [node_name]
```

In a new terminal:
``` bash
root@erlerobot:~# rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200
Starting mavlink <-> ROS interface over the following link:
  device:		/dev/ttyO4
  baudrate:		115200

Waiting for AP heartbeat
```

In another terminal:

```bash
root@erlerobot:~# rosnode list
/autopilot
/rosout
```

One powerful feature of ROS is that you can reassign Names from the command-line.

Go back to the rosrun autopilot_bridge terminal and use ctrl-C (there seems to be a problem with this node so you might need to open a new terminal). Now let's re-run it, but this time use a Remapping Argument to change the node's name:

```bash
root@erlerobot:~# rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200 __name:=my_autopilot
Starting mavlink <-> ROS interface over the following link:
  device:		/dev/ttyO4
  baudrate:		115200

Waiting for AP heartbeat
```
Now, if we go back and use rosnode list:

```bash
root@erlerobot:~# rosnode list
/my_autopilot
/rosout
```

Let's use another rosnode command, ping, to test that it's up:

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

### Review
What was covered:

- `roscore` = ros+core : master (provides name service for ROS) + rosout (stdout/stderr) + parameter server (parameter server will be introduced later)
- `rosnode` = ros+node : ROS tool to get information about a node.
- `rosrun` = ros+run : runs a node from a given package.
Now that you understand how ROS nodes work, let's look at how ROS topics work. Also, feel free to press Ctrl-C to stop turtlesim_node.
