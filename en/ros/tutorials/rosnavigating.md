# Navigating the ROS Filesystem

This tutorial introduces ROS filesystem concepts, and covers using the roscd, rosls, and rospack commandline tools using [Erle board](http://erlerobot.com) through all the process.

### Quick Overview of Filesystem Concepts
- *Packages*: Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.
- *Manifest* (package.xml): A manifest is a description of a package. Its serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

### Filesystem Tools
Code is spread across many ROS packages. Navigating with command-line tools such as ls and cd can be very tedious which is why ROS provides tools to help you.

#### Using rospack

`rospack` allows you to get information about packages. In this tutorial, we are only going to cover the find option, which returns the path to package.

Usage:
``` bash
# rospack find [package_name]
```

---

If you get the following error:
``` bash
rospack find roscpp
terminate called after throwing an instance of 'std::runtime_error'
  what():  locale::facet::_S_create_c_locale name not valid
Aborted

```
you can solve it setting the following environment variable:
``` bash
export LC_ALL="en_US.UTF-8"
```
---

Example:

``` bash
root@erlerobot:~# rospack find roscpp
/opt/ros/groovy/share/roscpp
```

#### Using roscd

roscd is part of the rosbash suite. It allows you to change directory (cd) directly to a package or a stack.

Usage:
``` bash
# roscd [locationname[/subdir]]
```
Run this example:
``` bash
root@erlerobot:~# roscd roscpp
root@erlerobot:/opt/ros/groovy/share/roscpp# pwd
/opt/ros/groovy/share/roscpp
```

You can see that `/opt/ros/groovy/share/roscpp` is the same path that rospack find gave in the previous example.

Note that `roscd`, like other ROS tools, will only find ROS packages that are within the directories listed in your `ROS_PACKAGE_PATH` environment variable. To see what is in your `ROS_PACKAGE_PATH`, type:

``` bash
root@erlerobot:~# echo $ROS_PACKAGE_PATH
/root/catkin_ws/src:/opt/ros/groovy/share:/opt/ros/groovy/stacks
```
You `ROS_PACKAGE_PATH` should contain a list of directories where you have ROS packages separated by colons.

Similarly to other environment paths, you can add additional directories to your `ROS_PACKAGE_PATH`, with each path separated by a colon ':'.

#####Subdirectories

roscd can also move to a subdirectory of a package or stack.

Try:

``` bash
root@erlerobot:~# roscd roscpp/cmake
root@erlerobot:/opt/ros/groovy/share/roscpp/cmake# pwd
/opt/ros/groovy/share/roscpp/cmake

```

#### roscd log

`roscd` log will take you to the folder where ROS stores log files. Note that if you have not run any ROS programs yet, this will yield an error saying that it does not yet exist.

``` bash
root@erlerobot:~# roscd log
No active roscore
```

If we launch `roscore` and then run the same command in another window (or launch `roscore` in background appending `&` at the end):
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


#### Using rosls

`rosls is part of the [rosbash](http://wiki.ros.org/rosbash) suite. It allows you to ls directly in a package by name rather than by absolute path.

Usage:
``` bash
# rosls [locationname[/subdir]]
```

Example:

``` bash
root@erlerobot:~/catkin_ws/devel# rosls catkin
cmake  package.xml
```

#### Tab Completion

It can get tedious to type out an entire package name. In the previous example, roscpp_tutorials is a fairly long name. Luckily, some ROS tools support TAB completion.

Start by typing:
``` bash
# roscd catk<<< now push the TAB key >>>
```
After pushing the TAB key, the command line should fill out the rest:
``` bash
$ roscd catkin/
```
This works because `catkin` is currently the only ROS package that starts with `catkin`.

### Review
You may have noticed a pattern with the naming of the ROS tools:

- `rospack` = ros + pack(age)
- `roscd` = ros + cd
- `rosls` = ros + ls

This naming pattern holds for many of the ROS tools.

### Environment variables
ROS strongly relays on the shell environment variables. Make sure that everything is configured as expected:

``` bash
root@erlerobot:~/catkin_ws/devel# export|grep ROS
declare -x ROS_DISTRO="groovy"
declare -x ROS_ETC_DIR="/opt/ros/groovy/etc/ros"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/root/catkin_ws/src:/opt/ros/groovy/share:/opt/ros/groovy/stacks"
declare -x ROS_ROOT="/opt/ros/groovy/share/ros"
declare -x ROS_TEST_RESULTS_DIR="/root/catkin_ws/build/test_results"

```

Now that you can get around in ROS, let's [create a package](creating_a_ros_package.md).
