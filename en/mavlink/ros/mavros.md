# `mavros` ROS package

This package (available [here](https://github.com/vooon/mavros)) implemment a MAVLink extendable communication node for ROS with UDP proxy for Ground Control Station. It includes the following features:

- Communication with autopilot via serial port
- UDP proxy for Ground Control Station
- mavlink_ros compatible ROS topics (Mavlink.msg)
- Plugin system for ROS-MAVLink translation
- Parameter manipulation tool
- Waypoint manipulation tool


### Making mavlink as a library

Before compiling mavros, this is a necessary step that might take you some time if not done properly so we'll explain how to do it:

Go to your ROS *workspace* and clone the following package `https://github.com/vooon/mavros`.

```
cd <catkin-ros-workspace>
cd src
git clone https://github.com/vooon/mavros
```

Now `checkout` the branch that corresponds with your ROS installation:
```
  master
  remotes/origin/HEAD -> origin/master
  remotes/origin/debian/hydro/mavlink
  remotes/origin/debian/hydro/precise/mavlink
  remotes/origin/debian/hydro/quantal/mavlink
  remotes/origin/debian/hydro/raring/mavlink
  remotes/origin/debian/indigo/mavlink
  remotes/origin/debian/indigo/saucy/mavlink
  remotes/origin/debian/indigo/trusty/mavlink
  remotes/origin/master
  remotes/origin/patches/debian/hydro/mavlink
  remotes/origin/patches/debian/hydro/precise/mavlink
  remotes/origin/patches/debian/hydro/quantal/mavlink
  remotes/origin/patches/debian/hydro/raring/mavlink
  remotes/origin/patches/debian/indigo/mavlink
  remotes/origin/patches/debian/indigo/saucy/mavlink
  remotes/origin/patches/debian/indigo/trusty/mavlink
  remotes/origin/patches/release/hydro/mavlink
  remotes/origin/patches/release/indigo/mavlink
  remotes/origin/release/hydro/mavlink
  remotes/origin/release/indigo/mavlink
  remotes/origin/upstream

```

Finally go to the `worspace` again and execute `catkin_make_isolated --install-space /opt/ros/hydro/ --install` (we recommend you to leave solely the mavlink package just cloned).

You should see something like this:
```
root@erlerobot:~/catkin_ws_hydro# catkin_make_isolated --install-space /opt/ros/hydro/ --install
Base path: /root/catkin_ws_hydro
Source space: /root/catkin_ws_hydro/src
Build space: /root/catkin_ws_hydro/build_isolated
Devel space: /root/catkin_ws_hydro/devel_isolated
Install space: /root/catkin_ws_hydro/install_isolated
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~~  traversing 1 packages in topological order:
~~  - mavlink (plain cmake)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The packages or cmake arguments have changed, forcing cmake invocation

==> Processing plain cmake package: 'mavlink'
==> Creating build directory: 'build_isolated/mavlink/devel'
==> cmake /root/catkin_ws_hydro/src/mavlink-gbp-release -DCMAKE_INSTALL_PREFIX=/root/catkin_ws_hydro/devel_isolated/mavlink in '/root/catkin_ws_hydro/build_isolated/mavlink/devel'
-- Found PythonInterp: /usr/bin/python (found suitable version "2.7.3", required is "2")
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/ardupilotmega.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/autoquad.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/common.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/matrixpilot.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/minimal.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/pixhawk.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/slugs.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/test.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/ualberta.xml
-- processing: /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/sensesoar.xml
-- Using Debian Python package layout
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws_hydro/build_isolated/mavlink/devel
==> make -j1 -l1 in '/root/catkin_ws_hydro/build_isolated/mavlink/devel'
Scanning dependencies of target ardupilotmega.xml-v1.0
[ 10%] Generating ardupilotmega.xml-v1.0-stamp
Validating /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/ardupilotmega.xml
Parsing /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/ardupilotmega.xml
Note: message DATA64 is longer than 64 bytes long (74 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message DATA96 is longer than 64 bytes long (106 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Validating /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/common.xml
Parsing /root/catkin_ws_hydro/src/mavlink-gbp-release/message_definitions/v1.0/common.xml
...
```

`mavlink` package should be compiled and installed. Let's now compile `mavros`.

#### Compiling `mavros`

Compilation is performed as described in the [mavlink_ros:compiling instructions](mavlink_ros.md). The package should be downloaded into `~/catkin_ws/src` and `catkin_make` should be called from `catkin_ws` (refer to [ROS: Building a ROS Package](../../ros/tutorials/building_a_ros_package.md) for learning more about ROS package compilation).

----

*WARNING: The `mavros` package is available for hydro+ distros. Take it into account.*

----

