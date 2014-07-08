#`mavlink_ros` ROS package

El paquete `mavlink_ros` es un *puente serie entre MAVLink y ROS*. Este paquete de ROS crea un nodo que permite enviar y recibir paquetes MAVLink a través de una interfaz serie.

El paquete [mavlink_ros](https://github.com/mavlink/mavlink_ros) se programó originalmente por Lorenz Meier para *rosmake* (versiones antigua de ROS). El paquete (**catkinized**) actualizado esta disponible [aquí](https://github.com/y22ma/mavlink_ros).

#### Compilando `mavlink_ros`

Para usar `mavlink_ros` es necesario clonar el repositorio del directorio `~/catkin_ws/src`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/y22ma/mavlink_ros
```

Luego necesitamos compilar el paquete con `catkin_make`:
```bash
cd  ~/catkin_ws/
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
-- ~~  traversing 2 packages in topological order:
-- ~~  - erle_beginner_tutorials
-- ~~  - mavlink_ros
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- +++ processing catkin package: 'mavlink_ros'
-- ==> add_subdirectory(mavlink_ros)
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- Found GLIB2: /usr/lib/arm-linux-gnueabihf/libglib-2.0.so
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- Found GTHREAD2: /usr/lib/arm-linux-gnueabihf/libgthread-2.0.so
-- mavlink_ros: 1 messages, 0 services
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
-- ~~  traversing 2 packages in topological order:
-- ~~  - erle_beginner_tutorials
-- ~~  - mavlink_ros
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- +++ processing catkin package: 'mavlink_ros'
-- ==> add_subdirectory(mavlink_ros)
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- mavlink_ros: 1 messages, 0 services
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws/build
Scanning dependencies of target mavlink_ros_gencpp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 20%] Generating C++ code from mavlink_ros/Mavlink.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 20%] Built target mavlink_ros_gencpp
Scanning dependencies of target mavlink_ros_genlisp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 40%] Generating Lisp code from mavlink_ros/Mavlink.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 40%] Built target mavlink_ros_genlisp
Scanning dependencies of target mavlink_ros_genpy
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 60%] Generating Python from MSG mavlink_ros/Mavlink
[ 80%] Generating Python msg __init__.py for mavlink_ros
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 80%] Built target mavlink_ros_genpy
Scanning dependencies of target mavlink_ros_serial
make[2]: Warning: File `/opt/ros/groovy/include/XmlRpcDecl.h' has modification time 4.2e+08 s in the future
[100%] Building CXX object mavlink_ros/CMakeFiles/mavlink_ros_serial.dir/src/mavlink_ros_serial.cpp.o
Linking CXX executable /root/catkin_ws/devel/lib/mavlink_ros/mavlink_ros_serial
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
```

#### Ejecutando `mavlink_ros`

```bash
rosrun mavlink_ros mavlink_ros_serial -p /dev/ttyO5 -b 115200
```

```bash
root@erlerobot:~/catkin_ws# rosrun mavlink_ros mavlink_ros_serial -p /dev/ttyO5 -b 115200
Trying to connect to /dev/ttyO5.. success.
Trying to configure /dev/ttyO5.. success.

READY, waiting for serial/ROS data.

Connected to /dev/ttyO5 with 115200 baud, 8 data bits, no parity, 1 stop bit (8N1)

MAVLINK SERIAL TO ROS BRIDGE STARTED ON MAV 42 (COMPONENT ID:110) - RUNNING.

```

---

*NOTA: Se require que ardupilot y ROS estén ejecutando en la mismo maquina*

---

En otro terminal:

```bash
root@erlerobot:~# rosnode list
/mavlink_ros_serial
/rosout
root@erlerobot:~# rostopic list
/fcu/imu
/fcu/mag
/fcu/raw/imu
/mavlink/from
/mavlink/to
/rosout
/rosout_agg
root@erlerobot:~# rosnode info mavlink_ros_serial
--------------------------------------------------------------------------------
Node [/mavlink_ros_serial]
Publications:
 * /mavlink/from [mavlink_ros/Mavlink]
 * /fcu/mag [sensor_msgs/MagneticField]
 * /rosout [rosgraph_msgs/Log]
 * /fcu/imu [sensor_msgs/Imu]
 * /fcu/raw/imu [sensor_msgs/Imu]

Subscriptions:
 * /mavlink/to [unknown type]

Services:
 * /mavlink_ros_serial/get_loggers
 * /mavlink_ros_serial/set_logger_level


contacting node http://erlerobot:59208/ ...
Pid: 999
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS


```
