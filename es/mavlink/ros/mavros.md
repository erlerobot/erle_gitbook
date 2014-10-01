# Paquete ROS `mavros`

 `mavros` es un nodo ROS que extiende las comuniciones mediente MAVLink mediante un proxy para una Estación de Control de Tierra. Está documentado [aquí](http://wiki.ros.org/mavros).

Este paquete (disponible [aquí](https://github.com/vooon/mavros)) implementa un nodo ROS que extiende las comuniciones mediente MAVLink mediante un proxy para una Estación de Control de Tierra que incluye las siguientes características:

- Comunicación el autopiloto a través de puerto serie.
- Proxy UDP para ka Estación de Control en Tierra
- mavlink_ros es compatible con los Topics de ROS (Mavlink.msg)
- Sistema de plugins para la traducción de ROS-MAVLink
- Herramienta de manipulación de parámetros.
- Herramienta de manipulación de Waypoint


### mavlink como una librería

Antes de compilar, es necesario realizar el siguiente paso que podría llegar algún tiempo si no se realiza correctamente, por lo tanto, vamos a explicar como hacerlo:

Ve al *espacio de trabajo* de ROS y clona el siguiente paquete `https://github.com/vooon/mavros`.

-----

*En lugar de clonar el repositorio manualmente se puede utilizar `wstool`.
-----

-----

*Nota: Un `.deb` ha sido generado para BeagleBone y esta disponible en https://github.com/vmayoral/ros-stuff/tree/master/deb*

-----

```
cd <catkin-ros-workspace>
cd src
git clone https://github.com/vooon/mavros
```

Ahora `checkout` la rama que corresponde con la instalación ROS:
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

Finalmente ve al *espacio de trabajo* de nuevo y ejecuta `catkin_make_isolated --install-space /opt/ros/hydro/ --install` (le recomendamos que deje únicamente el paquete mavlink).

Debes ver algo como esto:
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

El paquete `mavlink` debe de compilarse e instarse. Vamos a compilarlo.

### Compilando `mavros`

LA compilación se realizó como se describe en [mavlink_ros:compiling instructions](mavlink_ros.md). El paquete debe de ser descargado en `~/catkin_ws/src` y `catkin_make` debe de ser llamado desde `catkin_ws` (visite [ROS: Building a ROS Package](../../ros/tutorials/building_a_ros_package.md) para aprender más sobre como compilar paquetes ROS).

----

*Atención: El paquete `mavros` esta disponible para la distribución hydro+. Tenga esto en cuenta*

----

### Ejecutando `mavros`

Ahora lanzamos el Autopiloto:
``` bash
ArduCopter.elf -A tcp:*:6000:wait
```

Podemos facilmente verficar que se lanza un socket en todas las interfaces en el puerto 6000:
``` bash
root@erlerobot:~# netstat -nap|grep Ardu
tcp        0      0 0.0.0.0:6000            0.0.0.0:*               LISTEN      24632/ArduCopter.elf
```

Lanzamos `mavros`:
```
root@erlerobot:~# rosrun mavros mavros_node _fcu_url:=192.168.7.2
[ INFO] [1404307547.212478943]: serial: device: /dev/ttyAutopilot @ 115200 bps
[ INFO] [1404307547.232303568]: udp: Bind address: 0.0.0.0:14555
[ INFO] [1404307547.246258485]: udp: GCS address: 127.0.0.1:14550
[ INFO] [1404307548.616922693]: Plugin Command [alias command] loaded and initialized
[ INFO] [1404307548.745958777]: Plugin GPS [alias gps] loaded and initialized
[ INFO] [1404307548.965828360]: Plugin IMUPub [alias imu_pub] loaded and initialized
[ INFO] [1404307549.109628860]: Plugin Param [alias param] loaded and initialized
[ INFO] [1404307549.200207318]: Plugin RCIO [alias rc_io] loaded and initialized
[ INFO] [1404307549.357753943]: Plugin SystemStatus [alias sys_status] loaded and initialized
[ INFO] [1404307549.588623860]: Plugin Waypoint [alias waypoint] loaded and initialized
[ INFO] [1404307549.597910485]: MAVROS started on MAV 1 (component 240)

```

#### Jugando con `mavros`

![](../../img/mavlinkROS/mavros_graph.png)

```bash
victor@ubuntu:~$ rosnode list
/mavros
/rosout
victor@ubuntu:~$ rostopic list
/diagnostics
/mavlink/from
/mavlink/to
/mavros/battery
/mavros/fix
/mavros/gps_vel
/mavros/imu/atm_pressure
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/mag
/mavros/imu/temperature
/mavros/mission/waypoints
/mavros/rc/in
/mavros/rc/out
/mavros/state
/mavros/time_reference
/rosout
/rosout_agg

```

Necesitas establecer la tasa de muestreo:
```bash
rservice call /mavros/set_stream_rate 0 10 1
```

Ahora puedes visualizar los topics
```bash
 rostopic list
/diagnostics
/mavlink/from
/mavlink/to
/mavros/battery
/mavros/camera_image
/mavros/camera_image/compressed
/mavros/camera_image/compressed/parameter_descriptions
/mavros/camera_image/compressed/parameter_updates
/mavros/camera_image/compressedDepth
/mavros/camera_image/compressedDepth/parameter_descriptions
/mavros/camera_image/compressedDepth/parameter_updates
/mavros/camera_image/theora
/mavros/camera_image/theora/parameter_descriptions
/mavros/camera_image/theora/parameter_updates
/mavros/fix
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gps_vel
/mavros/global_position/local
/mavros/global_position/rel_alt
/mavros/gps_vel
/mavros/imu/atm_pressure
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/mag
/mavros/imu/temperature
/mavros/mission/waypoints
/mavros/mocap/pose
/mavros/optical_flow
/mavros/position/local
/mavros/position/vision
/mavros/radio_status
/mavros/rc/in
/mavros/rc/out
/mavros/rc/override
/mavros/safety_area/set
/mavros/setpoint/accel
/mavros/setpoint/att_throttle
/mavros/setpoint/cmd_vel
/mavros/setpoint/local_position
/mavros/state
/mavros/time_reference
/mavros/vfr_hud
/mavros/vision_speed/speed_vector
/rosout
/rosout_agg
/tf
```

Por ejemplo, IMU:

```bash
rostopic echo /mavros/imu/data_raw
---
header:
  seq: 11149
  stamp:
    secs: 1412071746
    nsecs: 586876282
  frame_id: fcu
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.006
  y: -0.005
  z: 0.009
angular_velocity_covariance: [1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07]
linear_acceleration:
  x: -0.24516625
  y: 0.0196133
  z: 10.52253545
linear_acceleration_covariance: [8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08]
---
```
