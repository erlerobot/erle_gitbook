# `autopilot_bridge` ROS package

El paquete de ROS [autopilot_bridge](https://github.com/mikeclement/autopilot_bridge) hace de puente con los protocolos de piloto automático. Por ahora solo soporta MAVLink.

#### Compilando `autopilot_bridge`

La compilación se lleva a cabo tal y como se describe en [mavlink_ros:compiling instructions](mavlink_ros.md). El paquete tiene que ser descargado en `~/catkin_ws/src` y `catkin_make` debe ser llamado desde `catkin_ws` (Revisa [ROS: Building a ROS Package](../../ros/tutorials/building_a_ros_package.md) para aprender más acerca de compilar paquetes ROS).


#### Ejecutando `autopilot_bridge`

A través del la conexión serie:
```bash
rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200 --skip-time-hack
```

---

*NOTA: Esto requiere estar ejecutando ardupilot y ROS en la misma máquina*

---


``` bash
root@erlerobot:~# rosrun autopilot_bridge mavlink.py --device /dev/ttyO5 --baudrate 115200 --skip-time-hack
Starting mavlink <-> ROS interface over the following link:
  device:		/dev/ttyO5
  baudrate:		115200

Waiting for AP heartbeat
Heartbeat from AP (sys 1 comp 1 custom_mode 0)
Sending all stream request for rate 10
[WARN] [WallTime: 946693474.238413] Skipping time hack from autopilot, using saved system time

Starting autopilot loop...

```

----

*NOTA: Por defecto, el punto espera a recoger un tiempo del piloto automático (el mensahe `SYSTEM_TIME` con campos distintos de cero). Si se ejecuta en interiores o sin GPS, hay que especificar `--skip-time-hack` para deshabilitar este comportamiento. En este caso, el punto utiliza la hora del ordenador para todos los mensajes publicados.*

----

Para ejecutar a través de un conexión TCP:
```bash
rosrun autopilot_bridge mavlink.py --device tcp:127.0.0.1:6000
```

#### Jugando con `autopilot_bridge`

```bash
root@erlerobot:~# rosnode list
/autopilot
/rosout
root@erlerobot:~# rostopic list
/autopilot/arm
/autopilot/gps
/autopilot/gps_odom
/autopilot/guided_goto
/autopilot/heartbeat
/autopilot/imu
/autopilot/land
/autopilot/land_abort
/autopilot/mode
/autopilot/status
/autopilot/waypoint_goto
/rosout
/rosout_agg
```

Ahora escuchemos en `autopilot/imu`:

```bash
root@erlerobot:~# rostopic echo /autopilot/imu
header:
  seq: 1
  stamp:
    secs: 946693613
    nsecs: 904042959
  frame_id: base_footprint
orientation:
  x: 0.0100524548726
  y: 0.184838964438
  z: 0.00133298901126
  w: 0.982716504653
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
header:
  seq: 2
  stamp:
    secs: 946693614
    nsecs: 185938954
  frame_id: base_footprint
orientation:
  x: 0.0131375821778
  y: 0.217347333887
  z: 0.00157136571938
  w: 0.976004647118
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
...
```

Otro más:

```bash
root@erlerobot:~# rostopic echo /autopilot/status
header:
  seq: 1
  stamp:
    secs: 946685514
    nsecs: 678575992
  frame_id: ''
mode: 15
armed: False
ahrs_ok: False
alt_rel: -1247770
as_ok: False
as_read: 0.0
gps_ok: False
gps_sats: 0
gps_eph: 0
ins_ok: False
mag_ok: False
mis_cur: 0
pwr_ok: True
pwr_batt_rem: -1
pwr_batt_vcc: 0
pwr_batt_cur: -1
---
header:
  seq: 2
  stamp:
    secs: 946685516
    nsecs: 646455049
  frame_id: ''
mode: 15
armed: False
ahrs_ok: False
alt_rel: -463450
as_ok: False
as_read: 0.0
gps_ok: False
gps_sats: 0
gps_eph: 0
ins_ok: False
mag_ok: False
mis_cur: 0
pwr_ok: True
pwr_batt_rem: -1
pwr_batt_vcc: 0
pwr_batt_cur: -1
---

```

#### Visualizando `autopilot_bridge`

![node-graph](../../img/mavlinkROS/autopilot_bridge_graph.png)

El `/autopilot/imu` puede tener *eco*, pero no puede ser representado con rqt_plot.
