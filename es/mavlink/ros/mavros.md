# `mavros` ROS package


`mavros` es un nodo de comunicación extendsible a MAVLink de ROS con proxy UDP para la Estación de Control de Tierra. Esta documentado [aquí](http://wiki.ros.org/mavros) que incluye las siguientes características:

- Comuncicación con ardupilot a través de puerto serie
- Proxy UDP con la Estación de Control de Tierra
- mavlink_ros compatible con ROS Mavlink.msg
- Plugin para la traducción de ROS-MAVLink
- Herramienta de manipulación de parámetros
- Herramienta de manipulación de puntos de interés

### utilizando mavlink como una librería

Antes de compilar mavros, es necesario este paso que podría llevar algo de tiempo si no se hace correctamente, así que se explicará como hacerlo:

Ve a su *espacio de trabajo* de ROS y clona el siguiente paquete:

-----

*En lugar de clonar el repositorio manualmente, se puede utilizar `wstool`.*

-----

-----

*NOTA: Un `.deb` puede ser generado para la BeagleBone y esta disponible en https://github.com/vmayoral/ros-stuff/tree/master/deb*

-----

```
cd <catkin-ros-workspace>
cd src
git clone https://github.com/vooon/mavros
```
Ahora haz `checkout` de la rama que corresponde con la instalación de ROS:
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

Finalmente ve a tu `worspace` de nuevo y ejecuta `catkin_make_isolated --install-space /opt/ros/hydro/ --install`.

Deberías de ver algo como esto:
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

El paquete `mavlink` debería estar compilado e instalado. Ahora compilemos `mavros`.

### Compilando `mavros`

Realice la compilación como se describe en [mavlink_ros:compiling instructions](mavlink_ros.md). El paquete debe de ser descargado en `~/catkin_ws/src` y `catkin_make` debe de ser ejecutado desde `catkin_ws` (revisar [ROS: Building a ROS Package](../../ros/tutorials/building_a_ros_package.md) para aprender más sobre el proceso de compilación de paquetes ROS).
----

*ATENCIÓN: El paquete `mavros` esta disponible para la distribución hydro+. Tengo esto en cuenta.*

----



### Ejecutando `mavros`

Por ahora, `mavros` solo soporta un dispositivo serie en el lado del autopiloto. Esto significa que si queremos ejecutarlo con una conexión TCP debes crear un PTY utilizando herramientas como [socat](http://www.dest-unreach.org/socat/).

Empecemos ejecutando:
``` bash
ArduCopter.elf -A tcp:*:6000:wait
```
Podemos facilmente verificar que se lanza un socket en todas las interfaces escuchando en el puerto 6000:
``` bash
root@erlerobot:~# netstat -nap|grep Ardu
tcp        0      0 0.0.0.0:6000            0.0.0.0:*               LISTEN      24632/ArduCopter.elf
```
Ahora crearemos nuestro enlace *TCP-tty* utilizando `socat`:
``` bash
socat  pty,link=/dev/ttyAutopilot,raw  tcp:127.0.0.1:6000&
```
Si comprobamos `/dev` deberiamos ver algo como esto:
```bash
root@erlerobot:~# ls /dev
alarm            fuse          loop0         mmcblk0p2           ram12   rfkill     tty11  tty23  tty35  tty47  tty59         ttyS0     vcs3   vport0p0
ashmem           i2c-0         loop1         net                 ram13   rtc0       tty12  tty24  tty36  tty48  tty6          ttyS1     vcs4   watchdog
autofs           i2c-1         loop2         network_latency     ram14   shm        tty13  tty25  tty37  tty49  tty60         ttyS2     vcs5   watchdog0
binder           input         loop3         network_throughput  ram15   snd        tty14  tty26  tty38  tty5   tty61         ttyS3     vcs6   zero
block            kmem          loop4         null                ram2    spidev1.0  tty15  tty27  tty39  tty50  tty62         ubi_ctrl  vcs7
btrfs-control    kmsg          loop5         ppp                 ram3    spidev2.0  tty16  tty28  tty4   tty51  tty63         uinput    vcsa
bus              log           loop6         psaux               ram4    stderr     tty17  tty29  tty40  tty52  tty7          urandom   vcsa1
char             log_events    loop7         ptmx                ram5    stdin      tty18  tty3   tty41  tty53  tty8          usbmon0   vcsa2
console          logibone      loop-control  pts                 ram6    stdout     tty19  tty30  tty42  tty54  tty9          usbmon1   vcsa3
cpu_dma_latency  logibone_mem  mapper        ram0                ram7    tty        tty2   tty31  tty43  tty55  ttyAutopilot  usbmon2   vcsa4
disk             log_main      mem           ram1                ram8    tty0       tty20  tty32  tty44  tty56  ttyO0         vcs       vcsa5
fd               log_radio     mmcblk0       ram10               ram9    tty1       tty21  tty33  tty45  tty57  ttyO4         vcs1      vcsa6
full             log_system    mmcblk0p1     ram11               random  tty10      tty22  tty34  tty46  tty58  ttyO5         vcs2      vcsa7

```

Finalmente lanzamos `mavros`:
```
root@erlerobot:~# rosrun mavros mavros_node _serial_port:=/dev/ttyAutopilot _serial_baud:=115200 _gcs_host:=localhost
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

![](../../../en/img/mavlinkROS/mavros_graph.png)

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
Desafortunadamente parece que hay un problema con porque no produce ninguna salida
```bash
rostopic echo /mavros/imu/data_raw

```
`mavparam` parece que no responde tampoco:
```
rosrun mavros mavparam dump /tmp/apm.param

```
