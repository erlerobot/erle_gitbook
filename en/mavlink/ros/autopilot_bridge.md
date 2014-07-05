# `autopilot_bridge` ROS package

The [autopilot_bridge](https://github.com/mikeclement/autopilot_bridge) ROS package bridges to autopilot protocols. For now it supports only MAVLink.

#### Compiling `autopilot_bridge`

Compilation is performed as described in the [mavlink_ros:compiling instructions](mavlink_ros.md). The package should be downloaded into `~/catkin_ws/src` and `catkin_make` should be called from `catkin_ws` (refer to [ROS: Building a ROS Package](../../ros/tutorials/building_a_ros_package.md) for learning more about ROS package compilation).


#### Running `autopilot_bridge`

Over a serial connection:
```bash
rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200 --skip-time-hack
```

---

*NOTE: this requires both ardupilot and ROS running in the machine*

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

*NOTE: By default, the bridge waits to collect a time hack from the autopilot (`SYSTEM_TIME` message with non-zero fields). If running indoors or without GPS, you can specify `--skip-time-hack` to disable this behavior. In that case, the bridge uses the computer's time for all published messages.*

----

To run over a TCP connection:
```bash
rosrun autopilot_bridge mavlink.py --device tcp:127.0.0.1:6000
```

#### Playing with `autopilot_bridge`

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

Let's listen to the `autopilot/imu` ROS topic:

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

another one:

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

#### Visualizing `autopilot_bridge`

![node-graph](../../img/mavlinkROS/autopilot_bridge_graph.png)

The ROS topic `/autopilot/imu` can be _echoed_  but not plotted with rqt_plot.
