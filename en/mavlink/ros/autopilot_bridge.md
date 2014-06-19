# `autopilot_bridge` ROS package

The [autopilot_bridge](https://github.com/mikeclement/autopilot_bridge) ROS package bridges to autopilot protocols. For now it supports only MAVLink.

#### Compiling `autopilot_bridge`

Compilation is performed as described in the [mavlink_ros:compiling instructions](mavlink_ros.md). The package should be downloaded into `~/catkin_ws/src` and `catkin_make` should be called from `catkin_ws` (refer to [ROS: Building a ROS Package](../../ros/tutorials/building_a_ros_package.md) for learning more about ROS package compilation).


#### Running `autopilot_bridge`

Over a serial connection:
```bash
rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200
```

---

*NOTE: this requires both ardupilot and ROS running in the machine*

---


``` bash
root@erlerobot:~# rosrun autopilot_bridge mavlink.py --device /dev/ttyO4 --baudrate 115200
Starting mavlink <-> ROS interface over the following link:
  device:		/dev/ttyO4
  baudrate:		115200

Waiting for AP heartbeat
Heartbeat from AP (sys 1 comp 1 custom_mode 0)
Sending all stream request for rate 10
Waiting for non-zero time hack from autopilot...
```

----

*NOTE: Gets stucked there. Furthermore, there seems to be a problem when stopping this node that freezes the terminal.*

----

To run over a TCP connection (*untested*):
```bash
rosrun autopilot_bridge mavlink.py --device tcp:127.0.0.1:5762
```
