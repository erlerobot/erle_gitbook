# `mavros` ROS package

This package (available [here](https://github.com/vooon/mavros)) implemment a MAVLink extendable communication node for ROS with UDP proxy for Ground Control Station. It includes the following features:

- Communication with autopilot via serial port
- UDP proxy for Ground Control Station
- mavlink_ros compatible ROS topics (Mavlink.msg)
- Plugin system for ROS-MAVLink translation
- Parameter manipulation tool
- Waypoint manipulation tool

#### Compiling `mavros`

Compilation is performed as described in the [mavlink_ros:compiling instructions](mavlink_ros.md). The package should be downloaded into `~/catkin_ws/src` and `catkin_make` should be called from `catkin_ws` (refer to [ROS: Building a ROS Package](../../ros/tutorials/building_a_ros_package.md) for learning more about ROS package compilation).

----

*WARNING: Invoking, `catkin_make` for compiling `mavros` has proved to be time consuming. The package should come pre-compiled with the provided SD-card Erle Robotics images*

----

*COMPILATION ERRORS*: After fixing several issues there're still some open matters. The package hasn't compiled yet:

```bash
root@erlerobot:~/catkin_ws# catkin_make
Base path: /root/catkin_ws
Source space: /root/catkin_ws/src
Build space: /root/catkin_ws/build
Devel space: /root/catkin_ws/devel
Install space: /root/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/root/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /root/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /root/catkin_ws/devel;/opt/ros/groovy
-- This workspace overlays: /root/catkin_ws/devel;/opt/ros/groovy
-- Found gtest sources under '/usr/src/gtest': gtests will be built
-- Using CATKIN_TEST_RESULTS_DIR: /root/catkin_ws/build/test_results
-- catkin 0.5.65
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 14 packages in topological order:
-- ~~  - actionlib_msgs
-- ~~  - common_msgs (metapackage)
-- ~~  - diagnostic_msgs
-- ~~  - erle_beginner_tutorials
-- ~~  - geometry_msgs
-- ~~  - nav_msgs
-- ~~  - sensor_msgs
-- ~~  - autopilot_bridge
-- ~~  - mavlink_ros
-- ~~  - mavros
-- ~~  - shape_msgs
-- ~~  - stereo_msgs
-- ~~  - trajectory_msgs
-- ~~  - visualization_msgs
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'actionlib_msgs'
-- ==> add_subdirectory(common_msgs/actionlib_msgs)
-- actionlib_msgs: 3 messages, 0 services
-- +++ processing catkin metapackage: 'common_msgs'
-- ==> add_subdirectory(common_msgs/common_msgs)
-- +++ processing catkin package: 'diagnostic_msgs'
-- ==> add_subdirectory(common_msgs/diagnostic_msgs)
-- diagnostic_msgs: 3 messages, 1 services
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- +++ processing catkin package: 'geometry_msgs'
-- ==> add_subdirectory(common_msgs/geometry_msgs)
-- geometry_msgs: 23 messages, 0 services
-- +++ processing catkin package: 'nav_msgs'
-- ==> add_subdirectory(common_msgs/nav_msgs)
-- Generating .msg files for action nav_msgs/GetMap /root/catkin_ws/src/common_msgs/nav_msgs/action/GetMap.action
Generating for action GetMap
-- nav_msgs: 12 messages, 2 services
-- +++ processing catkin package: 'sensor_msgs'
-- ==> add_subdirectory(common_msgs/sensor_msgs)
-- sensor_msgs: 25 messages, 1 services
-- +++ processing catkin package: 'autopilot_bridge'
-- ==> add_subdirectory(autopilot_bridge)
-- autopilot_bridge: 3 messages, 0 services
-- +++ processing catkin package: 'mavlink_ros'
-- ==> add_subdirectory(mavlink_ros)
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- mavlink_ros: 1 messages, 0 services
-- +++ processing catkin package: 'mavros'
-- ==> add_subdirectory(mavros)
-- mavros: 7 messages, 15 services
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/ardupilotmega.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/autoquad.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/common.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/matrixpilot.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/minimal.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/pixhawk.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/slugs.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/test.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/ualberta.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/sensesoar.xml
-- +++ processing catkin package: 'shape_msgs'
-- ==> add_subdirectory(common_msgs/shape_msgs)
-- shape_msgs: 4 messages, 0 services
-- +++ processing catkin package: 'stereo_msgs'
-- ==> add_subdirectory(common_msgs/stereo_msgs)
-- stereo_msgs: 1 messages, 0 services
-- +++ processing catkin package: 'trajectory_msgs'
-- ==> add_subdirectory(common_msgs/trajectory_msgs)
-- trajectory_msgs: 2 messages, 0 services
-- +++ processing catkin package: 'visualization_msgs'
-- ==> add_subdirectory(common_msgs/visualization_msgs)
-- visualization_msgs: 10 messages, 0 services
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
-- ~~  traversing 14 packages in topological order:
-- ~~  - actionlib_msgs
-- ~~  - common_msgs (metapackage)
-- ~~  - diagnostic_msgs
-- ~~  - erle_beginner_tutorials
-- ~~  - geometry_msgs
-- ~~  - nav_msgs
-- ~~  - sensor_msgs
-- ~~  - autopilot_bridge
-- ~~  - mavlink_ros
-- ~~  - mavros
-- ~~  - shape_msgs
-- ~~  - stereo_msgs
-- ~~  - trajectory_msgs
-- ~~  - visualization_msgs
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'actionlib_msgs'
-- ==> add_subdirectory(common_msgs/actionlib_msgs)
-- actionlib_msgs: 3 messages, 0 services
-- +++ processing catkin metapackage: 'common_msgs'
-- ==> add_subdirectory(common_msgs/common_msgs)
-- +++ processing catkin package: 'diagnostic_msgs'
-- ==> add_subdirectory(common_msgs/diagnostic_msgs)
-- diagnostic_msgs: 3 messages, 1 services
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- +++ processing catkin package: 'geometry_msgs'
-- ==> add_subdirectory(common_msgs/geometry_msgs)
-- geometry_msgs: 23 messages, 0 services
-- +++ processing catkin package: 'nav_msgs'
-- ==> add_subdirectory(common_msgs/nav_msgs)
-- Generating .msg files for action nav_msgs/GetMap /root/catkin_ws/src/common_msgs/nav_msgs/action/GetMap.action
Generating for action GetMap
-- nav_msgs: 12 messages, 2 services
-- +++ processing catkin package: 'sensor_msgs'
-- ==> add_subdirectory(common_msgs/sensor_msgs)
-- sensor_msgs: 25 messages, 1 services
-- +++ processing catkin package: 'autopilot_bridge'
-- ==> add_subdirectory(autopilot_bridge)
-- autopilot_bridge: 3 messages, 0 services
-- +++ processing catkin package: 'mavlink_ros'
-- ==> add_subdirectory(mavlink_ros)
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- WARNING: you are using the obsolete 'PKGCONFIG' macro, use FindPkgConfig
-- mavlink_ros: 1 messages, 0 services
-- +++ processing catkin package: 'mavros'
-- ==> add_subdirectory(mavros)
-- mavros: 7 messages, 15 services
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/ardupilotmega.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/autoquad.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/common.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/matrixpilot.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/minimal.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/pixhawk.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/slugs.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/test.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/ualberta.xml
-- processing: /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/sensesoar.xml
-- +++ processing catkin package: 'shape_msgs'
-- ==> add_subdirectory(common_msgs/shape_msgs)
-- shape_msgs: 4 messages, 0 services
-- +++ processing catkin package: 'stereo_msgs'
-- ==> add_subdirectory(common_msgs/stereo_msgs)
-- stereo_msgs: 1 messages, 0 services
-- +++ processing catkin package: 'trajectory_msgs'
-- ==> add_subdirectory(common_msgs/trajectory_msgs)
-- trajectory_msgs: 2 messages, 0 services
-- +++ processing catkin package: 'visualization_msgs'
-- ==> add_subdirectory(common_msgs/visualization_msgs)
-- visualization_msgs: 10 messages, 0 services
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws/build
make[2]: Warning: File `/root/catkin_ws/src/common_msgs/actionlib_msgs/msg/GoalID.msg' has modification time 4.2e+02 s in the future
[  0%] Generating C++ code from actionlib_msgs/GoalID.msg
[  0%] Generating C++ code from actionlib_msgs/GoalStatus.msg
[  0%] Generating C++ code from actionlib_msgs/GoalStatusArray.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[  0%] Built target actionlib_msgs_gencpp
make[2]: Warning: File `/root/catkin_ws/src/common_msgs/actionlib_msgs/msg/GoalID.msg' has modification time 4.2e+02 s in the future
[  1%] Generating Lisp code from actionlib_msgs/GoalID.msg
[  1%] Generating Lisp code from actionlib_msgs/GoalStatus.msg
[  1%] Generating Lisp code from actionlib_msgs/GoalStatusArray.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[  1%] Built target actionlib_msgs_genlisp
make[2]: Warning: File `/root/catkin_ws/src/common_msgs/actionlib_msgs/msg/GoalID.msg' has modification time 4.2e+02 s in the future
[  1%] Generating Python from MSG actionlib_msgs/GoalID
[  2%] Generating Python from MSG actionlib_msgs/GoalStatus
[  2%] Generating Python from MSG actionlib_msgs/GoalStatusArray
[  2%] Generating Python msg __init__.py for actionlib_msgs
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[  2%] Built target actionlib_msgs_genpy
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[  3%] Generating C++ code from diagnostic_msgs/DiagnosticArray.msg
[  3%] Generating C++ code from diagnostic_msgs/DiagnosticStatus.msg
[  3%] Generating C++ code from diagnostic_msgs/KeyValue.msg
[  4%] Generating C++ code from diagnostic_msgs/SelfTest.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[  4%] Built target diagnostic_msgs_gencpp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[  4%] Generating Lisp code from diagnostic_msgs/DiagnosticArray.msg
[  4%] Generating Lisp code from diagnostic_msgs/DiagnosticStatus.msg
[  4%] Generating Lisp code from diagnostic_msgs/KeyValue.msg
[  5%] Generating Lisp code from diagnostic_msgs/SelfTest.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[  5%] Built target diagnostic_msgs_genlisp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[  5%] Generating Python from MSG diagnostic_msgs/DiagnosticArray
[  5%] Generating Python from MSG diagnostic_msgs/DiagnosticStatus
[  5%] Generating Python from MSG diagnostic_msgs/KeyValue
[  6%] Generating Python code from SRV diagnostic_msgs/SelfTest
[  6%] Generating Python msg __init__.py for diagnostic_msgs
[  6%] Generating Python srv __init__.py for diagnostic_msgs
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[  6%] Built target diagnostic_msgs_genpy
make[2]: Warning: File `/root/catkin_ws/src/common_msgs/geometry_msgs/msg/Point.msg' has modification time 4e+02 s in the future
[  7%] Generating C++ code from geometry_msgs/Point.msg
[  7%] Generating C++ code from geometry_msgs/Point32.msg
[  7%] Generating C++ code from geometry_msgs/PointStamped.msg
[  7%] Generating C++ code from geometry_msgs/Polygon.msg
[  8%] Generating C++ code from geometry_msgs/PolygonStamped.msg
[  8%] Generating C++ code from geometry_msgs/Pose2D.msg
[  8%] Generating C++ code from geometry_msgs/Pose.msg
[  8%] Generating C++ code from geometry_msgs/PoseArray.msg
[  9%] Generating C++ code from geometry_msgs/PoseStamped.msg
[  9%] Generating C++ code from geometry_msgs/PoseWithCovariance.msg
[  9%] Generating C++ code from geometry_msgs/PoseWithCovarianceStamped.msg
[  9%] Generating C++ code from geometry_msgs/Quaternion.msg
[ 10%] Generating C++ code from geometry_msgs/QuaternionStamped.msg
[ 10%] Generating C++ code from geometry_msgs/Transform.msg
[ 10%] Generating C++ code from geometry_msgs/TransformStamped.msg
[ 11%] Generating C++ code from geometry_msgs/Twist.msg
[ 11%] Generating C++ code from geometry_msgs/TwistStamped.msg
[ 11%] Generating C++ code from geometry_msgs/TwistWithCovariance.msg
[ 11%] Generating C++ code from geometry_msgs/TwistWithCovarianceStamped.msg
[ 12%] Generating C++ code from geometry_msgs/Vector3.msg
[ 12%] Generating C++ code from geometry_msgs/Vector3Stamped.msg
[ 12%] Generating C++ code from geometry_msgs/Wrench.msg
[ 12%] Generating C++ code from geometry_msgs/WrenchStamped.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 12%] Built target geometry_msgs_gencpp
make[2]: Warning: File `/root/catkin_ws/src/common_msgs/geometry_msgs/msg/Point.msg' has modification time 3.5e+02 s in the future
[ 13%] Generating Lisp code from geometry_msgs/Point.msg
[ 13%] Generating Lisp code from geometry_msgs/Point32.msg
[ 13%] Generating Lisp code from geometry_msgs/PointStamped.msg
[ 13%] Generating Lisp code from geometry_msgs/Polygon.msg
[ 14%] Generating Lisp code from geometry_msgs/PolygonStamped.msg
[ 14%] Generating Lisp code from geometry_msgs/Pose2D.msg
[ 14%] Generating Lisp code from geometry_msgs/Pose.msg
[ 14%] Generating Lisp code from geometry_msgs/PoseArray.msg
[ 15%] Generating Lisp code from geometry_msgs/PoseStamped.msg
[ 15%] Generating Lisp code from geometry_msgs/PoseWithCovariance.msg
[ 15%] Generating Lisp code from geometry_msgs/PoseWithCovarianceStamped.msg
[ 15%] Generating Lisp code from geometry_msgs/Quaternion.msg
[ 16%] Generating Lisp code from geometry_msgs/QuaternionStamped.msg
[ 16%] Generating Lisp code from geometry_msgs/Transform.msg
[ 16%] Generating Lisp code from geometry_msgs/TransformStamped.msg
[ 17%] Generating Lisp code from geometry_msgs/Twist.msg
[ 17%] Generating Lisp code from geometry_msgs/TwistStamped.msg
[ 17%] Generating Lisp code from geometry_msgs/TwistWithCovariance.msg
[ 17%] Generating Lisp code from geometry_msgs/TwistWithCovarianceStamped.msg
[ 18%] Generating Lisp code from geometry_msgs/Vector3.msg
[ 18%] Generating Lisp code from geometry_msgs/Vector3Stamped.msg
[ 18%] Generating Lisp code from geometry_msgs/Wrench.msg
[ 18%] Generating Lisp code from geometry_msgs/WrenchStamped.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 18%] Built target geometry_msgs_genlisp
make[2]: Warning: File `/root/catkin_ws/src/common_msgs/geometry_msgs/msg/Point.msg' has modification time 3.3e+02 s in the future
[ 19%] Generating Python from MSG geometry_msgs/Point
[ 19%] Generating Python from MSG geometry_msgs/Point32
[ 19%] Generating Python from MSG geometry_msgs/PointStamped
[ 19%] Generating Python from MSG geometry_msgs/Polygon
[ 20%] Generating Python from MSG geometry_msgs/PolygonStamped
[ 20%] Generating Python from MSG geometry_msgs/Pose2D
[ 20%] Generating Python from MSG geometry_msgs/Pose
[ 20%] Generating Python from MSG geometry_msgs/PoseArray
[ 21%] Generating Python from MSG geometry_msgs/PoseStamped
[ 21%] Generating Python from MSG geometry_msgs/PoseWithCovariance
[ 21%] Generating Python from MSG geometry_msgs/PoseWithCovarianceStamped
[ 21%] Generating Python from MSG geometry_msgs/Quaternion
[ 22%] Generating Python from MSG geometry_msgs/QuaternionStamped
[ 22%] Generating Python from MSG geometry_msgs/Transform
[ 22%] Generating Python from MSG geometry_msgs/TransformStamped
[ 22%] Generating Python from MSG geometry_msgs/Twist
[ 23%] Generating Python from MSG geometry_msgs/TwistStamped
[ 23%] Generating Python from MSG geometry_msgs/TwistWithCovariance
[ 23%] Generating Python from MSG geometry_msgs/TwistWithCovarianceStamped
[ 24%] Generating Python from MSG geometry_msgs/Vector3
[ 24%] Generating Python from MSG geometry_msgs/Vector3Stamped
[ 24%] Generating Python from MSG geometry_msgs/Wrench
[ 24%] Generating Python from MSG geometry_msgs/WrenchStamped
[ 25%] Generating Python msg __init__.py for geometry_msgs
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 25%] Built target geometry_msgs_genpy
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 26%] Generating C++ code from nav_msgs/GridCells.msg
[ 26%] Generating C++ code from nav_msgs/MapMetaData.msg
[ 26%] Generating C++ code from nav_msgs/OccupancyGrid.msg
[ 26%] Generating C++ code from nav_msgs/Odometry.msg
[ 27%] Generating C++ code from nav_msgs/Path.msg
[ 27%] Generating C++ code from nav_msgs/GetMapAction.msg
[ 27%] Generating C++ code from nav_msgs/GetMapActionGoal.msg
[ 28%] Generating C++ code from nav_msgs/GetMapActionResult.msg
[ 28%] Generating C++ code from nav_msgs/GetMapActionFeedback.msg
[ 28%] Generating C++ code from nav_msgs/GetMapGoal.msg
[ 28%] Generating C++ code from nav_msgs/GetMapResult.msg
[ 29%] Generating C++ code from nav_msgs/GetMapFeedback.msg
[ 29%] Generating C++ code from nav_msgs/GetMap.srv
[ 29%] Generating C++ code from nav_msgs/GetPlan.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 29%] Built target nav_msgs_gencpp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 29%] Generating Lisp code from nav_msgs/GridCells.msg
[ 30%] Generating Lisp code from nav_msgs/MapMetaData.msg
[ 30%] Generating Lisp code from nav_msgs/OccupancyGrid.msg
[ 30%] Generating Lisp code from nav_msgs/Odometry.msg
[ 30%] Generating Lisp code from nav_msgs/Path.msg
[ 31%] Generating Lisp code from nav_msgs/GetMapAction.msg
[ 31%] Generating Lisp code from nav_msgs/GetMapActionGoal.msg
[ 31%] Generating Lisp code from nav_msgs/GetMapActionResult.msg
[ 31%] Generating Lisp code from nav_msgs/GetMapActionFeedback.msg
[ 32%] Generating Lisp code from nav_msgs/GetMapGoal.msg
[ 32%] Generating Lisp code from nav_msgs/GetMapResult.msg
[ 32%] Generating Lisp code from nav_msgs/GetMapFeedback.msg
[ 32%] Generating Lisp code from nav_msgs/GetMap.srv
[ 33%] Generating Lisp code from nav_msgs/GetPlan.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 33%] Built target nav_msgs_genlisp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 33%] Generating Python from MSG nav_msgs/GridCells
[ 33%] Generating Python from MSG nav_msgs/MapMetaData
[ 34%] Generating Python from MSG nav_msgs/OccupancyGrid
[ 34%] Generating Python from MSG nav_msgs/Odometry
[ 34%] Generating Python from MSG nav_msgs/Path
[ 34%] Generating Python from MSG nav_msgs/GetMapAction
[ 35%] Generating Python from MSG nav_msgs/GetMapActionGoal
[ 35%] Generating Python from MSG nav_msgs/GetMapActionResult
[ 35%] Generating Python from MSG nav_msgs/GetMapActionFeedback
[ 35%] Generating Python from MSG nav_msgs/GetMapGoal
[ 36%] Generating Python from MSG nav_msgs/GetMapResult
[ 36%] Generating Python from MSG nav_msgs/GetMapFeedback
[ 36%] Generating Python code from SRV nav_msgs/GetMap
[ 36%] Generating Python code from SRV nav_msgs/GetPlan
[ 37%] Generating Python msg __init__.py for nav_msgs
[ 37%] Generating Python srv __init__.py for nav_msgs
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 37%] Built target nav_msgs_genpy
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 37%] Generating C++ code from sensor_msgs/CameraInfo.msg
[ 37%] Generating C++ code from sensor_msgs/ChannelFloat32.msg
[ 37%] Generating C++ code from sensor_msgs/CompressedImage.msg
[ 38%] Generating C++ code from sensor_msgs/FluidPressure.msg
[ 38%] Generating C++ code from sensor_msgs/Illuminance.msg
[ 38%] Generating C++ code from sensor_msgs/Image.msg
[ 38%] Generating C++ code from sensor_msgs/Imu.msg
[ 39%] Generating C++ code from sensor_msgs/JointState.msg
[ 39%] Generating C++ code from sensor_msgs/Joy.msg
[ 39%] Generating C++ code from sensor_msgs/JoyFeedback.msg
[ 40%] Generating C++ code from sensor_msgs/JoyFeedbackArray.msg
[ 40%] Generating C++ code from sensor_msgs/LaserEcho.msg
[ 40%] Generating C++ code from sensor_msgs/LaserScan.msg
[ 40%] Generating C++ code from sensor_msgs/MagneticField.msg
[ 41%] Generating C++ code from sensor_msgs/MultiEchoLaserScan.msg
[ 41%] Generating C++ code from sensor_msgs/NavSatFix.msg
[ 41%] Generating C++ code from sensor_msgs/NavSatStatus.msg
[ 41%] Generating C++ code from sensor_msgs/PointCloud.msg
[ 42%] Generating C++ code from sensor_msgs/PointCloud2.msg
[ 42%] Generating C++ code from sensor_msgs/PointField.msg
[ 42%] Generating C++ code from sensor_msgs/Range.msg
[ 42%] Generating C++ code from sensor_msgs/RegionOfInterest.msg
[ 43%] Generating C++ code from sensor_msgs/RelativeHumidity.msg
[ 43%] Generating C++ code from sensor_msgs/Temperature.msg
[ 43%] Generating C++ code from sensor_msgs/TimeReference.msg
[ 43%] Generating C++ code from sensor_msgs/SetCameraInfo.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 43%] Built target sensor_msgs_gencpp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 44%] Generating Lisp code from sensor_msgs/CameraInfo.msg
[ 44%] Generating Lisp code from sensor_msgs/ChannelFloat32.msg
[ 44%] Generating Lisp code from sensor_msgs/CompressedImage.msg
[ 44%] Generating Lisp code from sensor_msgs/FluidPressure.msg
[ 45%] Generating Lisp code from sensor_msgs/Illuminance.msg
[ 45%] Generating Lisp code from sensor_msgs/Image.msg
[ 45%] Generating Lisp code from sensor_msgs/Imu.msg
[ 45%] Generating Lisp code from sensor_msgs/JointState.msg
[ 46%] Generating Lisp code from sensor_msgs/Joy.msg
[ 46%] Generating Lisp code from sensor_msgs/JoyFeedback.msg
[ 46%] Generating Lisp code from sensor_msgs/JoyFeedbackArray.msg
[ 47%] Generating Lisp code from sensor_msgs/LaserEcho.msg
[ 47%] Generating Lisp code from sensor_msgs/LaserScan.msg
[ 47%] Generating Lisp code from sensor_msgs/MagneticField.msg
[ 47%] Generating Lisp code from sensor_msgs/MultiEchoLaserScan.msg
[ 48%] Generating Lisp code from sensor_msgs/NavSatFix.msg
[ 48%] Generating Lisp code from sensor_msgs/NavSatStatus.msg
[ 48%] Generating Lisp code from sensor_msgs/PointCloud.msg
[ 48%] Generating Lisp code from sensor_msgs/PointCloud2.msg
[ 49%] Generating Lisp code from sensor_msgs/PointField.msg
[ 49%] Generating Lisp code from sensor_msgs/Range.msg
[ 49%] Generating Lisp code from sensor_msgs/RegionOfInterest.msg
[ 49%] Generating Lisp code from sensor_msgs/RelativeHumidity.msg
[ 50%] Generating Lisp code from sensor_msgs/Temperature.msg
[ 50%] Generating Lisp code from sensor_msgs/TimeReference.msg
[ 50%] Generating Lisp code from sensor_msgs/SetCameraInfo.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 50%] Built target sensor_msgs_genlisp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 50%] Generating Python from MSG sensor_msgs/CameraInfo
[ 51%] Generating Python from MSG sensor_msgs/ChannelFloat32
[ 51%] Generating Python from MSG sensor_msgs/CompressedImage
[ 51%] Generating Python from MSG sensor_msgs/FluidPressure
[ 51%] Generating Python from MSG sensor_msgs/Illuminance
[ 52%] Generating Python from MSG sensor_msgs/Image
[ 52%] Generating Python from MSG sensor_msgs/Imu
[ 52%] Generating Python from MSG sensor_msgs/JointState
[ 53%] Generating Python from MSG sensor_msgs/Joy
[ 53%] Generating Python from MSG sensor_msgs/JoyFeedback
[ 53%] Generating Python from MSG sensor_msgs/JoyFeedbackArray
[ 53%] Generating Python from MSG sensor_msgs/LaserEcho
[ 54%] Generating Python from MSG sensor_msgs/LaserScan
[ 54%] Generating Python from MSG sensor_msgs/MagneticField
[ 54%] Generating Python from MSG sensor_msgs/MultiEchoLaserScan
[ 54%] Generating Python from MSG sensor_msgs/NavSatFix
[ 55%] Generating Python from MSG sensor_msgs/NavSatStatus
[ 55%] Generating Python from MSG sensor_msgs/PointCloud
[ 55%] Generating Python from MSG sensor_msgs/PointCloud2
[ 55%] Generating Python from MSG sensor_msgs/PointField
[ 56%] Generating Python from MSG sensor_msgs/Range
[ 56%] Generating Python from MSG sensor_msgs/RegionOfInterest
[ 56%] Generating Python from MSG sensor_msgs/RelativeHumidity
[ 56%] Generating Python from MSG sensor_msgs/Temperature
[ 57%] Generating Python from MSG sensor_msgs/TimeReference
[ 57%] Generating Python code from SRV sensor_msgs/SetCameraInfo
[ 57%] Generating Python msg __init__.py for sensor_msgs
[ 57%] Generating Python srv __init__.py for sensor_msgs
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 57%] Built target sensor_msgs_genpy
make[2]: Warning: File `/opt/ros/groovy/lib/gencpp/gen_cpp.py' has modification time 4.1e+08 s in the future
[ 58%] Generating C++ code from autopilot_bridge/Heartbeat.msg
[ 58%] Generating C++ code from autopilot_bridge/LLA.msg
[ 58%] Generating C++ code from autopilot_bridge/Status.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 58%] Built target autopilot_bridge_gencpp
make[2]: Warning: File `/opt/ros/groovy/lib/genlisp/gen_lisp.py' has modification time 4.1e+08 s in the future
[ 58%] Generating Lisp code from autopilot_bridge/Heartbeat.msg
[ 59%] Generating Lisp code from autopilot_bridge/LLA.msg
[ 59%] Generating Lisp code from autopilot_bridge/Status.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 59%] Built target autopilot_bridge_genlisp
make[2]: Warning: File `/opt/ros/groovy/lib/genpy/genmsg_py.py' has modification time 4.1e+08 s in the future
[ 59%] Generating Python from MSG autopilot_bridge/Heartbeat
[ 59%] Generating Python from MSG autopilot_bridge/LLA
[ 60%] Generating Python from MSG autopilot_bridge/Status
[ 60%] Generating Python msg __init__.py for autopilot_bridge
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 60%] Built target autopilot_bridge_genpy
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 61%] Generating C++ code from mavlink_ros/Mavlink.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 61%] Built target mavlink_ros_gencpp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 61%] Generating Lisp code from mavlink_ros/Mavlink.msg
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 61%] Built target mavlink_ros_genlisp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 61%] Generating Python from MSG mavlink_ros/Mavlink
[ 61%] Generating Python msg __init__.py for mavlink_ros
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 61%] Built target mavlink_ros_genpy
Scanning dependencies of target mavlink_ros_serial
make[2]: Warning: File `/opt/ros/groovy/include/XmlRpcDecl.h' has modification time 4.2e+08 s in the future
[ 62%] Building CXX object mavlink_ros/CMakeFiles/mavlink_ros_serial.dir/src/mavlink_ros_serial.cpp.o
Linking CXX executable /root/catkin_ws/devel/lib/mavlink_ros/mavlink_ros_serial
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 62%] Built target mavlink_ros_serial
[ 62%] Built target ardupilotmega.xml-v1.0
[ 62%] Built target autoquad.xml-v1.0
[ 62%] Built target common.xml-v1.0
[ 62%] Built target matrixpilot.xml-v1.0
Scanning dependencies of target mavconn
make[2]: Warning: File `/opt/ros/groovy/include/ros/assert.h' has modification time 4.2e+08 s in the future
[ 63%] Building CXX object mavros/CMakeFiles/mavconn.dir/src/mavconn/mavconn_interface.cpp.o
[ 63%] Building CXX object mavros/CMakeFiles/mavconn.dir/src/mavconn/mavconn_serial.cpp.o
[ 63%] Building CXX object mavros/CMakeFiles/mavconn.dir/src/mavconn/mavconn_udp.cpp.o
In file included from /root/catkin_ws/src/mavros/src/mavconn/mavconn_udp.cpp:23:0:
/root/catkin_ws/src/mavros/src/mavconn/mavconn_udp.h:47:40: warning: ‘auto_ptr’ is deprecated (declared at /usr/include/c++/4.6/backward/auto_ptr.h:87) [-Wdeprecated-declarations]
Linking CXX shared library /root/catkin_ws/devel/lib/libmavconn.so
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 63%] Built target mavconn
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 63%] Generating C++ code from mavros/Mavlink.msg
[ 63%] Generating C++ code from mavros/Waypoint.msg
[ 63%] Generating C++ code from mavros/WaypointList.msg
[ 64%] Generating C++ code from mavros/BatteryStatus.msg
[ 64%] Generating C++ code from mavros/State.msg
[ 64%] Generating C++ code from mavros/RCIn.msg
[ 64%] Generating C++ code from mavros/RCOut.msg
[ 65%] Generating C++ code from mavros/ParamSet.srv
[ 65%] Generating C++ code from mavros/ParamGet.srv
[ 65%] Generating C++ code from mavros/ParamPull.srv
[ 66%] Generating C++ code from mavros/ParamPush.srv
[ 66%] Generating C++ code from mavros/WaypointSetCurrent.srv
[ 66%] Generating C++ code from mavros/WaypointClear.srv
[ 66%] Generating C++ code from mavros/WaypointPull.srv
[ 67%] Generating C++ code from mavros/WaypointPush.srv
[ 67%] Generating C++ code from mavros/WaypointGOTO.srv
[ 67%] Generating C++ code from mavros/OverrideRCIn.srv
[ 67%] Generating C++ code from mavros/CommandLong.srv
[ 68%] Generating C++ code from mavros/CommandBool.srv
[ 68%] Generating C++ code from mavros/CommandMode.srv
[ 68%] Generating C++ code from mavros/CommandHome.srv
[ 68%] Generating C++ code from mavros/StreamRate.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 68%] Built target mavros_gencpp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 69%] Generating Lisp code from mavros/Mavlink.msg
[ 69%] Generating Lisp code from mavros/Waypoint.msg
[ 69%] Generating Lisp code from mavros/WaypointList.msg
[ 69%] Generating Lisp code from mavros/BatteryStatus.msg
[ 70%] Generating Lisp code from mavros/State.msg
[ 70%] Generating Lisp code from mavros/RCIn.msg
[ 70%] Generating Lisp code from mavros/RCOut.msg
[ 70%] Generating Lisp code from mavros/ParamSet.srv
[ 71%] Generating Lisp code from mavros/ParamGet.srv
[ 71%] Generating Lisp code from mavros/ParamPull.srv
[ 71%] Generating Lisp code from mavros/ParamPush.srv
[ 72%] Generating Lisp code from mavros/WaypointSetCurrent.srv
[ 72%] Generating Lisp code from mavros/WaypointClear.srv
[ 72%] Generating Lisp code from mavros/WaypointPull.srv
[ 72%] Generating Lisp code from mavros/WaypointPush.srv
[ 73%] Generating Lisp code from mavros/WaypointGOTO.srv
[ 73%] Generating Lisp code from mavros/OverrideRCIn.srv
[ 73%] Generating Lisp code from mavros/CommandLong.srv
[ 73%] Generating Lisp code from mavros/CommandBool.srv
[ 74%] Generating Lisp code from mavros/CommandMode.srv
[ 74%] Generating Lisp code from mavros/CommandHome.srv
[ 74%] Generating Lisp code from mavros/StreamRate.srv
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 74%] Built target mavros_genlisp
make[2]: Warning: File `/opt/ros/groovy/share/std_msgs/msg/Header.msg' has modification time 4.1e+08 s in the future
[ 74%] Generating Python from MSG mavros/Mavlink
[ 75%] Generating Python from MSG mavros/Waypoint
[ 75%] Generating Python from MSG mavros/WaypointList
[ 75%] Generating Python from MSG mavros/BatteryStatus
[ 75%] Generating Python from MSG mavros/State
[ 76%] Generating Python from MSG mavros/RCIn
[ 76%] Generating Python from MSG mavros/RCOut
[ 76%] Generating Python code from SRV mavros/ParamSet
[ 76%] Generating Python code from SRV mavros/ParamGet
[ 77%] Generating Python code from SRV mavros/ParamPull
[ 77%] Generating Python code from SRV mavros/ParamPush
[ 77%] Generating Python code from SRV mavros/WaypointSetCurrent
[ 77%] Generating Python code from SRV mavros/WaypointClear
[ 78%] Generating Python code from SRV mavros/WaypointPull
[ 78%] Generating Python code from SRV mavros/WaypointPush
[ 78%] Generating Python code from SRV mavros/WaypointGOTO
[ 79%] Generating Python code from SRV mavros/OverrideRCIn
[ 79%] Generating Python code from SRV mavros/CommandLong
[ 79%] Generating Python code from SRV mavros/CommandBool
[ 79%] Generating Python code from SRV mavros/CommandMode
[ 80%] Generating Python code from SRV mavros/CommandHome
[ 80%] Generating Python code from SRV mavros/StreamRate
[ 80%] Generating Python msg __init__.py for mavros
[ 80%] Generating Python srv __init__.py for mavros
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 80%] Built target mavros_genpy
make[2]: Warning: File `/opt/ros/groovy/include/XmlRpcDecl.h' has modification time 4.2e+08 s in the future
[ 81%] Building CXX object mavros/CMakeFiles/mavros_node.dir/src/mavros_node.cpp.o
In file included from /root/catkin_ws/src/mavros/src/mavros_node.cpp:30:0:
/root/catkin_ws/src/mavros/src/mavconn/mavconn_udp.h:47:40: warning: ‘auto_ptr’ is deprecated (declared at /usr/include/c++/4.6/backward/auto_ptr.h:87) [-Wdeprecated-declarations]
[ 81%] Building CXX object mavros/CMakeFiles/mavros_node.dir/src/uas.cpp.o
Linking CXX executable /root/catkin_ws/devel/lib/mavros/mavros_node
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 81%] Built target mavros_node
Scanning dependencies of target mavros_plugins
make[2]: Warning: File `/opt/ros/groovy/include/XmlRpcDecl.h' has modification time 4.2e+08 s in the future
[ 81%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/dummy.cpp.o
[ 81%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/sys_status.cpp.o
[ 82%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/imu_pub.cpp.o
[ 82%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/gps.cpp.o
[ 82%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/param.cpp.o
[ 82%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/waypoint.cpp.o
[ 83%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/rc_io.cpp.o
[ 83%] Building CXX object mavros/CMakeFiles/mavros_plugins.dir/src/plugins/command.cpp.o
Linking CXX shared library /root/catkin_ws/devel/lib/libmavros_plugins.so
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 83%] Built target mavros_plugins
Scanning dependencies of target minimal.xml-v1.0
[ 83%] Generating minimal.xml-v1.0-stamp
Validating /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/minimal.xml
Parsing /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/minimal.xml
Found 1 MAVLink message types in 1 XML files
Generating C implementation in directory /root/catkin_ws/devel/include/mavlink/v1.0/minimal
Copying fixed headers
[ 83%] Built target minimal.xml-v1.0
Scanning dependencies of target pixhawk.xml-v1.0
[ 83%] Generating pixhawk.xml-v1.0-stamp
Validating /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/pixhawk.xml
Parsing /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/pixhawk.xml
Note: message IMAGE_AVAILABLE is longer than 64 bytes long (100 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message WATCHDOG_PROCESS_INFO is longer than 64 bytes long (263 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message PATTERN_DETECTED is longer than 64 bytes long (114 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Validating /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/common.xml
Parsing /root/catkin_ws/src/mavros/mavlink/message_definitions/v1.0/common.xml
Note: message GPS_STATUS is longer than 64 bytes long (109 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message HIGHRES_IMU is longer than 64 bytes long (70 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message HIL_SENSOR is longer than 64 bytes long (72 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message SIM_STATE is longer than 64 bytes long (92 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message FILE_TRANSFER_START is longer than 64 bytes long (262 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message FILE_TRANSFER_DIR_LIST is longer than 64 bytes long (257 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message HIL_STATE_QUATERNION is longer than 64 bytes long (72 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message LOG_DATA is longer than 64 bytes long (105 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message GPS_INJECT_DATA is longer than 64 bytes long (121 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message SERIAL_CONTROL is longer than 64 bytes long (87 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Note: message ENCAPSULATED_DATA is longer than 64 bytes long (263 bytes), which can cause fragmentation since many radio modems use 64 bytes as maximum air transfer unit.
Merged enum MAV_CMD
Found 123 MAVLink message types in 2 XML files
Generating C implementation in directory /root/catkin_ws/devel/include/mavlink/v1.0/pixhawk
Generating C implementation in directory /root/catkin_ws/devel/include/mavlink/v1.0/common
Copying fixed headers
Traceback (most recent call last):
  File "/root/catkin_ws/src/mavros/mavlink/pymavlink/generator/mavgen.py", line 166, in <module>
    mavgen(opts, args)
  File "/root/catkin_ws/src/mavros/mavlink/pymavlink/generator/mavgen.py", line 93, in mavgen
    mavgen_c.generate(opts.output, xml)
  File "/usr/local/lib/python2.7/dist-packages/pymavlink/generator/mavgen_c.py", line 650, in generate
    copy_fixed_headers(basename, xml_list[0])
  File "/usr/local/lib/python2.7/dist-packages/pymavlink/generator/mavgen_c.py", line 493, in copy_fixed_headers
    shutil.copy(src, dest)
  File "/usr/lib/python2.7/shutil.py", line 117, in copy
    copyfile(src, dst)
  File "/usr/lib/python2.7/shutil.py", line 82, in copyfile
    with open(src, 'rb') as fsrc:
IOError: [Errno 2] No such file or directory: '/usr/local/lib/python2.7/dist-packages/pymavlink/generator/C/include_v1.0/pixhawk/pixhawk.pb.h'
make[2]: *** [mavros/pixhawk.xml-v1.0-stamp] Error 1
make[1]: *** [mavros/CMakeFiles/pixhawk.xml-v1.0.dir/all] Error 2
make: *** [all] Error 2
Invoking "make" failed
```
