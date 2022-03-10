# About Project Artmis Drone #

# Requirements #
This package is compatible with ROS Noetic version (Ubuntu 20.04). it is suggested that do not use the full installation but the desktop installation.
```
$ sudo apt-get install ros-noetic-desktop
```
Build opencv for ROS noetic :
```
$ sudo apt-get update -y
$ sudo apt-get install -y libopencv-dev
```

Install cv-bridge : 
```
$ sudo apt-get install ros-$ROS_DISTRO-cv-bridge
```

Install Image Transport : 
```
$ sudo apt-get update -y
$ sudo apt-get install -y libimage-transport-dev
```

Additions to CMakeLists : 


In REQUIRED COMPONENTS : 
```
find_package(catkin REQUIRED COMPONENTS
 rospy
 roscpp
 sensor_msgs
 std_msgs
 cv_bridge
 image_transport
)
```
Include Directories : 
```
include_directories(include ${catkin_INCLUDE_DIRS})
FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
```
Building Opencv codes : 
```
add_executable(<build file> src/<filename>.cpp)
target_link_libraries(<build file> ${OpenCV_LIBS} ${catkin_LIBRARIES})
```
# Download and Compiling #
```
$ cd <catkin_ws>
$ catkin build
```

Here <catkin_ws> is the path of the catkin workspace. Please refer to the [tutorial](http://wiki.ros.org/ROS/Tutorials) about how to create a catkin workspace in ROS.

# Run
The simplest way is calling after you have built the workspace successfully.

```
$ cd <where you check out the code>
$ source devel/setup.bash
$ roslaunch urc_drone simple.launch
```
# Running with keyboard
In second terminal:

```
$ rosrun urc_drone drone_keyboard
```

# Running Aruco Localization
In a third terminal:

```
$ roslaunch arucoDetection detection_drone.launch
```


# Read sensor data from ROS topics #
One can use [rqt_gui](http://wiki.ros.org/rqt_gui) to have an extensive amount of utilities for topic visualization and manipulation. Some of the useful topics reside below.
```
forward looking camera :  /drone/camera_top/image_raw
downward looking camera: /drone/camera_base/image_raw
GPS data:  /fix
IMU data: /imu
```
