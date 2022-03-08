# About Artmis Drone #

# Requirements #
This package is compatible with ROS Noetic version (Ubuntu 20.04). it is suggested that do not use the full installation but the desktop installation.
```
$ sudo apt-get install ros-noetic-desktop
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


# Read sensor data from ROS topics #
One can use [rqt_gui](http://wiki.ros.org/rqt_gui) to have an extensive amount of utilities for topic visualization and manipulation. Some of the useful topics reside below.
```
forward looking camera :  /drone/front_camera/image_raw
downward looking camera: /drone/down_camera/image_raw
sonar data:  /drone/sonar
laser range data: /drone/laser
```
