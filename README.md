
[MYNT EYE SDK]: https://github.com/slightech/MYNT-EYE-SDK.git

# MYNT EYE Camera - ROS Integration

This package lets you use the MYNT EYE stereo camera with ROS. It outputs the camera left and right images, depth map, imu.

## Getting started

* Download the latest version of the [MYNT EYE SDK][].
* Download the MYNT EYE ROS Wrapper.

```
$ git clone https://github.com/slightech/MYNT-EYE-ROS-Wrapper.git
```

### Prerequisites

* Ubuntu 16.04
* ROS Kinetic
* [MYNT EYE SDK][] and its dependencies

## Build the program

The MYNT-EYE-ROS-Wrapper is a catkin package. It depends on the following ROS packages:

* tf2_ros
* nav_msgs
* roscpp
* rosconsole
* sensor_msgs
* opencv
* image_transport
* dynamic_reconfigure
* urdf

Place the package folder MYNT-EYE-ROS-Wrapper in your catkin workspace source folder `~/catkin_ws/src`.

Open a terminal and build the package:

```
$ cd ~/catkin_ws/
$ catkin_make
$ source ./devel/setup.bash
```

## Run the program

Open a terminal and launch the wrapper:

```
$ roslaunch MYNT-EYE-ROS-Wrapper mynt_camera.launch
```
