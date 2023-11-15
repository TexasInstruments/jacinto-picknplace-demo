# Robotics Arm Pick-n-Place Demo on TI Platform

This repository maintains a demo targeted for the Niryo Ned2 Robot Arm with TI Jacinto TDA4VM and AM6xA platforms. This document walks through how to run the applications provided in this repository.


## Package Components

This repository contains the following:

1. The `nodes` sub-directory contains the Python modules that implement the robotics arm pick-n-place demo.
2. The `docker` sub-directory contains docker file to build docker container
3. The `patches` sub-directory contains GIT patches for [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) and [gscam](https://github.com/ros-drivers/gscam) repositories needed to run the robot arm following demo. 
4. The `scripts` sub-directory includes scripts needed to set up the demo, e.g. applying the GIT patches to the corresponding repos and build the demo.


## Environnment Setup 

### Hardware Components 

The following hardware components are necessary for robot arm demos:

1. A [Niryo Ned2](https://niryo.com/) robot arm. It would be possible to use Niryo Simulator instead. But this document does not explain how to run the demo using the Niryo simulator. 
2. A working [TDA4VM SK board](https://www.ti.com/tool/SK-TDA4VM), E2 revision or higher and the SD card flashed with the latest stable EdgeAI SDK with Robotics SDK.
4. A USB camera connected to the SK board.
5. Niryo and SK board should be connected to the same network through either Ethernet router or switch.

### Setting Up J7 SK Board

Download the prebuilt SD card image and flash it to a SD card by following the [Edge AI Documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/08_06_01/exports/docs/devices/TDA4VM/linux/getting_started.html).

Follow the instructions in the Robotics SDK User Guide Documentation on  [Setting up Robotics SDK](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_06_01/TDA4VM/docs/source/docker/README.html) and [Docker Setup for ROS1](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_06_01/TDA4VM/docs/source/docker/setting_docker_ros1.html) to install Robotics SDK and work with ROS1 on J7 SK board.


1. Once logged on to the SK board, clone the https://github.com/TexasInstruments/jacinto-picknplace-demo repository under the /opt directory.

```
root@tda4vm-sk:/opt/edgeai-gst-apps# cd ..
root@tda4vm-sk:/opt# mkdir robot && cd robot
root@tda4vm-sk:/opt/robot# git clone --single-branch --branch apriltag_picknplace https://github.com/TexasInstruments/jacinto-picknplace-demo.git
```


2. Run `setup_demo.sh`.

```
root@tda4vm-sk:/opt/robot# cd jacinto-picknplace-demo
root@tda4vm-sk:/opt/robot# source scripts/setup_demo.sh
```

This script performs
- Clone the apriltag_ros GIT repo and apply a patch
- Clone the Niryo ned_ros GIt repo
- Clone the gscam GIT repo and apply a patch
- Build ROS1 Noetic docker container to install apriltag lib and necessary packages to build ned_ros

If the above commands are successful, the directory structure should look as follows:

```
/opt/robot# 
+ jacinto-picknplace-demo
    + docker
        + docker_build_picknplace.sh
        + Dockerfile.arm64v8.noetic
        + docker_run_picknplace.sh
        + env_list.txt
    + LICENSE
    + nodes
       + pick_n_place
    + patches
       + apriltag_ros_picknplace.patch
       + gscam_ti.patch
    + README.md
    + scripts
       + build_demo.sh
       + setup_demo.sh
       + setup_ros_ip.sh
```

And you can check if docker images are built successfully. An example output based on 8.6.1 release is shown below.

```
root@tda4vm-sk:/opt/robot# docker images

REPOSITORY                                             TAG                       IMAGE ID            CREATED             SIZE
j7-ros-noetic-j721e-picknplace                         8.6.1                     c44314300e33        2 hours ago         3.67GB
j7-ros-noetic-j721e                                    8.6.1                     3e2a00526333        4 weeks ago         3.46GB
j7-ros-noetic-common                                   8.6.1                     0a4df850348a        4 weeks ago         2.94GB
artifactory.itg.ti.com/docker-public-arm/arm64v8/ros   noetic-perception-focal   2a3b0bdf0c4b        6 months ago        2.62GB
```

## Building Demo on TDA4VM or AM6xA

1. Run docker container 

```
root@tda4vm-sk:/opt/robot# cd $HOME/j7ros_home
root@tda4vm-sk:~/j7ros_home# ./docker_run_picknplace.sh
```

2. Build demo

```
root@j7-docker:~/j7ros_home/ros_ws$ /opt/robot/jacinto-picknplace-demo/scripts/build_demo.sh
```

This script performs
- Build the apriltag_ros GIT repo
- Build the Niryo ned_ros GIt repo
- Build the gscam GIT repo and pick_n_place_niryo

## Running Demo

### Niryo Robot

1. Log on to the Niryo robot from PC. Password is 'robotics'

```
user@pc:~$ ssh niryo@ned2_ip_address
```

2. Kill the ROS master.

```
niryo@ned2 ~ $ killall -9 rosmaster
```

3. Set `ROS_IP` and `ROS_MASTER_URI`, which are the same as the Niryo IP address. And launch niroy robot.

```
niryo@ned2 ~ $ cd catkin_ws
niryo@ned2 ~/catkin_ws $ soruce devel/setup.bash
niryo@ned2 ~/catkin_ws $ export ned2_ip_address=w.x.y.z
niryo@ned2 ~/catkin_ws $ export ROS_IP=$ned2_ip_address
niryo@ned2 ~/catkin_ws $ export ROS_MASTER_URI=http://$ned2_ip_address:11311
niryo@ned2 ~/catkin_ws $ roslaunch niryo_robot_bringup niryo_ned2_robot.launch
```

### TDA4VM and AM6xA

1. Run docker and launch gscam

```
root@j7-docker:~/j7ros_home/ros_ws$ source /opt/robot/jacinto-picknplace-demo/scripts/setup_ros_ip.sh
root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws$ roslaunch pick_n_place_niryo v4l_niryo_mjpg.launch
```

2. On another terminal, run docker and launch apriltag_ros

```
root@j7-docker:~/j7ros_home/ros_ws$ source /opt/robot/jacinto-picknplace-demo/scripts/setup_ros_ip.sh
root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws$ roslaunch apriltag_ros continuous_detection.launch
```

3. On another terminal, run docker and run pick_n_place.py

```
root@j7-docker:~/j7ros_home/ros_ws$ source /opt/robot/jacinto-picknplace-demo/scripts/setup_ros_ip.sh
root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws$ rosrun pick_n_place_niryo pick_n_place.py
```

