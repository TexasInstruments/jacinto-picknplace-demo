#!/bin/bash
ROS_DISTRO=noetic
SDK_VER=8.6.1
DOCKER_TAG=j7-ros-$ROS_DISTRO-$SOC-picknplace:$SDK_VER
USE_PROXY=0
DOCKER_DIR=/opt/robot/jacinto-picknplace-demo/docker
IP_ADDR=$(ifconfig | grep -A 1 'eth0' | tail -1 | awk '{print $2}')
if [[ ! $IP_ADDR =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    IP_ADDR=$(ifconfig | grep -A 1 'wlp1s0' | tail -1 | awk '{print $2}')
fi
if [ "$#" -lt 1 ]; then
    CMD=/bin/bash
else
    CMD="$@"
fi
docker run -it --rm \
    -v /home/root/j7ros_home:/root/j7ros_home \
    -v /:/host:ro \
    -v /opt/robotics_sdk:/opt/robotics_sdk \
    -v /opt/robot:/opt/robot \
    -v /home/root/j7ros_home/.ros:/root/.ros \
    -v /opt/edgeai-gst-apps:/opt/edgeai-gst-apps \
    -v /opt/imaging:/opt/imaging \
    -v /opt/model_zoo:/opt/model_zoo \
    -v /usr/include/dlpack:/usr/include/dlpack \
    -v /dev:/dev \
    --privileged \
    --network host \
    --env USE_PROXY=$USE_PROXY \
    --env TIVA_LIB_VER=8.6.0 \
    --env J7_IP_ADDR=$IP_ADDR \
    --env-file $DOCKER_DIR/env_list.txt \
    --device-cgroup-rule='c 235:* rmw' \
      $DOCKER_TAG $CMD
