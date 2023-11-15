#!/bin/bash

#  Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

export PROJ_PATH=/opt/robot/jacinto-picknplace-demo


# 1. Clone edgeai-gst-apps-pick-n-place and untar AprilTag detection artifacts
cd /opt
if [[ ! -d "edgeai-gst-apps-pick-n-place" ]]; then
    git clone --single-branch --branch niryo_picknplace_8.6 https://github.com/TexasInstruments/edgeai-gst-apps-pick-n-place.git

    mkdir /opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-apriltag-416x416
    cd edgeai-gst-apps-pick-n-place/models
    tar -xf ONR-OD-8200-yolox-nano-lite-mmdet-apriltag-416x416.tar.gz -C /opt/model_zoo/ONR-OD-8200-yolox-nano-lite-mmdet-apriltag-416x416
fi

# 2. Clone the apriltag_ros repo
cd  $PROJ_PATH/nodes
if [[ ! -d "apriltag_ros" ]]; then
    git clone --single-branch --branch master https://github.com/AprilRobotics/apriltag_ros.git
    cd  apriltag_ros
    git checkout 0da016c9ebe5da2ddd2229da691235d52e777dcd -b jacinto_picknplace_demo
    git apply $PROJ_PATH/patches/apriltag_ros_picknplace.patch
    git add --all
    git commit -m 'Jacinto pick_n_place demo patch'
fi

# 3. Clone the ned_ros repo
cd  $PROJ_PATH/nodes
if [[ ! -d "ned_ros" ]]; then
    git clone --single-branch --branch master https://github.com/NiryoRobotics/ned_ros.git
    cd  ned_ros
    git checkout fa4b0d92b44b2a68dd857e391c088143a6dcd716 -b jacinto_picknplace_demo
fi

# 4. Clone gscam
cd   $PROJ_PATH/nodes/pick_n_place
if [[ ! -d "gscam" ]]; then
    git clone --single-branch --branch master https://github.com/ros-drivers/gscam.git
    cd gscam
    git checkout -b jacinto_picknplace_demo
    git apply $PROJ_PATH/patches/gscam_ti.patch
    git add .   
    git commit -m "Customized for TI Jacinto Robotics SDK: Added pipleline that uses TI GStreamer plugins, and added NV12 encoding mode."
fi

# 5. Build docker
echo "Building docker image ..."
bash $PROJ_PATH/docker/docker_build_picknplace.sh

ln -s $PROJ_PATH/docker/docker_run_picknplace.sh /home/root/j7ros_home/docker_run_picknplace.sh
