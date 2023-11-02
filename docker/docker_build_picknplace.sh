#!/bin/bash
DOCKER_TAG=j7-ros-noetic-${SOC}-picknplace:8.6.1
DOCKER_DIR=/opt/robot/jacinto-picknplace-demo/docker
USE_PROXY=1
# modify the server and proxy URLs as requied
if [ "${USE_PROXY}" -ne "0" ]; then
    REPO_LOCATION=artifactory.itg.ti.com/docker-public-arm/
    HTTP_PROXY=http://webproxy.ext.ti.com:80
else
    REPO_LOCATION=
fi
echo "USE_PROXY = $USE_PROXY"
echo "REPO_LOCATION = $REPO_LOCATION"


if [ "$(docker images -q $DOCKER_TAG 2> /dev/null)" == "" ]; then
    docker build \
        -t $DOCKER_TAG \
        --build-arg USE_PROXY=$USE_PROXY \
        --build-arg REPO_LOCATION=$REPO_LOCATION \
        --build-arg HTTP_PROXY=$HTTP_PROXY \
        --build-arg SOC=$SOC \
        -f $DOCKER_DIR/Dockerfile.arm64v8.noetic .
    echo "Docker build -t $DOCKER_TAG completed!"
else
    echo "$DOCKER_TAG already exists."
fi

