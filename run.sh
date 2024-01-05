#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
IMAGE_NAME=mqt0029/roboboat
IMAGE_TAG=sitl
CONTAINER_NAME=roboboat-sitl-container
CONTAINER_ID=$(docker ps -aqf "name=$CONTAINER_NAME")

if [ -z "${CONTAINER_ID}" ]; then

    docker run \
    --tty \
    --detach \
    --name ${CONTAINER_NAME} \
    --runtime nvidia \
    --gpus all \
    --interactive \
    --privileged \
    --network host \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ${SCRIPT_DIR}/mhsboat:/root/ros_ws/src/mhsboat \
    ${IMAGE_NAME}:${IMAGE_TAG}

else
    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`

    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        docker start ${CONTAINER_ID}
    fi

    docker exec -it ${CONTAINER_ID} bash

    xhost -local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`
fi
