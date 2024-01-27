#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
IMAGE_NAME=mqt0029/roboboat
IMAGE_TAG=sitl_wsl2
CONTAINER_NAME=roboboat-WLS2-sitl-container
CONTAINER_ID=$(docker ps -aqf "name=$CONTAINER_NAME")

if [ -z "${CONTAINER_ID}" ]; then

    docker run \
    --tty \
    --detach \
    --name ${CONTAINER_NAME} \
    --interactive \
    --privileged \
    --device /dev/dxg \
    --device /dev/dri/card0 \
    --device /dev/dri/renderD128 \
    --env DISPLAY=$DISPLAY \
    --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env PULSE_SERVER=$PULSE_SERVER \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume /mnt/wslg:/mnt/wslg \
    --volume /usr/lib/wsl:/usr/lib/wsl \
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
