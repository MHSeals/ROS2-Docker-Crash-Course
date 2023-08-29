#!/usr/bin/env bash

IMAGE_NAME=mhseals/roboboat
IMAGE_TAG=windows-intel
CONTAINER_NAME=roboboat-container
CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# get image from Docker Hub
docker pull ${IMAGE_NAME}:${IMAGE_TAG}

# if container does not exist, create it
if [ -z "${CONTAINER_ID}" ]; then

docker run \
--tty \
--detach \
--name ${CONTAINER_NAME} \
--privileged \
--runtime nvidia \
--env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--volume ${SCRIPT_DIR}/mhs_roboboat:/root/roboboat_ws/src/mhs_roboboat \
--device /dev/dri/card1 \
--device /dev/dri/renderD129 \
${IMAGE_NAME}:${IMAGE_TAG}

else

# allow UI spawning through X server
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`

# if container exists but is not running, start it
if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
    docker start ${CONTAINER_ID}
fi

# enter container
docker exec -it ${CONTAINER_ID} bash

# disallow UI spawning through X server
xhost -local:`docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID}`

fi
