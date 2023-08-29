#!/usr/bin/env bash

IMAGE_NAME=mhseals/roboboat
IMAGE_TAG=linux-intel
CONTAINER_NAME=roboboat-container
CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# get image from Docker Hub
LOCAL_IMAGE=$(docker images -q "${IMAGE_NAME}:${IMAGE_TAG}")

if [[ -z "$LOCAL_IMAGE" ]]; then
    # Image is not available locally. Pull it.
    docker pull "${IMAGE_NAME}:${IMAGE_TAG}"
else
    # Image is available locally. Check if it's up-to-date.
    LATEST_IMAGE_ID=$(docker pull "${IMAGE_NAME}:${IMAGE_TAG}" | grep -o 'Digest:.*' | sed 's/Digest: sha256://g' | cut -d' ' -f1)
    LOCAL_IMAGE_ID=$(docker images --digests | grep "${IMAGE_NAME}:${IMAGE_TAG}" | awk '{print $3}')

    if [[ "$LATEST_IMAGE_ID" != "$LOCAL_IMAGE_ID" ]]; then
        echo "The image was outdated and has been updated."
    fi
fi

# if container does not exist, create it
if [ -z "${CONTAINER_ID}" ]; then

docker run \
--tty \
--detach \
--network host \
--name ${CONTAINER_NAME} \
--privileged \
--volume /dev:/dev \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--volume ${SCRIPT_DIR}/mhs_roboboat:/root/roboboat_ws/src/mhs_roboboat \
--env DISPLAY=$DISPLAY \
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
