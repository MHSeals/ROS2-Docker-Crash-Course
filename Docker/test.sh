#!/usr/bin/env bash

xhost +

docker run \
    --interactive \
    --tty \
    --rm \
    --gpus all \
    --runtime nvidia \
    --network host \
    --privileged \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --env DISPLAY=$DISPLAY \
    mhseals/roboboat:linux-nvidia \
    glxinfo | grep "OpenGL"

# --device /dev/dri/card0 \
# --device /dev/dri/renderD128 \

# --device /dev/dxg \
# --device /dev/dri/card0 \
# --device /dev/dri/renderD128 \
# --volume /mnt/wslg:/mnt/wslg \
# --volume /usr/lib/wsl:/usr/lib/wsl \
# --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
# --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
# --env PULSE_SERVER=$PULSE_SERVER \

xhost -
