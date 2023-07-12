#!/usr/bin/env bash

TEST_OS=windows
TEST_GPU=intel

docker run \
    --tty \
    --rm \
    --privileged \
    --gpus all \
    --shm-size 16G \
    --device /dev/dxg \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e PULSE_SERVER=$PULSE_SERVER \
    -v /usr/lib/wsl:/usr/lib/wsl \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    roboboat:${TEST_OS}-${TEST_GPU} \
    glxinfo | grep "OpenGL"