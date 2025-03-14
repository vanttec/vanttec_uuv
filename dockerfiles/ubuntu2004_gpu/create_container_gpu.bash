#!/bin/bash

# Settings used for accessing X11 server and thus getting graphics within the container
DOCKER_GRAPHICS_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --device=/dev/dri:/dev/dri"

xhost +

docker run -it -d\
    $DOCKER_GRAPHICS_ARGS \
    --name uuv_gpu \
    --gpus all \
    --privileged \
    -v "${PWD%/*/*}:/ws/vanttec_uuv/src" \
    -v "/dev/bus/usb/:/dev/bus/usb" \
    uuv_gpu \
    /bin/bash
