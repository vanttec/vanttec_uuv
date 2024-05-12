#!/bin/bash

# Settings used for accessing X11 server and thus getting graphics within the container
DOCKER_GRAPHICS_ARGS="--env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --device=/dev/dri:/dev/dri"

xhost +

docker run -it -d\
    $DOCKER_GRAPHICS_ARGS \
    --name uuv \
    --gpus all \
    -v "$PWD:/ws/vanttec_uuv/src" \
    --network host \
    uuv \
    /bin/bash