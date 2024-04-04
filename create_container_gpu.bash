#!/bin/bash

# Settings used for getting graphics within the container
#DOCKER_GRAPHIC_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --device=/dev/dri:/dev/dri"

# Settings used for gpu
#DOCKER_GPU_ARGS="--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all"

#xhost +

docker run -it -d\
    --name uuv \
    uuv \
    /bin/bash