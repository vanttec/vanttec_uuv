#!/bin/bash

# Settings used for accessing X11 server and thus getting graphics within the container
#DOCKER_GRAPHICSx_ARGS="-e DISPLAY=host.docker.internal:0 -e QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --device=/dev/dri:/dev/dri"

xhost +

docker run -it -d\
    -e DISPLAY=host.docker.internal:0 \
    --name uuv \
    -v "$PWD:/home/uuv/vanttec_uuv/src" \
    -v "$(realpath ../vanttec_sim):/home/uuv/vanttec_sim/src" \
    uuv \
    /bin/bash