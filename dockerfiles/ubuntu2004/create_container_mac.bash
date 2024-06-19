#!/bin/bash

xhost +

docker run -it -d\
    -e DISPLAY=host.docker.internal:0 \
    --name uuv \
    -v "${PWD%/* / *}:/ws/vanttec_uuv/src" \
    -v "$(realpath ../vanttec_sim):/home/uuv/vanttec_sim/src" \
    uuv \
    /bin/bash