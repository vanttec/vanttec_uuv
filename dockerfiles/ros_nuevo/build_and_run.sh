# RUN with ./build_and_run.sh

#!/bin/bash

# names
IMAGE_NAME="ros_boost_image"
CONTAINER_NAME="ros_boost_container"
DOCKERFILE_DIR="dockerfiles/ros_nuevo/dockerfile"   
WORKDIR="/root"      # path inside container

# image config
echo "[+] Building Docker image: $IMAGE_NAME"
docker build -t $IMAGE_NAME $DOCKERFILE_DIR

# check image
if [ $? -ne 0 ]; then
    echo "[-] Docker build failed. Exiting."
    exit 1
fi

# Run ZED 
echo "[+] Starting ZED Explorer on host..."
/usr/local/zed/tools/ZED_Explorer &

# container config
echo "[+] Running Docker container: $CONTAINER_NAME"
docker run -it --name $CONTAINER_NAME --rm $IMAGE_NAME bash

# After container exits, kill ZED
echo "[*] Cleaning up ZED Explorer"
pkill -f ZED_Explorer



