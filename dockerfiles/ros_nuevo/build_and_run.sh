# /bin/bash
#! RUN with ./build_and_run.sh

# Constants
IMAGE_NAME="ros_boost_image"
CONTAINER_NAME="ros_boost_container"
DOCKERFILE_DIR="$(pwd)"   
WORKDIR="/root"                     
HOST_SRC_PATH="$(cd ../.. && pwd)/src"


# Before running, check paths plsplspls
#echo "[DEBUG] Current directory: $(pwd)"
#echo "[DEBUG] DOCKERFILE_DIR: $DOCKERFILE_DIR"
#echo "[DEBUG] HOST_SRC_PATH: $HOST_SRC_PATH"

# Image
echo "[+] Building Docker image: $IMAGE_NAME"
docker build -t $IMAGE_NAME "$DOCKERFILE_DIR"

# Build check
if [ $? -ne 0 ]; then
    echo "[-] Docker build failed. Exiting."
    exit 1
fi

# TODO: check that this starts ZED
#echo "[+] Starting ZED Explorer on host..."
#/usr/local/zed/tools/ZED_Explorer &

# Container
echo "[+] Running Docker container: $CONTAINER_NAME"
docker run -it \
    --name $CONTAINER_NAME \
    --rm \
    --privileged \
    -v /dev:/dev \
    -v "$HOST_SRC_PATH:/ws/src" \
    $IMAGE_NAME bash

# TODO: check that this ends ZED process
#echo "[*] Cleaning up ZED Explorer"
#pkill -f ZED_Explorer