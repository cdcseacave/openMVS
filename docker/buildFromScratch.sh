#!/bin/bash

# Example use:
# ./buildFromScratch.sh --cuda --master --workspace /home/username/datapath/

WORKSPACE=$(pwd)


while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --cuda)
            CUDA_BUILD_ARGS="--build-arg CUDA=1 --build-arg BASE_IMAGE=nvidia/cuda:12.9.1-devel-ubuntu24.04"

            CUDA_RUNTIME_ARGS="--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics"

            CUDA_CONTAINER_SUFFIX="-cuda"
            shift
            ;;
        --master)
            MASTER_ARGS="--build-arg MASTER=1"
            shift
            ;;
        --workspace)
            WORKSPACE=$2
            shift
            shift
            ;;
        *)
            echo "Unknown argument: $key"
            exit 1
            ;;
    esac
done

# no need to do `xhost +` anymore
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
DISPLAY_ARGS="--volume=$XSOCK:$XSOCK:rw --volume=$XAUTH:$XAUTH:rw --env=XAUTHORITY=$XAUTH --env=DISPLAY=unix$DISPLAY"

echo Running with workspace: "$WORKSPACE"

docker build -t="openmvs-ubuntu$CUDA_CONTAINER_SUFFIX" --build-arg "USER_ID=$(id -u)" --build-arg "GROUP_ID=$(id -g)" $CUDA_BUILD_ARGS $MASTER_ARGS . 
docker run $CUDA_RUNTIME_ARGS --entrypoint bash --ipc=host --shm-size=4gb -w /work -v "$WORKSPACE:/work" $DISPLAY_ARGS -it openmvs-ubuntu$CUDA_CONTAINER_SUFFIX

