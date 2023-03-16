# Example use:
# ./buildFromScratch.sh --cuda --master --workspace /home/username/datapath/

WORKSPACE=`pwd`

while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --cuda)
            CUDA_BUILD_ARGS="--build-arg CUDA=1 --build-arg BASE_IMAGE=nvidia/cuda:11.8.0-devel-ubuntu22.04"
            CUDA_RUNTIME_ARGS="--gpus all"
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

echo Running with workspace: $WORKSPACE

docker build --no-cache -t=openmvs-ubuntu$CUDA_CONTAINER_SUFFIX --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) $CUDA_BUILD_ARGS $MASTER_ARGS .;
docker run $CUDA_RUNTIME_ARGS --entrypoint bash -w /work -v $WORKSPACE:/work -it openmvs-ubuntu$CUDA_CONTAINER_SUFFIX;
