docker build --no-cache -t="openmvs-ubuntu-cuda" --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) --build-arg CUDA=1 --build-arg BASE_IMAGE=nvidia/cuda:11.8.0-devel-ubuntu22.04 .;
docker run --gpus=all --entrypoint bash -w /work -v $1:/work -it openmvs-ubuntu;
