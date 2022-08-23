docker build --no-cache -t="openmvs-ubuntu" --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) --file Dockerfile_CUDA .;
docker run --gpus=all --entrypoint bash -w /work -v $1:/work -it openmvs-ubuntu;
