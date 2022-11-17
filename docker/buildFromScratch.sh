docker build --no-cache -t="openmvs-ubuntu" --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .;
docker run --entrypoint bash -w /work -v $1:/work -it openmvs-ubuntu;
