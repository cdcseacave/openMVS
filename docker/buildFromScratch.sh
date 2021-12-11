docker build --no-cache -t="openmvs-ubuntu" --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .;
docker run -w /working -v $1:/working -it openmvs-ubuntu;