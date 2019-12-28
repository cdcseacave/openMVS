docker pull openmvs/openmvs-ubuntu:latest
docker run -w /working -v $1:/working -it openmvs/openmvs-ubuntu:latest
