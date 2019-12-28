docker build -t="openmvs-ubuntu" .;
docker run -w /working -v $1:/working -it openmvs-ubuntu;