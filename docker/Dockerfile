FROM ubuntu:18.04

# Initialize the environment 
RUN apt update
RUN apt install -y cmake git vim

# Prepare and empty machine for building:
RUN apt-get update -yq
RUN apt-get install -yq
RUN apt-get -y install git mercurial cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev

# Eigen (Known issues with eigen 3.3.7 as of 12/10/2019, so using this tested branch/commit instead) 
RUN git clone https://gitlab.com/libeigen/eigen --branch 3.2 ed5cd0a4d16e12daa1bef608628c103e67969d63
RUN mkdir eigen_build
RUN cd eigen_build &&\
	cmake . ../eigen &&\
	make && make install &&\
	cd ..

# Boost
RUN apt-get -y install libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

# OpenCV
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq libopencv-dev

# CGAL
RUN apt-get -y install libcgal-dev libcgal-qt5-dev

# VCGLib
RUN git clone https://github.com/cdcseacave/VCG.git vcglib

# Build from stable release openMVS1.0
RUN git clone https://github.com/cdcseacave/openMVS.git --branch v1.0

# Uncomment below (and comment above) to use the latest commit from the master branch
# RUN git clone https://github.com/cdcseacave/openMVS.git openMVS

RUN mkdir openMVS_build
RUN cd openMVS_build &&\
	cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT=/vcglib

# Install OpenMVS library
RUN cd openMVS_build &&\
	make -j4 &&\
	make install
ENV PATH /usr/local/bin/OpenMVS:$PATH
