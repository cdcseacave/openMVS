FROM ubuntu:22.04

ARG MASTER
ARG USER_ID
ARG GROUP_ID

# Prepare and empty machine for building:
RUN apt-get update -yq
RUN apt-get -yq install build-essential git cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev libglew-dev libglfw3-dev

# Eigen 
RUN git clone https://gitlab.com/libeigen/eigen --branch 3.4
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
RUN apt-get -yq install libcgal-dev libcgal-qt5-dev

# VCGLib
RUN git clone https://github.com/cdcseacave/VCG.git vcglib

# Build from stable openMVS release or the latest commit from the develop branch
RUN if [[ -n "$MASTER" ]] ; then git clone https://github.com/cdcseacave/openMVS.git --branch master ; else git clone https://github.com/cdcseacave/openMVS.git --branch develop ; fi

RUN mkdir openMVS_build
RUN cd openMVS_build &&\
	cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT=/vcglib -DOpenMVS_USE_CUDA=OFF

# Install OpenMVS library
RUN cd openMVS_build &&\
	make -j4 &&\
	make install

# Set permissions such that the output files can be accessed by the current user (optional)
RUN addgroup --gid $GROUP_ID user &&\
	adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
USER user

# Add binaries to path
ENV PATH /usr/local/bin/OpenMVS:$PATH
