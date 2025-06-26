#!/bin/bash

set -e

# Parse arguments to see if --cuda was passed and equals 1 or --master was passed and equals 1
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --cuda)
            CUDA="$2"
            shift
            shift
            ;;
        --master)
            MASTER="$2"
            shift
            shift
            ;;
        --user_id)
            USER_ID="$2"
            shift
            shift
            ;;
        --group_id)
            GROUP_ID="$2"
            shift
            shift
            ;;
        *)
            echo "Unknown argument: $key"
            exit 1
            ;;
    esac
done

if [[ "$CUDA" == "1" ]]; then
    echo "Building with CUDA support"
    EIGEN_BUILD_ARG="-DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda/"
    OPENMVS_BUILD_ARG="-DOpenMVS_USE_CUDA=ON -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/ -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda/ -DCUDA_INCLUDE_DIRS=/usr/local/cuda/include/ -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64 -DCUDA_NVCC_EXECUTABLE=/usr/local/cuda/bin/ -DCMAKE_CUDA_ARCHITECTURES=all -DEIGEN3_INCLUDE_DIR=/usr/local/include/eigen3"
else
    echo "Building without CUDA support"
    EIGEN_BUILD_ARG=""
    OPENMVS_BUILD_ARG="-DOpenMVS_USE_CUDA=OFF"
fi

if [[ "$MASTER" == "1" ]]; then
    echo "Pulling from master branch"
else
    echo "Pulling from develop branch"
fi

DEBIAN_FRONTEND=noninteractive apt-get update -yq

DEBIAN_FRONTEND=noninteractive apt-get -yq install build-essential git cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev libglew-dev libglfw3-dev && rm -rf /var/lib/apt/lists/*

# Eigen
git clone https://gitlab.com/libeigen/eigen --branch 3.4
mkdir eigen_build
cd eigen_build &&\
    cmake . ../eigen $EIGEN_BUILD_ARG &&\
    make && make install &&\
    cd .. && rm -rf eigen_build eigen

# Boost
DEBIAN_FRONTEND=noninteractive apt-get -yq install libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

# OpenCV
DEBIAN_FRONTEND=noninteractive apt-get install -yq libopencv-dev

# CGAL (dependencies not needed to (not) build CGAL, but for using some parts of it)
DEBIAN_FRONTEND=noninteractive apt-get -yq install libboost-program-options-dev libboost-system-dev libboost-thread-dev libgmp-dev libmpfr-dev zlib1g-dev

git clone https://github.com/cgal/cgal --branch=v6.0.1
mkdir cgal_build
cd cgal_build &&\
    cmake . ../cgal &&\
    make && make install &&\
    cd .. && rm -rf cgal_build cgal



# Python
DEBIAN_FRONTEND=noninteractive apt-get -yq install python3-dev

# VCGLib
git clone https://github.com/cdcseacave/VCG.git vcglib

# Build from stable openMVS release or the latest commit from the develop branch
if [[ "$MASTER" == "1" ]]; then
    git clone https://github.com/cdcseacave/openMVS.git --branch master
else
    git clone https://github.com/cdcseacave/openMVS.git --branch develop
fi

mkdir openMVS_build
cd openMVS_build &&\
    cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT=/vcglib $OPENMVS_BUILD_ARG

# Install OpenMVS library
make -j4 &&\
    make install &&\
    cd .. && rm -rf openMVS_build vcglib

# Set permissions such that the output files can be accessed by the current user (optional)
echo "Setting permissions for user $USER_ID:$GROUP_ID"
addgroup --gid $GROUP_ID user &&\
    adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
