#!/bin/bash

rm -rf openMVS_build && mkdir openMVS_build

cd openMVS_build &&\
    cmake .. -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT=/vcglib

# add below args for CUDA, refer docker/buildInDocker.sh for base container and additional stuff required
# -DOpenMVS_USE_CUDA=ON -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/ -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda/ -DCUDA_INCLUDE_DIRS=/usr/local/cuda/include/ -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64 -DCUDA_NVCC_EXECUTABLE=/usr/local/cuda/bin/

# Install OpenMVS library
make -j && sudo make install