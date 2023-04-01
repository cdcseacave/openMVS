FROM ubuntu:22.04

ARG USERNAME=openmvs
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG INSTALL_ZSH="true"

# Prepare and empty machine for building:
RUN apt-get update -yq

COPY .devcontainer/library-scripts/*.sh /tmp/library-scripts/
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && /bin/bash /tmp/library-scripts/common-debian.sh "${INSTALL_ZSH}" "${USERNAME}" "${USER_UID}" "${USER_GID}" "${UPGRADE_PACKAGES}" "true" "true"\
    #
    # ****************************************************************************
    # * TODO: Add any additional OS packages you want included in the definition *
    # * here. We want to do this before cleanup to keep the "layer" small.       *
    # ****************************************************************************
    # && apt-get -y install --no-install-recommends <your-package-list-here> \
    #
    && apt-get -y install --no-install-recommends build-essential git cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev libglew-dev libglfw3-dev \
    # Boost
    libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev \
    # OpenCV
    libopencv-dev \
    # CGAL
    libcgal-dev libcgal-qt5-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* /tmp/library-scripts

# Eigen 
RUN git clone https://gitlab.com/libeigen/eigen --branch 3.4
RUN mkdir eigen_build
RUN cd eigen_build &&\
    cmake . ../eigen &&\
    make && make install &&\
    cd ..

# VCGLib
RUN git clone https://github.com/cdcseacave/VCG.git vcglib
