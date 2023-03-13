ARG BASE_IMAGE=ubuntu:22.04

FROM $BASE_IMAGE

ARG MASTER
ARG USER_ID
ARG GROUP_ID
ARG CUDA=0

COPY buildInDocker.sh /tmp/buildInDocker.sh
RUN /tmp/buildInDocker.sh --cuda $CUDA && rm /tmp/buildInDocker.sh

USER user

# Add binaries to path
ENV PATH /usr/local/bin/OpenMVS:$PATH
