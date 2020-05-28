ARG CUDA_V="10.0"
ARG CUDNN_V="7"

FROM nvidia/cuda:${CUDA_V}-cudnn${CUDNN_V}-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
        git \
        curl \
        ca-certificates \
        build-essential \
        wget \
        python3-pip

RUN python3 -m pip install -U pip
RUN pip3 install jupyter numpy torch torchvision
