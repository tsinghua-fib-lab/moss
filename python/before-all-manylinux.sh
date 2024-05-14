#! /bin/bash

set -e
set -x

pip install cmake
# yum install -y wget

# Install CUDA 11.8:
yum-config-manager --add-repo https://developer.download.nvidia.com/compute/cuda/repos/rhel7/x86_64/cuda-rhel7.repo
yum install --setopt=obsoletes=0 -y \
    cuda-nvcc-11-8 \
    cuda-cudart-devel-11-8 \
    cuda-nvtx-11-8
ln -s cuda-11.8 /usr/local/cuda

rm -r /project/build || true
mkdir -p /project/build
