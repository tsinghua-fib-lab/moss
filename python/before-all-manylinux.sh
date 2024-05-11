#! /bin/bash

set -e
set -x

pip install cmake
yum install -y wget

# Install CUDA 12.4:
yum-config-manager --add-repo https://developer.download.nvidia.com/compute/cuda/repos/rhel7/x86_64/cuda-rhel7.repo
yum install --setopt=obsoletes=0 -y \
    cuda-nvcc-12-4 \
    cuda-cudart-devel-12-4 \
    cuda-nvtx-12-4
ln -s cuda-12.4 /usr/local/cuda

# Install gRPC
wget -O grpc.src.tar.gz http://tsingroc-private-binary.oss-cn-beijing.aliyuncs.com/grpc1.49.2.src.tar.gz
tar -xzf grpc.src.tar.gz
cd grpc
mkdir -p "cmake/build"
cd "cmake/build"
cmake ../.. -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF
make -j && make -j install && ldconfig && cd ../../
cd .. && rm -r grpc
rm grpc.src.tar.gz
