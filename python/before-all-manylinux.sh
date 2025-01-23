#! /bin/bash

set -e
set -x

pip install cmake
yum install -y wget

# Install Boost
wget -O boost_1_86_0.tar.gz https://archives.boost.io/release/1.86.0/source/boost_1_86_0.tar.gz
tar -zxf boost_1_86_0.tar.gz
cd boost_1_86_0
./bootstrap.sh --with-libraries=filesystem,iostreams,program_options,regex,system --prefix=/usr/local  # avro dependency
./b2 cxxflags="-fPIC -std=c++17" install # C++17 to match moss
cd ..
rm -r boost_1_86_0
rm boost_1_86_0.tar.gz

# Install CUDA 11.8:
yum-config-manager --add-repo https://developer.download.nvidia.com/compute/cuda/repos/rhel7/x86_64/cuda-rhel7.repo
yum install --setopt=obsoletes=0 -y \
    cuda-nvcc-11-8 \
    cuda-cudart-devel-11-8 \
    cuda-nvtx-11-8
ln -s cuda-11.8 /usr/local/cuda

rm -r /project/build || true
mkdir -p /project/build
