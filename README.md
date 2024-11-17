# MOSS: MObility Simulation System

A GPU-accelerated Large-scale Open Microscopic Traffic Simulation System

Website: https://moss.fiblab.net

## Features

- **Efficient**: MOSS adopts GPU as the computational engine, which accelerates 100 times compared to existing microscopic traffic simulators, allowing rapid simulation of large-scale urban road networks.
- **Realistic**: MOSS provides the cutting-edge AIGC method to generate globally available realistic OD matrices for travel demand generation and allows the user to quickly calibrate the simulation parameters to obtain realistic simulation results.
- **Open**: The simulator, toolchain, and sample programs will be open-sourced on Github for community access, and we hope that more people will join in the development and application of MOSS.

## Related Repositories

- `mosstool`: The toolchain for MOSS, [URL](https://github.com/tsinghua-fib-lab/mosstool).
- sample programs: The sample programs for MOSS, [URL](https://github.com/tsinghua-fib-lab/moss-opt-showcases).

## Installation

### Prerequisites

- Linux
- CUDA 11.8
- Python >= 3.8

### Install

```bash
pip install python-moss
```

## FAQ

Q1: How to resolve the error `ImportError: /.../libstdc++.so.6: version 'GLIBCXX_3.4.30' not found`?

A1: Run `conda install -c conda-forge libstdcxx-ng=12` in the current conda environment.

## Development

#### Build

1. Install Boost
```bash
wget -O boost_1_86_0.tar.gz https://archives.boost.io/release/1.86.0/source/boost_1_86_0.tar.gz
tar -zxvf boost_1_86_0.tar.gz
cd boost_1_86_0
./bootstrap.sh --with-libraries=filesystem,iostreams,program_options,regex,system --prefix=/usr/local  # avro dependency
./b2 install
cd ..
rm -r boost_1_86_0
rm boost_1_86_0.tar.gz
```

## From v0.4 to v1.0

That is what we change and why we change it.
- Focus on the microscopic traffic simulation only (vehicle and pedestrian), no crowd in AOI, no bus for more clear code to support community contribution.
- No overlap in junction to avoid deadlock following CBLab's design.
- Can output files with widely-used data format for visualization (visualization is the first for the user to understand the simulation). We choose AVRO as the output format.
- AOI is just as a marker of the starting/ending point of vehicles/pedestrians, no other functions for more clear code.
- clear code structure and documentation written in English.
