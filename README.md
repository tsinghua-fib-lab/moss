# MOSS: MObility Simulation System

[![Upload Python Package](https://github.com/tsinghua-fib-lab/moss/actions/workflows/python-publish.yml/badge.svg)](https://github.com/tsinghua-fib-lab/moss/actions/workflows/python-publish.yml)

A GPU-accelerated Large-scale Open Microscopic Traffic Simulation System

Website: https://moss.fiblab.net

Documentation: https://python-moss.readthedocs.io/en/latest/

API Reference: https://python-moss.readthedocs.io/en/latest/apidocs/index.html

## Features

- **Efficient**: MOSS adopts GPU as the computational engine, which accelerates 100 times compared to existing microscopic traffic simulators, allowing rapid simulation of large-scale urban road networks.
- **Realistic**: MOSS provides the cutting-edge AIGC method to generate globally available realistic OD matrices for travel demand generation and allows the user to quickly calibrate the simulation parameters to obtain realistic simulation results.
- **Open**: The simulator, toolchain, and sample programs will be open-sourced on Github for community access, and we hope that more people will join in the development and application of MOSS.

## Awesome MOSS

The related repositories of MOSS are as follows:
- `cityproto`: The protobuf-driven data structure definition for all city simulation projects of FIBLAB, [URL](https://github.com/tsinghua-fib-lab/cityproto). The project provides C/C++, Golang, Python, and Javascript/Typescript interfaces to access the data structure. (match **python-moss>=1.0.0**)
- `mosstool`: The toolchain for MOSS, [URL](https://github.com/tsinghua-fib-lab/mosstool). The project is a Python packages that includes map building, traffic demand generation, SUMO format conversion, and some format conversion utilities. It is the key to build the input of MOSS. (match **python-moss>=1.0.0**)
- `routing`: A gRPC-powered routing service with A* shortest path algorithm, [URL](https://github.com/tsinghua-fib-lab/routing). The project is the necessary service for `mosstool` when generating traffic demand or just calling `pre_route` functions to compute the routes of people. (match **python-moss>=1.0.0**)
- `moss-replay`: A internal JS/TS package for building web-based UI to visualize the simulation results, [URL](https://github.com/tsinghua-fib-lab/moss-replay). (match **python-moss>=1.0.0**)
- `moss-webui-frontend`: A web UI to visualize the PostgreSQL output of the MOSS project, [URL](https://github.com/tsinghua-fib-lab/moss-webui-frontend). The project is only a frontend built on `moss-replay` and provide a simple 2D visualization for debugging. Users can choose the backend URL in the web settings to connect to their backend and databases. (match **python-moss>=1.0.0**)
- `moss-webui-backend`: A web backend to provide HTTP Restful API for the database output of the MOSS project, [URL](https://github.com/tsinghua-fib-lab/moss-webui-backend). The project provides Docker images to allow users to deploy the backend easily. (match **python-moss>=1.0.0**)
- `moss-ui`: A simple UI to visualize the AVRO output of the MOSS project by adapting `moss-replay` to the AVRO output, [URL](https://github.com/tsinghua-fib-lab/moss-ui). The project is a desktop application using web technologies, but it now faces performance challenges to manage data without a database. (match **python-moss>=1.0.0**, *unstable*)
- MOSS based transportation optimization benchmark: A benchmark for large-scale transportation optimization benchmark based on MOSS, [URL](https://github.com/tsinghua-fib-lab/moss-benchmark). The project includes simulation performance evaluation, traffic signal control optimization, dynamic lane control optimization, tidal lane control optimization, congestion pricing optimization, and road planning optimization. (match python-moss>=0.3.3,<1.0.0, **will be updated**)
- sample programs: The sample programs for MOSS, [URL](https://github.com/tsinghua-fib-lab/moss-opt-showcases). The project includes the showcases of MOSS for traffic signal control optimization. (match python-moss==0.2.0, *deprecated*)

## Installation

#### Prerequisites

- Linux
- CUDA 11.8
- Python >= 3.9

#### Install

```bash
pip install python-moss
```

#### Very Simple Demo

We assume that you have the map input `map.pb` and the person input `person.pb` generated by `mosstool`. The following code is a simple demo to run the simulation.
```python
from moss import Engine, Verbosity
from moss.export import DBRecorder

e = Engine(
    "name",
    "data/map.pb",
    "data/person.pb",
    0,
    1,
    output_dir="output", # AVRO output, local directory
    speed_stat_interval=300, # open road status statistics
    verbose_level=Verbosity.ALL,
)
recorder = DBRecorder(
    e,
    "postgres://user:password@url:port/simulation",
    "map_db.map_coll", # map collection used for webui-backend
    "name",
) # used for PostgreSQL output
for _ in range(3600):
    e.next_step(1)
    recorder.record()
    # YOU CAN DO SOMETHING HERE
    # persons = e.fetch_persons()
    # ...
# save the simulation results to the database
recorder.flush()
```

## Development

If you are interested in the development of MOSS, you can follow the instructions below.

#### Prequsites

- Linux
- CUDA 11.8 or higher
- CMake >= 3.18
- Python >= 3.9
- Network that can access the GitHub repository

#### Compile and Build

1. Install Boost
```bash
wget -O boost_1_86_0.tar.gz https://archives.boost.io/release/1.86.0/source/boost_1_86_0.tar.gz
tar -zxvf boost_1_86_0.tar.gz
cd boost_1_86_0
./bootstrap.sh --with-libraries=filesystem,iostreams,program_options,regex,system --prefix=/usr/local  # avro dependency
./b2 cxxflags=-fPIC install
cd ..
rm -r boost_1_86_0
rm boost_1_86_0.tar.gz
```

2. Build MOSS
```bash
mkdir build
cd build
cmake ..
make -j
```

3. Test MOSS: You can use the compiled MOSS executable to run the simulation to skip the python package installation.
```bash
./build/bin/moss -h

Usage: Moss [--help] [--version] --name VAR --config VAR [--gpu VAR] [--quiet]

Optional arguments:
  -h, --help     shows help message and exits 
  -v, --version  prints version information and exits 
  -n, --name     name of the simulation [required]
  -c, --config   path to config file [required]
  --gpu          GPU device ID [nargs=0..1] [default: 0]
  -q, --quiet
```

The config file is a YAML file that contains the simulation parameters. You can refer to the [ConfigFile](examples/config.yaml) in the repository. The meanings of the parameters can be found in the python package Engine's docstring in [engine.py](python/src/moss/engine.py).

4. Install Python Package
```bash
pip install . -v
```

5. You can submit a pull request to the repository to contribute to the project.

## Version History

#### v1.1

- We apply a CUDA memory arena to do dynamic memory allocation in the GPU memory, which make the project been seen as a right CUDA project.
- Based on the memory arena, we implement the checkpoint mechanism to save the simulation state to CPU memory and restore it from CPU memory.
- More efficient database recorder with incompatible APIs.
- Some bug fixes, performance improvements and more Python API.

#### From v0.4 to v1.0

That is what we change and why we change it.
- Focus on the microscopic traffic simulation only (vehicle and pedestrian), no crowd in AOI, no bus for more clear code to support community contribution.
- No overlap in junction to avoid deadlock following CBLab's design.
- Can output files with widely-used data format for visualization (visualization is the first for the user to understand the simulation). We choose AVRO as the output format.
- AOI is just as a marker of the starting/ending point of vehicles/pedestrians, no other functions for more clear code.
- clear code structure and documentation written in English.
