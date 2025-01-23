# Development

If you are interested in the development of MOSS, you can follow the instructions below.

## Prerequisites

- Linux
- CUDA 11.8 or higher
- CMake >= 3.18
- Python >= 3.9
- Network that can access the GitHub repository

## Compile and Build

1. Install Boost
```bash
wget -O boost_1_86_0.tar.gz https://archives.boost.io/release/1.86.0/source/boost_1_86_0.tar.gz
tar -zxvf boost_1_86_0.tar.gz
cd boost_1_86_0
./bootstrap.sh --with-libraries=filesystem,iostreams,program_options,regex,system --prefix=/usr/local  # avro dependency
./b2 cxxflags="-fPIC -std=c++17" install  # C++17 to match moss
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

The config file is a YAML file that contains the simulation parameters. You can refer to the [ConfigFile](../examples/config.yaml) in the repository. The meanings of the parameters can be found in the python package Engine's docstring in [moss.engine](apidocs/moss/moss.engine.md).

1. Install Python Package
```bash
pip install . -v
```

:::{note}
We are welcome to any contributions to the project. You can submit a pull request to the repository to contribute to the project.
:::
