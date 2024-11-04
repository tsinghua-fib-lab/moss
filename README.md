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

## V1.0.0 Roadmap

- better vehicle model with our best practices
- linked-list based add/remove buffer to avoid some bug about the size of array
- AVRO based output
- a moss-ui project to show the simulation result locally by WebGL
- cityproto V2
- better documentation with pdoc website
