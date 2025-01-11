# MOSS: MObility Simulation System


[![Upload Python Package](https://github.com/tsinghua-fib-lab/moss/actions/workflows/python-publish.yml/badge.svg)](https://github.com/tsinghua-fib-lab/moss/actions/workflows/python-publish.yml)

A GPU-accelerated Large-scale Open Microscopic Traffic Simulation System

Website: [https://moss.fiblab.net](https://moss.fiblab.net)

Documentation: [https://moss.fiblab.net/docs/introduction](https://moss.fiblab.net/docs/introduction)

API Reference: [https://docs.fiblab.net/moss](https://docs.fiblab.net/moss)

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

## Table of Contents

```{toctree}
:maxdepth: 2

1.get-started
2.development
apidocs/index
3.version
```
