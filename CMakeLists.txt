cmake_minimum_required(VERSION 3.18)

# project info
project(cuda LANGUAGES CXX CUDA)

# global settings
# activate intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set runtime output dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build/bin/")

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -fPIC")
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# include cmake
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
include(BuildType)
# if Release build, disable all warning
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-w)
endif(CMAKE_BUILD_TYPE STREQUAL "Release")

include(FetchContent)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG 10.2.1
)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
  GIT_TAG 0.8.0
)
FetchContent_Declare(
  argparse
  GIT_REPOSITORY https://github.com/p-ranav/argparse
  GIT_TAG v3.0
)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11
  GIT_TAG v2.12.0
)
FetchContent_Declare(
  cityproto
  GIT_REPOSITORY https://github.com/tsinghua-fib-lab/cityproto
  GIT_TAG v1.25.1
)
set(FETCHCONTENT_QUIET OFF)
set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "")
set(protobuf_BUILD_TESTS OFF CACHE BOOL "")
set(protobuf_BUILD_PROTOC_BINARIES OFF CACHE BOOL "")
set(gRPC_BUILD_TEST OFF CACHE BOOL "")
set(gRPC_BUILD_TESTS OFF CACHE BOOL "")
set(gRPC_BUILD_CODEGEN OFF CACHE BOOL "")
set(gRPC_BUILD_CSHARP_EXT OFF CACHE BOOL "")
set(gRPC_BUILD_GRPC_CPP_PLUGIN OFF CACHE BOOL "")
set(gRPC_BUILD_GRPC_CSHARP_PLUGIN OFF CACHE BOOL "")
set(gRPC_BUILD_GRPC_NODE_PLUGIN OFF CACHE BOOL "")
set(gRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN OFF CACHE BOOL "")
set(gRPC_BUILD_GRPC_PHP_PLUGIN OFF CACHE BOOL "")
set(gRPC_BUILD_GRPC_PYTHON_PLUGIN OFF CACHE BOOL "")
set(gRPC_BUILD_GRPC_RUBY_PLUGIN OFF CACHE BOOL "")
FetchContent_MakeAvailable(fmt yaml-cpp argparse pybind11 cityproto)

add_subdirectory(src)

# add_subdirectory(tests)
