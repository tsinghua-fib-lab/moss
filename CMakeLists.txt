cmake_minimum_required(VERSION 3.18)

# project info
project(cuda LANGUAGES CXX CUDA)

# global settings
include(FetchContent)
FetchContent_Declare(
  gRPC
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG v1.49.2 # e.g v1.28.0
)
set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailable(gRPC)

# activate intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set runtime output dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/build/bin/")

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# include cmake
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
include(BuildType)

add_subdirectory(src)

# add_subdirectory(tests)
