# ************ README ************
# 1. 通常来说，在CMake中采用conan管理的依赖的包名为CONAN_PKG::<package_name>，
# 其中package_name为conanfile.txt中所指定的包名。
#
# 2. 如何选择使用 PUBLIC 还是 PRIVATE 作为target系列函数的访问控制选项
# - PUBLIC: 所链接的库应用在本lib的头文件中
# - PRIVATE: 所链接的库只应用在本lib的源文件中
# ************ README ************

# set(CMAKE_CXX_CPPLINT "cpplint;--filter=-build/c++11")

find_package(CUDAToolkit)

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION FALSE)

execute_process(
  COMMAND bash -c "date +%y%m%d.%H%M%S.%N"
  OUTPUT_VARIABLE DATE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_executable(cuda
  main.cu
  containers/vector.cu
  entity/aoi/aoi.cu
  entity/aoi/crowd/crowd.cu
  entity/junction/junction.cu
  entity/junction/trafficlight/trafficlight.cu
  entity/lane/lane.cu
  entity/person/person.cu
  entity/person/vehicle.cu
  entity/person/pedestrian.cu
  entity/road/road.cu
  mem/mem.cu
  output.cu
  rpc/routing.cu
  simulet.cu
  utils/color_print.cc
  utils/debug.cu
  utils/geometry.cu
  utils/timer.cc
  utils/utils.cu
)
set_target_properties(cuda PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_ARCHITECTURES 70
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-G>)
endif()

target_include_directories(cuda PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cuda
  PRIVATE cityproto
  PRIVATE CUDA::nvToolsExt
  PRIVATE argparse
  PRIVATE fmt
  PRIVATE yaml-cpp
)
target_compile_definitions(cuda PRIVATE VERSION="build.${DATE}")

pybind11_add_module(_simulet
  python_api.cu
  containers/vector.cu
  entity/aoi/aoi.cu
  entity/aoi/crowd/crowd.cu
  entity/junction/junction.cu
  entity/junction/trafficlight/trafficlight.cu
  entity/lane/lane.cu
  entity/person/person.cu
  entity/person/vehicle.cu
  entity/person/pedestrian.cu
  entity/road/road.cu
  mem/mem.cu
  output.cu
  rpc/routing.cu
  simulet.cu
  utils/color_print.cc
  utils/debug.cu
  utils/geometry.cu
  utils/timer.cc
  utils/utils.cu
)
set_target_properties(_simulet PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_ARCHITECTURES 70
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-G>)
endif()

target_link_libraries(_simulet
  PRIVATE cityproto
  PRIVATE CUDA::nvToolsExt
  PRIVATE fmt
)
target_compile_definitions(_simulet PRIVATE VERSION="build.${DATE}")
target_include_directories(_simulet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
