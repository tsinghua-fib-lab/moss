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

add_compile_options(-fPIC)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION FALSE)

add_subdirectory(lib/argparse)
add_subdirectory(lib/fmt)
add_subdirectory(lib/yaml-cpp)
add_subdirectory(city)

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

target_link_libraries(cuda
  PRIVATE city CUDA::nvToolsExt
  PRIVATE argparse
  PRIVATE fmt
  PRIVATE yaml-cpp
)
target_include_directories(cuda
  PRIVATE lib/argparse/include
  PRIVATE lib/fmt/include
  PRIVATE lib/yaml-cpp/include
)
target_compile_definitions(cuda PRIVATE VERSION="build.${DATE}")

add_subdirectory(lib/pybind11)
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
  PRIVATE city CUDA::nvToolsExt
  PRIVATE fmt
)
target_include_directories(_simulet
  PRIVATE lib/fmt/include
)
target_compile_definitions(_simulet PRIVATE VERSION="build.${DATE}")
