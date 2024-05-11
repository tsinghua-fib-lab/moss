get_filename_component(SRC_DIR ../src ABSOLUTE)
add_executable(test
  test_heap.cu
  ${SRC_DIR}/containers/vector.cu
  ${SRC_DIR}/mem/mem.cu
)
set_target_properties(test
  PROPERTIES CUDA_SEPARABLE_COMPILATION ON
)
set_target_properties(test PROPERTIES CUDA_ARCHITECTURES 70)
target_include_directories(test PRIVATE ${SRC_DIR})
