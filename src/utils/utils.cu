#include "utils/macro.h"
#include "utils/utils.cuh"

cudaStream_t NewStream() {
  cudaStream_t s;
  CUCHECK(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking));
  return s;
}
