#ifndef SRC_UTILS_MACRO_H_
#define SRC_UTILS_MACRO_H_

#include <stdint.h>
#include <stdexcept>

using uint = uint32_t;

#define ALL_BIT unsigned(-1)
#define WARP_ID (threadIdx.x & 31)
#define THREAD_ID (threadIdx.x + blockIdx.x * blockDim.x)
#define CUCHECK(x)                                                         \
  do {                                                                     \
    cudaError_t error = x;                                                 \
    if (error != cudaSuccess) {                                            \
      const char* error_string = cudaGetErrorString(error);                \
      printf("%s:%d: (%d) %s\n", __FILE__, __LINE__, error, error_string); \
      throw std::runtime_error(error_string);                              \
    }                                                                      \
  } while (false)
#define CHECK CUCHECK(cudaGetLastError())
#define RED(x) "\033[1;31m" x "\033[0m"
#define GREEN(x) "\033[32m" x "\033[0m"
#define PI 3.1415926536
#define PERF true
#define STUCK_MONITOR false
#define DETERMINISTIC false
#ifdef NDEBUG
#define VER VERSION "[Release]"
#else
#define VER VERSION "[Debug]"
#endif
#endif  // SRC_UTILS_MACRO_H_
