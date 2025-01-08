#ifndef SRC_UTILS_DEBUG_CUH_
#define SRC_UTILS_DEBUG_CUH_
#include <cuda.h>

enum ErrorType {
  ANY,
};

__host__ __device__ void SetError(uint64_t error);

__host__ __device__ uint64_t ErrorCode(ErrorType type, uint detail);

void CheckError(const char* file, int lineno);

#define CHECK_ERROR CheckError(__FILE__, __LINE__)

#endif
