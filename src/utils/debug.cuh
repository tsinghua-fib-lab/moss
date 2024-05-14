#ifndef SRC_UTILS_DEBUG_CUH_
#define SRC_UTILS_DEBUG_CUH_
#include <cuda.h>

enum ErrorType {
  LANE_VEH_ADD_BUFFER_FULL,
  LANE_VEH_REMOVE_BUFFER_FULL,
};
__device__ __host__ void SetError(uint64_t error);
__device__ __host__ uint64_t ErrorCode(ErrorType type, uint detail);
void CheckError(const char* file, int lineno);
#define CHECK_ERROR CheckError(__FILE__, __LINE__)
#endif
