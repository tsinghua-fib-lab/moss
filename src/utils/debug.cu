#include "utils/color_print.h"
#include "utils/debug.cuh"

__managed__ uint64_t _error = 0;
__device__ __host__ void SetError(uint64_t error) { _error = error; }
__device__ __host__ uint64_t ErrorCode(ErrorType type, uint detail) {
  return (detail << 4) | uint(type);
}
void CheckError(const char* file, int lineno) {
  if (!_error) {
    return;
  }
  auto type = (ErrorType)(_error & 0xf);
  uint detail = _error >> 4;
  switch (type) {
    case ErrorType::LANE_VEH_ADD_BUFFER_FULL: {
      Fatal("Error: The vehicle add buffer of Lane [", detail,
            "] is full, consider increasing its size.\nFile \"", file,
            "\", line ", lineno);
    } break;
    case ErrorType::LANE_VEH_REMOVE_BUFFER_FULL: {
      Fatal("Error: The vehicle remove buffer of Lane [", detail,
            "] is full, consider increasing its size.\nFile \"", file,
            "\", line ", lineno);
    } break;
    default:;
  }
}
