#include "utils/color_print.h"
#include "utils/debug.cuh"

__managed__ uint64_t _error = 0;

__host__ __device__ void SetError(uint64_t error) { _error = error; }

__host__ __device__ uint64_t ErrorCode(ErrorType type, uint detail) {
  return (detail << 4) | uint(type);
}

void CheckError(const char* file, int lineno) {
  if (!_error) {
    return;
  }
  auto type = (ErrorType)(_error & 0xf);
  uint detail = _error >> 4;
  switch (type) {
    case ErrorType::ANY: {
      throw std::runtime_error("Error: Code " + std::to_string(detail) +
                               ".\nFile \"" + file + "\", line " +
                               std::to_string(lineno));
    } break;
    default:;
  }
}
