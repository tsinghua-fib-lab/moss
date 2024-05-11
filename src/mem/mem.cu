#include <cuda.h>
#include <cstdint>
#include <mutex>
#include "mem/mem.cuh"
#include "utils/macro.h"

__device__ unsigned long long MemManager::_d_malloc(size_t size) {
  auto x = atomicAdd(&_usage, size);
  assert(x + size <= _size);
  return atomicAdd(&_ptr, size);
}

unsigned long long MemManager::_malloc(size_t size) {
  std::lock_guard lock(mtx);
  _usage += size;
  if (_usage > _size) {
    printf(RED("Memory limit exceeded! Request: %.3fGiB+%.3fGiB\n"),
           (double)_usage / (1 << 30), (double)size / (1 << 30));
    exit(-1);
  }
  auto x = _ptr;
  _ptr += size;
  return x;
}

void MemManager::PreferCPU() {
  CUCHECK(cudaMemAdvise((void*)_start, _size, cudaMemAdviseSetPreferredLocation,
                        cudaCpuDeviceId));
}

void MemManager::PreferGPU() {
  CUCHECK(cudaMemAdvise((void*)_start, _size,
                        cudaMemAdviseUnsetPreferredLocation, cudaCpuDeviceId));
  CUCHECK(cudaMemAdvise((void*)_start, _size, cudaMemAdviseSetPreferredLocation,
                        device));
}

void MemManager::Init(size_t size, uint device, bool verbose) {
  this->device = device;
  this->verbose = verbose;
  size_t free_byte = 0;
  size_t total_byte = 0;
  CUCHECK(cudaMemGetInfo(&free_byte, &total_byte));
  if (verbose)
    printf(GREEN("GPU memory: %.3fMiB/%.3fMiB\n"),
           double(total_byte - free_byte) / (1 << 20),
           double(total_byte) / (1 << 20));
  // 分配内存
  CUCHECK(cudaMallocManaged((void**)&_ptr, size));
  if (!_ptr) {
    printf(RED("Cannot allocate memory: %.3fMiB\n"), double(size) / (1 << 20));
    exit(-1);
  } else {
    if (verbose)
      printf(GREEN("Allocate memory: %.3fMiB\n"), double(size) / (1 << 20));
  }
  _start = _ptr;
  _size = size;
  _usage = 0;
}

void MemManager::PrintUsage() {
  if (verbose) {
    printf("Memory usage: %.3fMiB/%.3fMiB\n", (double)_usage / (1 << 20),
           double(_size) / (1 << 20));
  }
}

unsigned long long operator"" _GiB(unsigned long long x) { return x << 30; }

unsigned long long operator"" _MiB(unsigned long long x) { return x << 20; }

void PrintMem() {
  size_t free, total;
  CUCHECK(cudaMemGetInfo(&free, &total));
  printf("Mem: %ld/%ldMiB\n", (total - free) >> 20, total >> 20);
}
