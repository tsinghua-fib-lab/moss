#ifndef SRC_MEM_MEM_CUH_
#define SRC_MEM_MEM_CUH_

#include <cuda.h>
#include <stdio.h>
#include <cassert>
#include <mutex>
#include "utils/macro.h"

unsigned long long operator""_GiB(unsigned long long x);
unsigned long long operator""_MiB(unsigned long long x);

struct MemManager {
  unsigned long long _start, _ptr, _size, _usage;
  std::mutex mtx;
  bool verbose;
  uint device;

  __device__ unsigned long long _d_malloc(size_t size);
  unsigned long long _malloc(size_t size);
  static MemManager* New(size_t size, uint device, bool verbose) {
    MemManager* ptr;
    CUCHECK(cudaMallocManaged((void**)&ptr, sizeof(MemManager)));
    CUCHECK(cudaMemset(ptr, 0, sizeof(MemManager)));
    ptr->Init(size, device, verbose);
    return ptr;
  }
  void Init(size_t size, uint device, bool verbose);
  void PrintUsage();
  void PreferCPU();
  void PreferGPU();
  template <class T>
  void Malloc(T** x, size_t size) {
    *x = (T*)_malloc((size + 15) & ~15);
  }
  template <class T>
  void MallocManaged(T** x, size_t size) {
    *x = (T*)_malloc((size + 15) & ~15);
  }
  template <class T>
  __host__ __device__ void Free(T*) {}

  // 由cudaMalloc分配的值
  template <class T>
  T* DValue() {
    T* t;
    Malloc(&t, sizeof(T));
    return t;
  }

  // 由cudaMallocManaged分配的值
  template <class T>
  T* MValue() {
    T* t;
    MallocManaged(&t, sizeof(T));
    return t;
  }

  template <class T>
  T* MValueZero() {
    T* t;
    MallocManaged(&t, sizeof(T));
    CUCHECK(cudaMemset(t, 0, sizeof(T)));
    return t;
  }

  template <class T>
  void MValueZero(T*& x) {
    MallocManaged(&x, sizeof(T));
    CUCHECK(cudaMemset(x, 0, sizeof(T)));
  }

  template <class T>
  void MValue(T*& x) {
    MallocManaged(&x, sizeof(T));
  }

  // 由cudaMalloc分配的数组
  template <class T>
  T* DArray(size_t size) {
    T* t;
    Malloc(&t, size * sizeof(T));
    return t;
  }

  template <class T>
  void DArray(T*& x, size_t size) {
    Malloc(&x, size * sizeof(T));
  }

  // 由cudaMallocManaged分配的数组
  template <class T>
  T* MArrayZero(size_t size) {
    T* t;
    MallocManaged(&t, size * sizeof(T));
    CUCHECK(cudaMemset(t, 0, size * sizeof(T)));
    return t;
  }

  template <class T>
  void TryFree(T* x) {
    if (x) {
      Free(x);
    }
  }

  // 释放并重新申请MArray
  template <class T>
  void ReMArray(T*& x, size_t size) {
    TryFree(x);
    MallocManaged(&x, size * sizeof(T));
  }

  // 全零的MArray
  template <class T>
  void MArrayZero(T*& x, size_t size) {
    MallocManaged(&x, size * sizeof(T));
    memset(x, 0, size * sizeof(T));
  }

  template <class T>
  void ReMArrayZero(T*& x, size_t size) {
    TryFree(x);
    MallocManaged(&x, size * sizeof(T));
    memset(x, 0, size * sizeof(T));
  }
};

// 从GPU复制数据到CPU
template <class T>
void Sync(T& dst, const T* src) {
  CUCHECK(cudaMemcpy(&dst, src, sizeof(T), cudaMemcpyDeviceToHost));
}

// 从GPU复制数组到CPU
template <class T>
void Sync(T* dst, const T* src, size_t size) {
  CUCHECK(cudaMemcpy(dst, src, size * sizeof(T), cudaMemcpyDeviceToHost));
}

template <class T>
__global__ void _cpy(T* dst, const T* src, size_t size) {
  memcpy(dst, src, size * sizeof(T));
}

// 从GPU Heap复制数据到CPU
// template <class T>
// void SyncHeap(T* dst, const T* src, size_t size) {
//   T* tmp = DArray<T>(size);
//   _cpy<<<1, 1>>>(tmp, src, size);
//   cudaMemcpy(dst, tmp, size * sizeof(T), cudaMemcpyDeviceToHost);
//   // Free(tmp);
// }

#endif
