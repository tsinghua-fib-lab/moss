#ifndef SRC_CONTAINERS_ARRAY_CUH_
#define SRC_CONTAINERS_ARRAY_CUH_

#include <vector>
#include "mem/mem.cuh"
#include "utils/macro.h"

template <class T>
struct MArrayT {
  static T* New(MemManager* mem, uint size) { return mem->MArray<T>(size); }
  static void Free(MemManager* mem, T* data) { mem->Free(data); }
};

// Array wrapper for easy unified memory management
template <class T, class Allocator>
class Arr {
 public:
  uint size;
  T* data;
  Arr() = default;
  Arr(uint size) : size(size), data(size ? Allocator(size) : nullptr) {}
  void New(MemManager* mem, uint size_) {
    size = size_;
    data = size_ ? Allocator::New(mem, size_) : nullptr;
  }
  void Free(MemManager* mem) {
    if (size) {
      Allocator::Free(mem, data);
      size = 0;
      data = nullptr;
    }
  }
  __host__ __device__ void Fill(const T& value) const {
    for (int i = 0; i < size; ++i) {
      data[i] = value;
    }
  }
  __host__ __device__ operator bool() const { return size; }
  __host__ __device__ T* begin() { return data; }
  __host__ __device__ const T* begin() const { return data; }
  __host__ __device__ T* end() { return data + size; }
  __host__ __device__ const T* end() const { return data + size; }
  __host__ __device__ T& back() { return data[size - 1]; }
  __host__ __device__ const T& back() const { return data[size - 1]; }
  __host__ __device__ T& operator[](uint index) {
    assert(index < size);
    return data[index];
  }
  __host__ __device__ const T& operator[](uint index) const {
    assert(index < size);
    return data[index];
  }
  __host__ __device__ void CopyFrom(const Arr<T, Allocator>& other) {
    assert(size == other.size);
    memcpy(data, other.data, size * sizeof(T));
  }
  void Save(std::vector<T>& out) const {
    out.resize(size);
    if (size) {
      memcpy(out.data(), data, size * sizeof(T));
    }
  }
  void Load(const std::vector<T>& in) {
    assert(size == in.size());
    memcpy(data, in.data(), size * sizeof(T));
  }
};

template <class T>
using MArr = Arr<T, MArrayT<T>>;

#endif
