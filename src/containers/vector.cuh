#ifndef SRC_CONTAINERS_VECTOR_CUH_
#define SRC_CONTAINERS_VECTOR_CUH_

#include <cuda.h>
#include <cassert>
#include <utility>
#include <vector>
#include "mem/mem.cuh"
#include "utils/debug.cuh"
#include "utils/utils.cuh"

using Mutex = uint;
// 给Mutex加锁
__device__ void Lock(Mutex& m);
// 给Mutex去锁
__device__ void Unlock(Mutex& m);

// 向量的初始大小，不要小于32
constexpr int VECTOR_INITIAL_CAPACITY = 32;

// GPU上的向量结构体
template <class T>
struct DVector {
  bool is_fixed : 1;
  uint capacity : 31, size;
  T* data;
  MemManager* mem;
  Mutex mutex;
  uint64_t _error_code;

  __device__ uint AppendNoLock(const T& x) {
    assert(is_fixed);
    if (size >= capacity) {
      capacity = capacity ? capacity << 1 : VECTOR_INITIAL_CAPACITY;
      auto* arr = (T*)mem->_d_malloc(capacity * sizeof(T));
      if (data) {
        memcpy(arr, data, size * sizeof(T));
        mem->Free(data);
      }
      data = arr;
    }
    data[size] = x;
    return size++;
  }

  // 原子地添加数据，返回数据索引
  __device__ uint Append(const T& x) {
    if (is_fixed) {
      uint index = atomicInc(&size, ALL_BIT);
      if (index >= capacity) {
        SetError(_error_code);
        return 0;
      }
      data[index] = x;
      return index;
    }
    auto mask = __activemask();
    auto is_leader = IsLeader(mask);
    __shared__ T* arr;
    if (is_leader) {
      Lock(mutex);
      if (size + 32 >= capacity) {
        capacity = capacity ? capacity << 1 : VECTOR_INITIAL_CAPACITY;
        arr = (T*)mem->_d_malloc(capacity * sizeof(T));
        if (data) {
          memcpy(arr, data, size * sizeof(T));
          mem->Free(data);
        }
        data = arr;
        __threadfence();
      } else {
        arr = data;
      }
    }
    __syncwarp(mask);
    uint index = atomicInc(&size, ALL_BIT);
    arr[index] = x;
    __syncwarp(mask);
    if (is_leader) {
      Unlock(mutex);
    }
    return index;
  }

  __device__ T& GetAppend() {
    uint index = atomicInc(&size, ALL_BIT);
    assert(index < capacity);
    return data[index];
  }

  void ExpandCpu(uint n) {
    assert(!is_fixed);
    if (size + n + 32 >= capacity) {
      capacity = size + n + 33;
      T* arr = (T*)mem->_malloc(capacity * sizeof(T));
      if (data) {
        memcpy(arr, data, size * sizeof(T));
        mem->Free(data);
      }
      data = arr;
    }
  }

  uint AppendCpu(const T& x) {
    assert(!is_fixed);
    ExpandCpu(0);
    data[size] = x;
    return size++;
  }

  // 预留n的空间
  void Reserve(uint n) {
    assert(!data);
    mem->DArray(data, n);
    capacity = n;
    is_fixed = true;
    // if (capacity < n) {
    //   capacity = n;
    //   T* arr = Malloc<T>(capacity);
    //   if (data) {
    //     memcpy(arr, data, size * sizeof(T));
    //     free(data);
    //   }
    //   data = arr;
    // }
  }

  void Save(std::vector<T>& out) {
    out.resize(size);
    if (size) {
      memcpy(out.data(), data, size * sizeof(T));
    }
  }

  void Load(const std::vector<T>& in) {
    if (in.size() + 32 > capacity) {
      capacity = in.size() + 32;
      T* arr = (T*)mem->_malloc(capacity * sizeof(T));
      if (data) {
        mem->Free(data);
      }
      data = arr;
    }
    size = in.size();
    memcpy(data, in.data(), size * sizeof(T));
  }
  __device__ __host__ void Clear() { size = 0; }
  __device__ __host__ T* begin() { return data; }
  __device__ __host__ const T* begin() const { return data; }
  __device__ __host__ T* end() { return data + size; }
  __device__ __host__ const T* end() const { return data + size; }
  __device__ __host__ T& back() { return data[size - 1]; }
  __device__ __host__ T& operator[](uint index) {
    assert(index < size);
    return data[index];
  }
  __device__ __host__ const T& operator[](uint index) const {
    assert(index < size);
    return data[index];
  }
  __device__ __host__ operator bool() const { return size; }
};

template <class T>
__global__ void _destruct_d_vector(DVector<T>* d_vector) {
  if (d_vector->data) {
    d_vector->mem->Free(d_vector->data);
  }
}

template <class T>
__device__ void SortById(const DVector<T>& arr) {
  for (int i = 0; i + 1 < arr.size; ++i) {
    int m = i;
    for (int j = i + 1; j < arr.size; ++j) {
      if (arr.data[j]->id < arr.data[i]->id) {
        m = j;
      }
    }
    if (m != i) {
      Swap(arr.data[i], arr.data[m]);
    }
  }
}

// CPU上的向量类
template <class T>
class Vector {
 private:
  std::vector<T> _vector;
  MemManager* mem;

 public:
  DVector<T>* _d_vector;
  explicit Vector(MemManager* mem)
      : mem(mem), _d_vector(mem->MValueZero<DVector<T>>()) {
    _d_vector->mem = mem;
  }
  // 获取GPU数据结构
  DVector<T>* Cuda() { return _d_vector; }
  // 获取CPU向量
  std::vector<T>& Cpu() {
    _vector.resize(_d_vector->size);
    // SyncHeap(_vector.data(), _d_vector->data, _d_vector->size);
    memcpy(_vector.data(), _d_vector->data, _d_vector->size * sizeof(T));
    return _vector;
  }
  bool Empty() { return _d_vector->size == 0; }
  bool Size() { return _d_vector->size; }
  void Clear() { _d_vector->size = 0; }
  void Reserve(size_t n) { _d_vector->Reserve(n); }
  ~Vector() {
    _destruct_d_vector<<<1, 1>>>(_d_vector);
    CUCHECK(cudaDeviceSynchronize());
    mem->Free(_d_vector);
  }
};

// 不使用锁的向量，注意不要并行操作同一个对象
// Delete方法要求对象有index属性
template <class T>
struct DVectorNoLock {
  uint size, capacity;
  T* data;
  MemManager* mem;

  __device__ int Append(const T& x) {
    if (size >= capacity) {
      capacity = capacity ? capacity << 1 : VECTOR_INITIAL_CAPACITY;
      T* arr = (T*)mem->_d_malloc(capacity * sizeof(T));
      if (data) {
        memcpy(arr, data, size * sizeof(T));
        mem->Free(data);
      }
      data = arr;
    }
    data[size] = x;
    return size++;
  }

  void ExpandCpu(uint n) {
    if (size + n + 32 >= capacity) {
      capacity = size + n + 33;
      T* arr = (T*)mem->_malloc(capacity * sizeof(T));
      if (data) {
        memcpy(arr, data, size * sizeof(T));
        mem->Free(data);
      }
      data = arr;
    }
  }

  uint AppendCpu(const T& x) {
    ExpandCpu(0);
    data[size] = x;
    return size++;
  }

  // 与最后一个元素交换删除
  __device__ __host__ bool Delete(uint index) {
    --size;
    if (index != size) {
      data[index] = data[size];
      return true;
    }
    return false;
  }

  void Save(std::vector<T>& out) {
    out.resize(size);
    if (size) {
      memcpy(out.data(), data, size * sizeof(T));
    }
  }

  void Load(const std::vector<T>& in) {
    if (in.size() + 32 > capacity) {
      capacity = in.size() + 32;
      T* arr = (T*)mem->_malloc(capacity * sizeof(T));
      if (data) {
        mem->Free(data);
      }
      data = arr;
    }
    size = in.size();
    memcpy(data, in.data(), size * sizeof(T));
  }

  __device__ __host__ void Clear() { size = 0; }
  __device__ __host__ T* begin() { return data; }
  __device__ __host__ const T* begin() const { return data; }
  __device__ __host__ T* end() { return data + size; }
  __device__ __host__ const T* end() const { return data + size; }
  __device__ __host__ T& back() { return data[size - 1]; }
  __device__ __host__ T& operator[](uint index) {
    assert(index < size);
    return data[index];
  }
  __device__ __host__ const T& operator[](uint index) const {
    assert(index < size);
    return data[index];
  }
  __device__ __host__ operator bool() const { return size; }
};

template <class U, class V>
struct Pair {
  U first;
  V second;
};

template <class U, class V>
using DHeapNoLock = DVectorNoLock<Pair<U, V>>;

// 向上交换
template <class U, class V>
__device__ void _heap_up(DHeapNoLock<U, V>& v, uint i) {
  auto x = v[i];
  uint j;
  while (i && x.first < v[(j = (i - 1) >> 1)].first) {
    v[i] = v[j];
    v[j] = x;
    i = j;
  }
}

// 向下交换
template <class U, class V>
__device__ void _heap_down(DHeapNoLock<U, V>& v, uint i) {
  auto x = v[i];
  uint j;
  while ((j = (i << 1) + 1) < v.size) {
    if (j + 1 < v.size && v[j + 1].first <= v[j].first) {
      ++j;
    }
    if (x.first <= v[j].first) {
      break;
    }
    v[i] = v[j];
    v[j] = x;
    i = j;
  }
}

template <class U, class V>
__device__ void Heapify(DHeapNoLock<U, V>& v) {
  if (v.size <= 1) return;
  for (int i = (v.size >> 1) - 1; i > -1; --i) {
    _heap_down(v, i);
  }
}

template <class U, class V>
__device__ void HeapPush(DHeapNoLock<U, V>& v, const Pair<U, V>& x) {
  v.Append(x);
  _heap_up(v, v.size - 1);
}

template <class U, class V>
__device__ V HeapPop(DHeapNoLock<U, V>& v) {
  V ret = v[0].second;
  --v.size;
  if (v.size) {
    v[0] = v.data[v.size];
    _heap_down(v, 0);
  }
  return ret;
}

#endif
