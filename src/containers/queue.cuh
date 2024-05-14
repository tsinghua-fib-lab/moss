#ifndef SRC_CONTAINERS_QUEUE_CUH_
#define SRC_CONTAINERS_QUEUE_CUH_

#include "mem/mem.cuh"
#include "utils/utils.cuh"

// 队列，通过循环使用一个定长数组实现
template <class T>
struct DQueueNoLock {
  uint size, capacity, offset;
  T* data;
  MemManager* mem;
  __device__ void Push(const T& x) {
    if (size == capacity) {
      capacity = capacity ? capacity << 1 : 4;
      auto arr = (T*)mem->_d_malloc(capacity * sizeof(T));
      if (data) {
        auto r = size - offset;
        memcpy(arr, data + offset, r * sizeof(T));
        memcpy(arr + r, data, offset * sizeof(T));
        mem->Free(data);
      }
      data = arr;
      offset = 0;
    }
    auto index = offset + size++;
    data[index >= capacity ? index - capacity : index] = x;
  }
  __device__ T Pop() {
    --size;
    auto index = offset++;
    if (offset == capacity) {
      offset = 0;
    }
    return data[index];
  }
};

#endif
