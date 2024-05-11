#ifndef SRC_UTILS_UTILS_CUH_
#define SRC_UTILS_UTILS_CUH_

#include <cuda.h>
#include <cassert>
#include "utils/macro.h"

#if NDEBUG
#define ASSERT(x) \
  if (!(x)) __trap();
#else
#define ASSERT assert
#endif

// 找到首个不小于target的值的偏移量，返回size表示没找到
template <typename T>
__host__ __device__ uint32_t LowerBound(const T* array, uint32_t size,
                                        const T& target) {
  uint32_t imin = 0, imax = size;
  while (imax > imin) {
    auto imid = (imax + imin) >> 1;
    if (array[imid] < target) {
      imin = imid + 1;
    } else {
      imax = imid;
    }
  }
  return imin;
}

// 找到首个大于target的值的偏移量，返回size表示没找到
template <typename T>
__host__ __device__ uint32_t UpperBound(const T* array, uint32_t size,
                                        const T& target) {
  uint32_t imin = 0, imax = size;
  while (imax > imin) {
    auto imid = (imax + imin) >> 1;
    if (array[imid] <= target) {
      imin = imid + 1;
    } else {
      imax = imid;
    }
  }
  return imin;
}

template <typename T>
__host__ __device__ T Clamp(const T& val, const T& lo, const T& hi) {
  return (val < lo) ? lo : (hi < val) ? hi : val;
}

template <typename T>
__host__ __device__ void Clamp_(T& val, const T& lo, const T& hi) {
  val = (val < lo) ? lo : (hi < val) ? hi : val;
}

template <typename T>
__device__ void Min_(T& a, const T& b) {
  a = min(a, b);
}

template <typename T>
__device__ void Max_(T& a, const T& b) {
  a = max(a, b);
}

// 返回自己是否是mask中的第一个
__device__ __forceinline__ bool IsLeader(uint mask) {
  return (mask & ~((1 << (WARP_ID + 1)) - 1)) == 0;
}

// 返回自己是否是Warp活动线程中的第一个
__device__ __forceinline__ bool IsWarpLeader() {
  return IsLeader(__activemask());
}

template <class T>
__device__ void atomicExch(T** addr, T* val) {
  static_assert(sizeof(T*) == sizeof(unsigned long long));
  atomicExch((unsigned long long*)addr, (unsigned long long)val);
}

template <class T>
__device__ T* atomicLoad(T** addr) {
  static_assert(sizeof(T*) == sizeof(unsigned long long));
  return (T*)atomicAdd((unsigned long long*)addr, 0ull);
}

cudaStream_t NewStream();

template <class U, class V>
__device__ __host__ U min(U u, V v) {
  return u <= v ? u : v;
}

template <class U, class V>
__device__ __host__ U max(U u, V v) {
  return u >= v ? u : v;
}

// 自动配置kernel运行的grid和block大小
// template <class Func, class... Args>
// void Launch(uint size, cudaStream_t stream, Func func, Args&&... args) {
//   int grid_size, block_size;
//   if (!size) {
//     return;
//   }
//   CUCHECK(cudaOccupancyMaxPotentialBlockSize(&grid_size, &block_size,
//                                              (void*)func, 0, size));
//   // printf("Suggestion: <%d,%d>\n", grid_size, block_size);
//   grid_size = (size + block_size - 1) / block_size;
//   func<<<grid_size, block_size, 0, stream>>>(std::forward<Args>(args)...);
// }

template <class Func>
void SetGridBlockSize(Func func, int size, int sm_count, int& grid_size,
                      int& block_size) {
  CUCHECK(cudaOccupancyMaxPotentialBlockSize(&grid_size, &block_size,
                                             (void*)func, 0, 0));
  block_size = (block_size + 31) / 32;
  // 增大grid_size直到与SM数量相当
  while (block_size >= 2 &&
         (size + block_size * 16 - 1) / (block_size * 16) <= sm_count) {
    block_size /= 2;
  }
  block_size *= 32;
  grid_size = (size + block_size - 1) / block_size;
}

void PrintMem();

// 二分查找 arr[i-1]<=x<arr[i]，i=0~n
template <class T>
__device__ __host__ uint Search(T* arr, uint n, T x) {
  uint i = 0, j = n;
  while (i < j) {
    uint m = (i + j) >> 1;
    if (x < arr[m]) {
      j = m;
    } else {
      i = m + 1;
    }
  }
  return i;
}

// 从链表中移除元素
template <class T>
__device__ void ListRemove(T* x, T*& head) {
  if (x->prev) {
    x->prev->next = x->next;
  } else {
    head = x->next;
  }
  if (x->next) {
    x->next->prev = x->prev;
  }
}

// 从指针数组中构建链表
template <class T>
__device__ void ListLink(T** start, T** end) {
  if (start == end) {
    return;
  }
  (**start).prev = nullptr;
  --end;
  (**end).next = nullptr;
  if (start < end) {
    (**start).next = start[1];
    (**end).prev = end[-1];
    for (auto** p = start + 1; p < end; ++p) {
      (**p).prev = p[-1];
      (**p).next = p[1];
    }
  }
}

template <class T>
__device__ bool ListCheckLoop(T* p) {
  if (p && p->next) {
    T* q = p->next->next;
    while (q) {
      if (p == q) {
        return true;
      }
      p = p->next;
      q = q->next;
      if (!q) {
        break;
      }
      q = q->next;
    }
  }
  return false;
}

template <class T>
__device__ __host__ void Swap(T& a, T& b) {
  T c(a);
  a = b;
  b = c;
}

template <class T>
__device__ T AngleNorm(T x) {
  return x > PI ? x - 2 * PI : x < -PI ? x + 2 * PI : x;
}

template <class T>
__device__ T AngleLerp(T a, T b, T k) {
  return a + AngleNorm(b - a) * k;
}

// template <typename T, size_t N>
// class Average {
//  private:
//   static_assert(N);
//   T storage[N];
//   bool full = false;
//   size_t ptr = 0;

//  public:
//   Average() = default;
//   double update(T x) {
//     storage[ptr] = x;
//     ptr = (ptr + 1) % N;
//     if (ptr == 0) {
//       full = true;
//     }
//     return full ? double(x - storage[ptr]) / N : double(x) / ptr;
//   }
// };

#endif  // SRC_UTILS_UTILS_CUH_
