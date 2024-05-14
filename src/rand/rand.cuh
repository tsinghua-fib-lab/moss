#ifndef SRC_RNAD_RAND_CUH_
#define SRC_RNAD_RAND_CUH_

#include <cuda.h>
#include <cassert>
#include <cstdint>
#include <type_traits>

namespace simulet::rand {

// class Rng1024 {
//   uint64_t state[16];
//   int pos;
//  public:
//   __device__ uint64_t xorshift() {
//     uint64_t state0 = state[pos];
//     pos = (pos + 1) % 16;
//     uint64_t state1 = state[pos];
//     state1 ^= state1 << 31;
//     state1 ^= state1 >> 11;
//     state0 ^= state0 >> 30;
//     state[pos] = state0 ^ state1;
//     return state[pos] * 1181783497276652981LL;
//   }
// };

// https://en.wikipedia.org/wiki/Xorshift
// 注意：不可多线程使用，state不可设为0

static inline float __host_uint_as_float(uint u) {
  union {
    uint u;
    float f;
  } x{.u = u};
  return x.f;
}

struct Rng64 {
  uint64_t state;
  // 生成随机的32位无符号整数
  __host__ __device__ uint32_t u32() {
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return (state * 0x2545F4914F6CDD1Dull) >> 32;
  }
  __host__ __device__ void SetSeed(uint64_t x) {
    assert(x);
    state = x;
  }
  // 生成[0,1)内的均匀随机数
  __host__ __device__ float Rand() {
#ifdef __CUDA_ARCH__
    return __uint_as_float(u32() >> 9 | 0x3f800000) - 1.f;
#else
    return __host_uint_as_float(u32() >> 9 | 0x3f800000) - 1.f;
#endif
  }
  // 以p的概率返回true
  __host__ __device__ float PTrue(float p) { return Rand() < p; }
  // 返回[0,n)间的准均匀随机数
  __host__ __device__ int RandInt(int n) { return u32() % n; }
  // __host__ __device__ int RandIntP(const float* p, uint n) {
  //   uint i = 0;
  //   float x = p[0], r = Rand();
  //   while (x <= r && i + 1 < n) {
  //     x += p[++i];
  //   }
  //   return i;
  // }
  __host__ __device__ int RandIntCDF(const float* cdf, uint n) {
    auto r = Rand();
    uint i = 0, j = n - 1;
    for (; i < j; ++i) {
      if (r <= cdf[i]) return i;
    }
    return j;
  }
  template <class T>
  __host__ __device__ auto&& Choose(T& arr) {
    return arr[RandInt(arr.size)];
  }
};
}  // namespace simulet::rand

#endif
