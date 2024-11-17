#ifndef SRC_RAND_RAND_CUH_
#define SRC_RAND_RAND_CUH_

#include <cuda.h>
#include <cassert>
#include <cstdint>
#include <type_traits>

namespace moss::rand {

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
// ATTENTION: Not thread-safe, state must not be 0

static inline float __host_uint_as_float(uint u) {
  union {
    uint u;
    float f;
  } x{.u = u};
  return x.f;
}

struct Rng64 {
  uint64_t state;
  // generate a random 32-bit unsigned integer
  __host__ __device__ uint32_t u32() {
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return (state * 0x2545F4914F6CDD1Dull) >> 32;
  }
  // set the seed
  __host__ __device__ void SetSeed(uint64_t x) {
    assert(x);
    state = x;
  }
  // generate a uniform random float in [0,1)
  __host__ __device__ float Rand() {
#ifdef __CUDA_ARCH__
    return __uint_as_float(u32() >> 9 | 0x3f800000) - 1.f;
#else
    return __host_uint_as_float(u32() >> 9 | 0x3f800000) - 1.f;
#endif
  }
  // return true with probability p
  __host__ __device__ float PTrue(float p) { return Rand() < p; }
  // return a uniform random integer in [0,n)
  __host__ __device__ int RandInt(int n) { return u32() % n; }
};
}  // namespace moss::rand

#endif
