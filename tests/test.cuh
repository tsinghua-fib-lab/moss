#ifndef TESTS_TEST_CUH_
#define TESTS_TEST_CUH_

__device__ float rand(float x) {
  x = sinf(x) * 31459.26535;
  return x - floorf(x);
}

#endif