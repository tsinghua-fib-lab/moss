#include "vector.cuh"

__device__ void Lock(Mutex& m) {
  while (atomicCAS(&m, 0, 1))
    ;
}
__device__ void Unlock(Mutex& m) { atomicExch(&m, 0); }
