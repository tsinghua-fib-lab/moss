#include <cuda.h>
#include <stdio.h>
#include <cassert>
#include <random>
#include "containers/vector.cuh"
#include "mem/mem.cuh"
#include "utils/macro.h"
#include "utils/utils.cuh"

__global__ void f(int* a, DVectorNoLock<Pair<int, int>>& b, int n) {
  for (uint i = 0; i < n; ++i) {
    HeapPush(b, {a[i], a[i]});
  }
  for (uint i = 0; i < n; ++i) {
    a[i] = HeapPop(b);
  }
}

int main() {
  const int n = 100;
  auto* mem = MemManager::New(20_GiB, true);
  auto* a = mem->MArray<int>(n);
  auto* b = mem->MValue<DVectorNoLock<Pair<int, int>>>();
  b->mem = mem;

  for (int t = 0; t < 1000; ++t) {
    for (uint i = 0, j; i < n; ++i) {
      j = std::rand() % (i + 1);
      a[i] = a[j];
      a[j] = i;
    }
    // for (int i = 0; i < n; ++i) {
    //   printf("%d ", a[i]);
    // }
    // printf("\n");
    f<<<1, 1>>>(a, *b, n);
    Sync();
    CHECK;
    // for (int i = 0; i < n; ++i) {
    //   printf("%d ", a[i]);
    // }
    // printf("\n");
    for (int i = 0; i < n; ++i) {
      assert(a[i] == i);
    }
  }
  printf("All passed!\n");
}
