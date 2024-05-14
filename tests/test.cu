#include <cuda.h>
#include <stdio.h>
#include "utils/macro.h"
#include "utils/utils.cuh"

__global__ void f(int* a, int n) {
  auto tid = THREAD_ID;
  if (tid < n) {
    a[tid] = tid;
  }
}

int main() {
  auto a = MArray<int>(100);
  // f<<<2, 64>>>(a, 100);
  Launch(100, 0, f, a, 100);
  Sync();
  for (int i = 0; i < 10; ++i) {
    printf("%d ", a[i]);
  }
  printf("\n");
}
