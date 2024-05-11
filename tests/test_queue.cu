#include <cuda.h>
#include <stdio.h>
#include <random>
#include "containers/queue.cuh"
#include "utils/macro.h"
#include "utils/utils.cuh"

__global__ void f(DQueueNoLock<int>& a) {
  a.Push(1);
  a.Push(2);
  a.Push(3);
  a.Push(4);
  printf("os: %d %d\n", a.offset, a.size);
  printf("%d %d\n", a.Pop(), a.Pop());
  a.Push(5);
  a.Push(6);
  printf("os: %d %d\n", a.offset, a.size);
  a.Push(7);
  printf("os: %d %d\n", a.offset, a.size);
  printf("%d %d %d %d %d\n", a.Pop(), a.Pop(), a.Pop(), a.Pop(), a.Pop());
}

int main() {
  auto mem = MemManager::New(20_GiB, true);
  auto* a = mem->MValue<DQueueNoLock<int>>();
  a->mem = mem;
  f<<<1, 1>>>(*a);
  Sync();
  // CHECK;
  printf("All passed!\n");
}
