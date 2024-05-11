#include <cuda.h>
#include <stdio.h>
#include "mem/mem.cuh"
#include "utils/macro.h"
#include "utils/utils.cuh"

__global__ void f(uint* output, uint size) {
  uint a = 0;
  while (malloc(size)) {
    ++a;
  }
  output[THREAD_ID] = a;
}

int main(int argc, char** argv) {
  if (argc < 5) {
    return -1;
  }
  size_t limit;
  cudaDeviceSetLimit(cudaLimitMallocHeapSize, atoi(argv[4]) << 20);
  cudaDeviceGetLimit(&limit, cudaLimitMallocHeapSize);
  printf("%.3fGiB %ldB\n", double(limit) / (1 << 30), limit);

  uint grid = atoi(argv[1]), block = atoi(argv[2]), size = atoi(argv[3]);
  auto* output = MArrayZero<uint>(grid * block);
  f<<<grid, block>>>(output, size);
  Sync();
  CHECK;
  uint sum = 0;
  for (uint i = 0; i < grid * block; ++i) {
    sum += output[i];
  }
  printf("malloc: %d, size: %.3fMiB %dB\n", sum, double(sum * size) / (1 << 20),
         sum * size);
}
