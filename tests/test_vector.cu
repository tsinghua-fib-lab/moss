/**
 * @file main.cu
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date YYYY-MM-DD
 *
 * @copyright Copyright (c) 2021 Tsingroc
 *
 */

#include <cuda.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "mem/mem.cuh"
#include "test.cuh"
#include "utils/macro.h"

__global__ void f_1(DVector<int>* a) {
  auto id = threadIdx.x + blockIdx.x * blockDim.x;
  a->Append(id);
}

void test_1(MemManager* mem, uint grid_size, uint block_size) {
  Vector<int> a(mem);
  f_1<<<grid_size, block_size>>>(a.Cuda());
  cudaDeviceSynchronize();
  CUCHECK(cudaGetLastError());
  auto data = a.Cpu();
  assert(data.size() == grid_size * block_size);
  std::sort(data.begin(), data.end());
  int i = 0;
  for (auto j : data) {
    if (i != j) {
      printf("%d %d\n", i, j);
    }
    assert(i == j);
    ++i;
  }
  CUCHECK(cudaGetLastError());
  printf("%ld\n", data.size());
}

__global__ void f_2(DVector<int>* a, int* b, uint* b_size) {
  auto id = threadIdx.x + blockIdx.x * blockDim.x;
  // if (rand(id) < 0.8) {
  a->Append(id);
  b[atomicInc(b_size, ALL_BIT)] = id;
  // }
}

void test_2(MemManager* mem, uint grid_size, uint block_size) {
  Vector<int> a(mem);
  auto* b = mem->MArray<int>(grid_size * block_size);
  auto* b_size = mem->MValue<uint>();
  f_2<<<grid_size, block_size>>>(a.Cuda(), b, b_size);
  cudaDeviceSynchronize();
  CUCHECK(cudaGetLastError());
  auto data = a.Cpu();
  assert(data.size() == *b_size);
  std::sort(data.begin(), data.end());
  std::sort(b, b + *b_size);
  int i = 0;
  for (auto j : data) {
    assert(b[i++] == j);
  }
  printf("%ld %.4f\n", data.size(),
         (float)data.size() / (grid_size * block_size));
  CUCHECK(cudaGetLastError());
}

__global__ void f_3(DVector<int>* as, uint size) {
  uint id = THREAD_ID;
  as[id % size].Append(id);
}

void test_3(MemManager* mem, uint grid_size, uint block_size) {
  MArrZ<DVector<int>> as;
  as.New(mem, 37);
  f_3<<<grid_size, block_size>>>(as.data, as.size);
  CHECK;
}

int main() {
  auto* mem = MemManager::New(20_GiB, true);
  test_1(mem, 1, 32);
  test_1(mem, 1, 64);
  test_1(mem, 1, 1024);
  test_1(mem, 32, 1);
  test_1(mem, 64, 1);
  test_1(mem, 1024, 1);
  test_1(mem, 4343, 1);
  test_1(mem, 32, 32);
  test_1(mem, 43, 43);
  test_1(mem, 256, 256);
  CUCHECK(cudaGetLastError());

  test_2(mem, 1, 32);
  test_2(mem, 1, 64);
  test_2(mem, 1, 1024);
  test_2(mem, 32, 1);
  test_2(mem, 64, 1);
  test_2(mem, 1024, 1);
  test_2(mem, 4343, 1);
  test_2(mem, 32, 32);
  test_2(mem, 43, 43);
  test_2(mem, 256, 256);
  CUCHECK(cudaGetLastError());

  test_3(mem, 270, 1024);
  CHECK;

  printf("All tests passed!\n");
  return 0;
}
