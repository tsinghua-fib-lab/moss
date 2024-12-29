#ifndef SRC_MEM_MEM_CUH_
#define SRC_MEM_MEM_CUH_

#include <cuda.h>
#include <stdio.h>
#include <cassert>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>
#include "utils/color_print.h"
#include "utils/macro.h"

unsigned long long operator""_GiB(unsigned long long x);
unsigned long long operator""_MiB(unsigned long long x);

class RawPtr {
 public:
  void* ptr;
  size_t size;

  RawPtr(void* ptr, size_t size) : ptr(ptr), size(size) {}
};

class MemBlock {
 public:
  void*
      ptr;  // start address of memory block (nullptr means the block is empty)
  size_t size;  // memory block size
  bool free;    // whether the block is free
  int self_i;   // self index in the memory block vector
  int prev_i;   // previous block in the same cudaMallocManaged memory
  int next_i;   // next block in the same cudaMallocManaged memory

  MemBlock(void* p, size_t s, int self_i)
      : ptr(p), size(s), free(true), self_i(self_i), prev_i(-1), next_i(-1) {}

  inline bool Empty() { return ptr == nullptr; }
};

class MemManager {
  struct Checkpoint {
    std::vector<char*> data;       // data snapshot
    std::vector<MemBlock> blocks;  // block snapshot
  };

 private:
  static const size_t MIN_BLOCK_SIZE = 256;  // min block size：256B
  static const size_t DEFAULT_ALLOCATE_SIZE =
      1 << 26;                          // default allocate size：64MiB
  static const size_t ALIGNMENT = 256;  // memory alignment：256字节
  static const size_t ALIGN_MASK = ALIGNMENT - 1;

  uint device;
  bool verbose;

  std::vector<RawPtr> raw_ptrs;          // raw pointers for cudaMallocManaged
  std::vector<MemBlock> blocks;          // memory blocks
  std::unordered_map<void*, int> ptr2i;  // ptr to index map
  std::queue<int> empty_indices;         // empty indices for next "new" block

  std::mutex mtx;     // mutx for thread safety
  int next_search_i;  // last block iterator for quick search

  std::vector<Checkpoint> checkpoints;  // memory checkpoints

  // create a new block
  int new_block(void* ptr, size_t size);

  // find a free block that can hold the memory of size
  int find_fit(size_t size);

  // coalesce adjacent free blocks
  void coalesce(int i);

 public:
  MemManager(uint device, bool verbose, size_t reserve_space = 1024 * 1024)
      : device(device), verbose(verbose) {
    next_search_i = -1;
    blocks.reserve(reserve_space);
  }

  ~MemManager() {
    for (auto ptr : raw_ptrs) {
      cudaFree(ptr.ptr);
    }
  }

  // allocate memory
  void* allocate(size_t size);
  // deallocate memory
  void deallocate(void* ptr);
  void PreferCPU();
  void PreferGPU();
  void PrintUsage();

  template <class T>
  T* MValue(bool zero_init = true) {
    T* t = static_cast<T*>(allocate(sizeof(T)));
    if (zero_init) {
      CUCHECK(cudaMemset(t, 0, sizeof(T)));
    }
    return t;
  }

  template <class T>
  T* MArray(size_t size, bool zero_init = true) {
    T* t = static_cast<T*>(allocate(size * sizeof(T)));
    if (zero_init) {
      CUCHECK(cudaMemset(t, 0, size * sizeof(T)));
    }
    return t;
  }

  template <class T>
  void Free(T* ptr) {
    deallocate(ptr);
  }

  // save the whole memory to a cpu memory
  // return a id for this memory checkpoint for further restore
  int Save();

  // restore the memory checkpoint by id
  void Restore(int checkpoint_id);
};

#endif
