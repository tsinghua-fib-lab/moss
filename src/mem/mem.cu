#include <cuda.h>
#include <cstdint>
#include <mutex>
#include "mem/mem.cuh"
#include "utils/macro.h"

unsigned long long operator"" _GiB(unsigned long long x) { return x << 30; }

unsigned long long operator"" _MiB(unsigned long long x) { return x << 20; }

// create a new block
int MemManager::new_block(void* ptr, size_t size) {
  if (empty_indices.empty()) {
    blocks.emplace_back(ptr, size, blocks.size());
    return blocks.size() - 1;
  } else {
    int i = empty_indices.front();
    empty_indices.pop();
    blocks[i] = MemBlock(ptr, size, i);
    return i;
  }
}

// find a free block that can hold the memory of size
int MemManager::find_fit(size_t size) {
  // if (verbose) {
  //   Info("start find_fit size: ", size);
  // }
  if (next_search_i == -1) {
    next_search_i = 0;
  }
  for (int i = next_search_i; i < blocks.size(); ++i) {
    auto& block = blocks[i];
    if (!block.Empty() && block.free && block.size >= size) {
      next_search_i = i;
      return i;
    }
  }
  for (int i = 0; i < next_search_i; ++i) {
    auto& block = blocks[i];
    if (!block.Empty() && block.free && block.size >= size) {
      next_search_i = i;
      return i;
    }
  }
  return -1;
}

// coalesce adjacent free blocks
void MemManager::coalesce(int i) {
  // coalesce previous block
  auto& block = blocks[i];
  if (block.prev_i != -1 && blocks[block.prev_i].free) {
    auto& prev = blocks[block.prev_i];
    prev.size += block.size;
    prev.next_i = block.next_i;
    if (block.next_i != -1) {
      blocks[block.next_i].prev_i = block.prev_i;
    }
    next_search_i = block.prev_i;  // reset next_search_i
    block = MemBlock{nullptr, 0, -1};
    empty_indices.push(i);
  }
  // coalesce next block
  auto next_i = block.next_i;
  if (next_i != -1 && blocks[next_i].free) {
    auto& next = blocks[next_i];
    block.size += next.size;
    block.next_i = next.next_i;
    if (next.next_i != -1) {
      blocks[next.next_i].prev_i = i;
    }
    next_search_i = i;  // reset next_search_i
    next = MemBlock{nullptr, 0, -1};
    empty_indices.push(next_i);
  }
}

// allocate memory
void* MemManager::allocate(size_t size) {
  std::lock_guard<std::mutex> lock(mtx);

  // align to 256 bytes
  size = ((size + ALIGN_MASK) & ~ALIGN_MASK);

  // find a free block that can hold the memory of size
  int fit_i = find_fit(size);
  if (fit_i == -1) {
    // no free block can hold the memory, allocate a new block
    size_t alloc_size =
        size < DEFAULT_ALLOCATE_SIZE ? DEFAULT_ALLOCATE_SIZE : size;
    void* ptr;
    CUCHECK(cudaMallocManaged(&ptr, alloc_size));
    if (!ptr) {
      Fatal("cudaMallocManaged failed");
      return nullptr;  // allocation failed
    }
    raw_ptrs.emplace_back(ptr, alloc_size);
    // prefer CPU
    CUCHECK(cudaMemAdvise(ptr, alloc_size, cudaMemAdviseSetPreferredLocation,
                          cudaCpuDeviceId));
    if (verbose) {
      Info("MemManager: allocate size: ", size,
           "B; alloc_size from cuda: ", double(alloc_size) / 1024 / 1024, "MiB");
    }
    fit_i = new_block(ptr, alloc_size);
  }
  auto block = &blocks[fit_i];
  // split the block if the remaining space is larger than MIN_BLOCK_SIZE
  if (block->size - size >= MIN_BLOCK_SIZE) {
    // create a new block with the remaining space
    int block2_i =
        new_block(static_cast<char*>(block->ptr) + size, block->size - size);
    block = &blocks[fit_i];  // reassigned after new_block to avoid invalid
                             // reference
    auto block2 = &blocks[block2_i];
    block2->prev_i = block->self_i;
    block2->next_i = block->next_i;
    if (block->next_i != -1) {
      blocks[block->next_i].prev_i = block2->self_i;
    }
    block->next_i = block2->self_i;
    next_search_i = block2->self_i;
    // update the current block size
    block->size = size;
  }
  blocks[fit_i].free = false;
  auto ptr = blocks[fit_i].ptr;
  ptr2i[ptr] = fit_i;
  return ptr;
}

// deallocate memory
void MemManager::deallocate(void* ptr) {
  std::lock_guard<std::mutex> lock(mtx);
  // find the block that contains the ptr
  auto it = ptr2i.find(ptr);
  if (it == ptr2i.end()) {
    Fatal("double free or corruption, not found");
    return;
  }
  int i = it->second;
  auto& block = blocks[i];
  if (block.Empty()) {
    Fatal("double free or corruption, empty block");
    return;
  }
  if (block.ptr != ptr) {
    Fatal("double free or corruption, ptr mismatch");
    return;
  }
  if (block.free) {
    Fatal("double free or corruption, already free");
    return;
  }
  // mark the block as free and try to coalesce adjacent blocks
  block.free = true;
  coalesce(i);
  ptr2i.erase(it);
}

void MemManager::PreferCPU() {
  for (auto& block : blocks) {
    CUCHECK(cudaMemAdvise(block.ptr, block.size,
                          cudaMemAdviseSetPreferredLocation, cudaCpuDeviceId));
  }
}

void MemManager::PreferGPU() {
  for (auto& block : blocks) {
    CUCHECK(cudaMemAdvise(block.ptr, block.size,
                          cudaMemAdviseUnsetPreferredLocation,
                          cudaCpuDeviceId));
    CUCHECK(cudaMemAdvise(block.ptr, block.size,
                          cudaMemAdviseSetPreferredLocation, device));
  }
}

void MemManager::PrintUsage() {
  if (verbose) {
    size_t total_size = 0;
    size_t used_size = 0;
    for (auto& block : blocks) {
      total_size += block.size;
      if (!block.free) {
        used_size += block.size;
      }
    }
    Info("Memory usage: ", double(used_size) / 1024 / 1024, "MiB/",
         double(total_size) / 1024 / 1024, "MiB (",
         100.0 * used_size / total_size, "%)");
  }
}

// save the whole memory to a cpu memory
// return a id for this memory checkpoint for further restore
int MemManager::Save() {
  std::lock_guard<std::mutex> lock(mtx);

  Checkpoint checkpoint;
  checkpoint.blocks = blocks;
  for (auto& raw : raw_ptrs) {
    char* data = new char[raw.size];
    CUCHECK(cudaMemcpy(data, raw.ptr, raw.size, cudaMemcpyDeviceToHost));
    checkpoint.data.push_back(data);
  }
  checkpoints.push_back(std::move(checkpoint));
  return checkpoints.size() - 1;
}

// restore the memory checkpoint by id
void MemManager::Restore(int id) {
  std::lock_guard<std::mutex> lock(mtx);

  if (id < 0 || id >= checkpoints.size()) {
    Fatal("Restore: invalid checkpoint id: ", id);
  }
  // copy data back to device
  for (size_t i = 0; i < checkpoints[id].data.size(); i++) {
    CUCHECK(cudaMemcpy(raw_ptrs[i].ptr, checkpoints[id].data[i], raw_ptrs[i].size,
                       cudaMemcpyHostToDevice));
  }
  // restore blocks
  blocks = checkpoints[id].blocks;
  // restore ptr2i and empty_indices
  ptr2i.clear();
  empty_indices = std::queue<int>();
  for (size_t i = 0; i < blocks.size(); i++) {
    if (blocks[i].Empty()) {
      empty_indices.push(i);
    } else {
      if (blocks[i].free) {
        next_search_i = i;
      } else {
        ptr2i[blocks[i].ptr] = i;
      }
    }
  }
}
