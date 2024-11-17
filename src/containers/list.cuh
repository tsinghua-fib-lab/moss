// Trimmed down version of
// https://www.cse.iitk.ac.in/users/mainakc/lockfree.html/
#ifndef SRC_CONTAINERS_LINKEDLIST_CUH_
#define SRC_CONTAINERS_LINKEDLIST_CUH_

#include <cuda.h>

template <class T>
struct DNode {
  T* data;
  DNode* next;

  __device__ bool CompareAndSet(DNode* expectedRef, DNode* newRef) {
    auto oldValOut =
        atomicCAS(reinterpret_cast<unsigned long long*>(&next),
                  reinterpret_cast<unsigned long long>(expectedRef),
                  reinterpret_cast<unsigned long long>(newRef));
    if (oldValOut == reinterpret_cast<unsigned long long>(expectedRef)) {
      return true;
    }
    return false;
  }
};

template <class T>
struct DList {
  DNode<T> _;
  DNode<T>* head;

  void Init() {
    _.data = nullptr;
    _.next = nullptr;
    head = &_;
  }

  __device__ void Add(DNode<T>* pointer) {
    while (true) {
      auto* pred = head;
      auto* curr = pred->next;
      pointer->next = curr;
      bool test = pred->CompareAndSet(curr, pointer);
      if (test) {
        return;
      }
    }
  }

  inline __device__ void Clear() {
    for (auto* p = head; p;) {
      auto* next = p->next;
      p->next = nullptr;
      p = next;
    }
  }

  __host__ __device__ T* begin() { return head->next->data; }
  __host__ __device__ T* end() { return nullptr; }
  __host__ __device__ const T* begin() const { return head->next->data; }
  __host__ __device__ const T* end() const { return nullptr; }
  __host__ __device__ operator bool() const { return head->next != nullptr; }

  void Save(std::vector<T*>& out) const {
    auto* curr = head->next;
    while (curr) {
      out.push_back(curr->data);
      curr = curr->next;
    }
  }

  void Load(const std::vector<T*>& in) {
    // TODO
    throw std::runtime_error("Not implemented");
  }
};

#endif
