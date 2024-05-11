#ifndef _SRC_UTILS_BARRIER_
#define _SRC_UTILS_BARRIER_

#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

class Barrier {
 private:
  size_t total;    // 线程总数
  size_t waiting;  // 正在等待的线程数
  size_t round;    // barrier轮数
  std::mutex mtx;
  std::condition_variable cv;

 public:
  Barrier() = default;
  // disable copying of barrier
  Barrier(const Barrier&) = delete;
  Barrier& operator=(const Barrier&) = delete;

  void set(size_t total) {
    if (total == 0) {
      throw std::invalid_argument("Barrier thread count cannot be 0");
    }
    this->total = total;
    waiting = round = 0;
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mtx);
    std::size_t now = round;  // 记录当前轮数，这样直到下一轮才继续
    if (++waiting == total) {
      waiting = 0;
      // 进入下一轮
      ++round;
      cv.notify_all();
    } else {
      cv.wait(lock, [this, &now]() { return round != now; });
    }
  }
};

#endif
