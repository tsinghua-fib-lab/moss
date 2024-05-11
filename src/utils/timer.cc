#include "utils/timer.h"

// 获取微秒时间戳
uint64_t Time() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
