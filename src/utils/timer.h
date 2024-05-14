#ifndef SRC_UTILS_TIMER_H_
#define SRC_UTILS_TIMER_H_

#include <algorithm>
#include <chrono>
#include <vector>

uint64_t Time();

class Timer {
  int cnt = 0;
  bool full = false;
  std::vector<double> ts;

 public:
  explicit Timer(int n) { ts.resize(n); }
  double Tic() {
    auto old = ts[cnt];
    auto t = ts[cnt] = Time();
    ++cnt;
    if (cnt == ts.size()) {
      if (!full) {
        return full = true, cnt = 0, (t - ts[0]) / (ts.size() - 1);
      }
      cnt = 0;
    }
    if (full) {
      return (t - old) / ts.size();
    }
    return (t - ts[0]) / std::max(1, cnt - 1);
  }
};

#endif
