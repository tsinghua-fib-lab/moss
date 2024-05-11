#include "utils/color_print.h"
#include <chrono>
#include <ratio>

namespace color_print {
bool enable = true;
}
void _print_time() {
  uint64_t _t = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
  std::time_t t = _t / 1000'000;
  std::tm tm = *std::localtime(&t);
  // %F->%Y-%m-%d  %T->%H:%M:%S
  std::cout << std::put_time(&tm, "%F %T")
            << fmt::format(".{:06d}", _t % 1000'000);
}
