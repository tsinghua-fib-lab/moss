#ifndef _SRC_UTILS_COLOR_PRINT_H_
#define _SRC_UTILS_COLOR_PRINT_H_

#include <cassert>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>
#include "fmt/core.h"

namespace color_print {
extern bool enable;
}
template <class T>
void _print(const T& t) {
  std::cout << t << "\033[0m";
}

template <class First, class... Rest>
void _print(const First& first, Rest&&... rest) {
  std::cout << first;
  _print(std::forward<Rest>(rest)...);
}

void _print_time();

// 格式代码参考
//          foreground background
// black        30         40
// red          31         41
// green        32         42
// yellow       33         43
// blue         34         44
// magenta      35         45
// cyan         36         46
// white        37         47
//
// reset             0  (everything back to normal)
// bold/bright       1  (often a brighter shade of the same colour)
// underline         4
// inverse           7  (swap foreground and background colours)
// bold/bright off  21
// underline off    24
// inverse off      27

template <class... Args>
void Print(const std::string& format_code, Args&&... args) {
  std::cout << "\033[" << format_code << "m[";
  _print_time();
  std::cout << "] ";
  _print(std::forward<Args>(args)...);
}

template <class... Args>
void Fatal(Args&&... args) {
  Print("1;31", std::forward<Args>(args)...);
  std::cout << std::endl;
  assert(false);
  exit(-1);
}

template <class... Args>
void Info(Args&&... args) {
  if (!color_print::enable) return;
  Print("32", std::forward<Args>(args)...);
  std::cout << std::endl;
}

template <class... Args>
void Warn(Args&&... args) {
  Print("1;31", std::forward<Args>(args)...);
  std::cout << std::endl;
}

#endif
