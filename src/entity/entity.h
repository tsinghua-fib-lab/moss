#ifndef SRC_ENTITY_ENTITY_H_
#define SRC_ENTITY_ENTITY_H_

#include "utils/macro.h"

namespace simulet {
struct Position {
  bool is_aoi;
  uint id;
  float s;
};
}  // namespace simulet

#endif
