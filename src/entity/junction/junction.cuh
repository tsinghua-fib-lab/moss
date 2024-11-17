#ifndef SRC_ENTITY_JUNCTION_JUNCTION_CUH_
#define SRC_ENTITY_JUNCTION_JUNCTION_CUH_

#include <vector>
#include "containers/array.cuh"
#include "entity/junction/trafficlight/trafficlight.cuh"
#include "entity/lane/lane.cuh"
#include "fmt/core.h"
#include "protos.h"

namespace moss {
struct Moss;

enum TlPolicy {
  // manual control, no auto switch
  MANUAL,
  // fixed time wheel rotation
  FIXED_TIME,
  // max pressure algorithm
  MAX_PRESSURE,
  // disabling traffic lights and vehicles can pass freely
  NONE,
};

struct Junction {
  uint id, index;
  // lanes in junction
  MArrZ<Lane*> lanes;
  // traffic light
  trafficlight::MPTrafficLight tl;
  // traffic light policy
  TlPolicy tl_policy;
  // phase switching duration
  int phase_time;
};

namespace junction {
struct Data {
  // data array: index -> junction
  MArrZ<Junction> junctions;
  // data map: id -> junction
  std::unordered_map<uint, Junction*> junction_map;
  Moss* S;
  // kernel stream
  cudaStream_t stream;
  // kernel launch grid size (g_)
  int g_prepare, g_update;
  // kernel launch block size (b_)
  int b_prepare, b_update;

  void Init(Moss* S, const PbMap&);
  inline Junction* At(uint id) {
    auto iter = junction_map.find(id);
    if (iter == junction_map.end()) {
      throw std::range_error(fmt::format("junction {} not found", id));
    }
    return iter->second;
  }
  void PrepareAsync();
  void UpdateAsync();
};
}  // namespace junction

}  // namespace moss

#endif
