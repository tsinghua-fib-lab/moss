#ifndef SRC_ENTITY_AOI_AOI_CUH_
#define SRC_ENTITY_AOI_AOI_CUH_

#include "fmt/core.h"
#include "containers/array.cuh"
#include "protos.h"

namespace moss {

struct Lane;
struct Moss;

struct AoiGate {
  Lane* lane;
  float s;
};

struct Aoi {
  uint id;
  MArr<AoiGate> walking_gates, driving_gates;
  // position
  float x, y;

  __host__ __device__ float GetDrivingS(Lane* lane);
  __host__ __device__ float GetWalkingS(Lane* lane);
};

namespace aoi {
struct Data {
  MArr<Aoi> aois;
  std::unordered_map<uint, Aoi*> aoi_map;
  Moss* S;

  void Init(Moss* S, const PbMap&);
  inline Aoi* At(uint id) {
    auto iter = aoi_map.find(id);
    if (iter == aoi_map.end()) {
      throw std::range_error(fmt::format("aoi {} not found", id));
    }
    return iter->second;
  }
};
}  // namespace aoi

}  // namespace moss

#endif
