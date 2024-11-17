#include <algorithm>
#include <unordered_map>
#include <utility>
#include "entity/aoi/aoi.cuh"
#include "entity/lane/lane.cuh"
#include "entity/person/person.cuh"
#include "moss.cuh"
#include "protos.h"
#include "utils/color_print.h"
#include "utils/debug.cuh"
#include "utils/macro.h"
#include "utils/utils.cuh"

namespace moss {

__host__ __device__ float Aoi::GetDrivingS(Lane* lane) {
  for (auto& g : driving_gates) {
    if (g.lane == lane) {
      return g.s;
    }
  }
  printf(RED("[Error] Lane[%d] not found in aoi[%d]'s driving gates\n"),
         lane->id, id);
  assert(false);
}

__host__ __device__ float Aoi::GetWalkingS(Lane* lane) {
  for (auto& g : walking_gates) {
    if (g.lane == lane) {
      return g.s;
    }
  }
  printf(RED("[Error] Lane[%d] not found in aoi[%d]'s walking gates\n"),
         lane->id, id);
  assert(false);
}

namespace aoi {
void Data::Init(Moss* S, const PbMap& map) {
  this->S = S;
  aois.New(S->mem, map.aois_size());
  // init aoi_map
  {
    auto* p = aois.data;
    for (auto& pb : map.aois()) {
      aoi_map[pb.id()] = p++;
    }
  }
  uint index = 0;
  for (auto& pb : map.aois()) {
    auto& a = aois[index++];
    a.id = pb.id();
    // walking gates
    a.walking_gates.New(S->mem, pb.walking_positions_size());
    {
      uint index = 0;
      for (auto& p : pb.walking_positions()) {
        auto& o = a.walking_gates[index];
        o.lane = S->lane.At(p.lane_id());
        o.s = Clamp<float>(p.s(), 0.f, o.lane->length);
        ++index;
      }
    }
    // driving gates
    a.driving_gates.New(S->mem, pb.driving_positions_size());
    {
      uint index = 0;
      for (auto& p : pb.driving_positions()) {
        auto& o = a.driving_gates[index];
        o.lane = S->lane.At(p.lane_id());
        // trick: the max s is length - 1 to keep space for lane changing.
        o.s = Clamp<float>(p.s(), 0.f, o.lane->length - 1);
        ++index;
      }
    }
    // center position
    a.x = 0;
    a.y = 0;
    for (int i = 0; i < pb.positions_size(); ++i) {
      a.x += pb.positions(i).x();
      a.y += pb.positions(i).y();
    }
    a.x /= pb.positions_size();
    a.y /= pb.positions_size();
  }
}
}  // namespace aoi
}  // namespace moss
