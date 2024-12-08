#include <cmath>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "entity/junction/junction.cuh"
#include "entity/lane/lane.cuh"
#include "mem/mem.cuh"
#include "moss.cuh"
#include "protos.h"
#include "utils/color_print.h"
#include "utils/macro.h"
#include "utils/utils.cuh"

namespace moss::junction {

__global__ void Prepare(Junction* junctions, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& jc = junctions[id];
  trafficlight::Prepare(jc, jc.tl);
}

__global__ void Update(Junction* junctions, uint size, float step_interval,
                       float yellow_time, float phase_pressure_coeff) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& jc = junctions[id];
  trafficlight::Update(jc, jc.tl, step_interval, yellow_time,
                       phase_pressure_coeff);
}

void Data::Init(Moss* S, const PbMap& map) {
  this->S = S;
  stream = NewStream();
  junctions.New(S->mem, map.junctions_size());
  SetGridBlockSize(Prepare, junctions.size, S->sm_count, g_prepare, b_prepare);
  SetGridBlockSize(Update, junctions.size, S->sm_count, g_update, b_update);
  // create junction map: id -> junction
  {
    auto* p = junctions.data;
    for (auto& pb : map.junctions()) {
      junction_map[pb.id()] = p++;
    }
  }
  // create junctions
  uint index = 0;
  for (auto& pb : map.junctions()) {
    auto& j = junctions[index];
    // copy basic attributes
    j.id = pb.id();
    j.index = index++;
    j.lanes.New(S->mem, pb.lane_ids_size());
    {
      // build bidirectional mapping between lane and junction
      uint index = 0;
      for (auto& pb : pb.lane_ids()) {
        auto* l = j.lanes[index++] = S->lane.At(pb);
        l->parent_junction = &j;
        if (l->type == LaneType::LANE_TYPE_DRIVING) {
          // driving lane must have exactly a successor and a predecessor
          assert(l->predecessors.size == 1);
          assert(l->successors.size == 1);
        }
      }
    }
    // init traffic light
    j.tl_policy = TlPolicy::FIXED_TIME;
    j.tl.lane_pressure.New(S->mem, j.lanes.size);
    auto& phases = j.tl.phases;
    auto& phase_duration = j.tl.phase_duration;

    // TODO: get phase from avaiable phases (pb.phases()) instead of fixed program

    // warning for confused traffic light setting
    if (pb.phases_size() > 0 && !pb.has_fixed_program()) {
      Warn("junction ", j.id, " has phases but no fixed program, in MOSS, we now use the phases in fixed program as the candidate phases for max pressure algorithm, please check the input data");
    }

    if (pb.has_fixed_program()) {
      auto& tl = pb.fixed_program();
      if (tl.phases_size() > 0) {
        phases.New(S->mem, tl.phases_size());
        phase_duration.New(S->mem, tl.phases_size());
        auto n_lanes = tl.phases(0).states_size();
        int i = 0;
        for (auto& p : phases) {
          p.New(S->mem, n_lanes);
          auto& _p = tl.phases(i);
          phase_duration[i] = _p.duration();
          ++i;
          for (int j = 0; j < n_lanes; ++j) {
            p[j] = LightState(_p.states(j));
          }
        }
        j.tl.yellow_phases.New(S->mem, n_lanes);
        j.tl.ok = true;
        j.tl.set_force = true;
      }
      if (j.tl.phases.size > 0) {
        j.tl.phase_pressure_coeff.New(S->mem, j.tl.phases.size);
        j.tl.phase_pressure_coeff.Fill(1);
      }
    }
  }
  // if one of the junction's lanes are in the visualization area, add the whole
  // junction into the output lanes
  std::vector<Lane*> output_lanes;
  for (auto& j : junctions) {
    bool visible = false;
    for (auto* l : j.lanes) {
      if (S->config.x_min <= l->center_x && l->center_x <= S->config.x_max &&
          S->config.y_min <= l->center_y && l->center_y <= S->config.y_max) {
        visible = true;
        break;
      }
    }
    if (visible) {
      for (auto* l : j.lanes) {
        output_lanes.push_back(l);
      }
    }
  }
  std::sort(output_lanes.begin(), output_lanes.end());
  S->lane.output_lanes.New(S->mem, output_lanes.size());
  index = 0;
  for (auto* l : output_lanes) {
    S->lane.output_lanes[index++] = l;
  }
}

void Data::PrepareAsync() {
  if (junctions.size > 0) {
    Prepare<<<g_prepare, b_prepare, 0, stream>>>(junctions.data,
                                                 junctions.size);
  }
}

void Data::UpdateAsync() {
  if (junctions.size > 0) {
    Update<<<g_update, b_update, 0, stream>>>(
        junctions.data, junctions.size, S->config.step_interval,
        S->config.junction_yellow_time, S->config.phase_pressure_coeff);
  }
}

}  // namespace moss::junction
