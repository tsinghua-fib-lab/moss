#include <cmath>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "entity/entity.h"
#include "entity/junction/junction.cuh"
#include "entity/lane/lane.cuh"
#include "mem/mem.cuh"
#include "moss.cuh"
#include "output.cuh"
#include "protos.h"
#include "utils/color_print.h"
#include "utils/macro.h"
#include "utils/utils.cuh"

namespace moss::junction {
// 根据本车道s坐标计算切向角度
__device__ float GetDirectionByS(Lane* lane) {
  auto s = lane->length;
  if (s < lane->line_lengths[0] || s > lane->line_lengths.back()) {
    printf(RED("GetDirectionByS: s out of range\n"));
    s = Clamp(s, lane->line_lengths[0], lane->line_lengths.back());
  }
  if (s == lane->line_lengths[0]) {
    return lane->line_directions[0];
  }
  float direction;
  for (uint i = 1; i < lane->line_lengths.size; ++i) {
    if (s > lane->line_lengths[i - 1] && s <= lane->line_lengths[i]) {
      direction = lane->line_directions[i];
    }
  }
  return direction;
}

bool IsLeft(Lane* lane) {
  return lane->turn == LaneTurn::LANE_TURN_LEFT ||
         lane->turn == LaneTurn::LANE_TURN_AROUND;
}

bool EveryLeft(Lane* lane) {
  for (auto& suc : lane->successors) {
    // 每个后继都要是左转/掉头
    if (!IsLeft(suc.lane)) {
      return false;
    }
  }
  return true;
}

bool HasIndependentLeft(std::vector<Lane*>& lanes) {
  // 检查该group内是否有独立的左转车道，如果没有，则在这一group中不考虑对向直行和对向左转两个相位类型
  for (auto* lane : lanes) {
    if (!IsLeft(lane)) {
      continue;
    }
    // 对于左转车道，考察它的前驱的其他后继里面有没有非左转车道，如果有，则不是独立左转
    if (EveryLeft(lane->predecessor)) {
      return true;
    }
  }
  return false;
}

bool GroupContain(std::vector<Lane*>& group, Lane* lane) {
  for (auto& l : group) {
    if (l == lane) {
      return true;
    }
  }
  return false;
}

bool GroupStraight(std::vector<Lane*>& group, Lane* lane) {
  return lane->turn == LaneTurn::LANE_TURN_STRAIGHT &&
         GroupContain(group, lane);
}

bool GroupLeft(std::vector<Lane*>& group, Lane* lane) {
  return IsLeft(lane) && GroupContain(group, lane);
}

float GetEndDirection(Lane* l) { return l->line_directions.back(); }

template <class T>
void Add(std::vector<T>& a, const std::vector<T> b) {
  a.reserve(a.size() + b.size());
  a.insert(a.end(), b.begin(), b.end());
}

void GenerateAvailablePhases(Moss* S, Junction& jc) {
  // 按接入道路的角度将路口内的车道分为4类
  std::vector<Lane*> driving_lanes;
  for (auto* l : jc.lanes) {
    if (l->type == LaneType::LANE_TYPE_DRIVING) {
      driving_lanes.push_back(l);
    }
  }
  if (driving_lanes.empty()) {
    return;
  }

  float ref_angle = driving_lanes[0]->line_directions.back();
  // 准备道路角度数据
  // direction type []-> []lane*
  std::vector<Lane*> direction_groups[4];
  for (auto* l : driving_lanes) {
    // 0: -pi/4 ~ pi/4
    // 1: pi/4 ~ 3pi/4
    // 2: 3pi/4 ~ 5pi/4
    // 3: 5pi/4 ~ 7pi/4
    auto direction_type = int((l->predecessor->line_directions.back() -
                               ref_angle + PI * 2 + PI / 4) /
                              (PI / 2)) %
                          4;
    direction_groups[direction_type].push_back(l);
  }
  // 判断是否需要添加信控，3个以上进入的道路，需要信控
  uint count = 0;
  for (auto& group : direction_groups) {
    count += !group.empty();
  }
  if (count >= 3) {
    auto group0{direction_groups[0]};
    Add(group0, direction_groups[2]);
    auto group90{direction_groups[1]};
    Add(group90, direction_groups[3]);
    // 生成相位表
    std::vector<uint> is_green_funcs;
    // 将direction_groups[0]和direction_groups[2] 作为 group_0 0-180度对向
    if (HasIndependentLeft(group0)) {
      // 对向直行
      is_green_funcs.push_back(0);
      // 对向左转
      is_green_funcs.push_back(1);
    }
    // 将direction_groups[1]和direction_groups[3] 作为 group_90 90-270度对向
    if (HasIndependentLeft(group90)) {
      // 对向直行
      is_green_funcs.push_back(2);
      // 对向左转
      is_green_funcs.push_back(3);
    }
    for (uint i = 0; i < 4; i++) {
      if (direction_groups[i].empty()) {
        continue;
      }
      is_green_funcs.push_back(i + 4);
    }
    // 生成相位表
    std::unordered_map<uint, uint> id2index;
    uint n_lanes = 0;
    for (auto* l : jc.lanes) {
      id2index[l->id] = n_lanes++;
    }
    auto& phases = jc.tl.phases;
    phases.New(S->mem, is_green_funcs.size());
    for (auto& p : phases) {
      p.New(S->mem, n_lanes);
    }
    jc.tl.yellow_phases.New(S->mem, n_lanes);
    uint phase_index = 0;
    for (auto func_num : is_green_funcs) {
      auto& phase = phases[phase_index++];
      std::vector<Lane*> walking_lanes;
      int i = -1;
      for (auto* lane : jc.lanes) {
        ++i;
        if (lane->type == LaneType::LANE_TYPE_WALKING) {
          phase[i] = LightState::LIGHT_STATE_GREEN;  // 后续处理
          walking_lanes.push_back(lane);
          continue;
        }
        if (lane->turn == LaneTurn::LANE_TURN_RIGHT) {
          phase[i] = LightState::LIGHT_STATE_GREEN;
          continue;
        }
        // 由于 GroupStraight 和 GroupLeft 包含了
        // GroupContain，因此需要将group0、group90的两部分取或
        bool flag;
        switch (func_num) {
          case 0:
            flag = GroupStraight(group0, lane);
            break;
          case 1:
            flag = GroupLeft(group0, lane);
            break;
          case 2:
            flag = GroupStraight(group90, lane);
            break;
          case 3:
            flag = GroupLeft(group90, lane);
            break;
          case 4:
            flag = GroupContain(direction_groups[0], lane);
            break;
          case 5:
            flag = GroupContain(direction_groups[1], lane);
            break;
          case 6:
            flag = GroupContain(direction_groups[2], lane);
            break;
          case 7:
            flag = GroupContain(direction_groups[3], lane);
            break;
          default:;
        }
        phase[i] =
            flag ? LightState::LIGHT_STATE_GREEN : LightState::LIGHT_STATE_RED;
      }
      // 处理人行道
      for (auto* lane : walking_lanes) {
        auto index = id2index[lane->id];
        for (auto& overlap : lane->overlaps) {
          auto* other_lane = overlap.other;
          // 如果相交的车道是非右转的绿灯，则人行道是红灯
          if (other_lane->turn != LaneTurn::LANE_TURN_RIGHT &&
              phase[id2index[other_lane->id]] ==
                  LightState::LIGHT_STATE_GREEN) {
            phase[index] = LightState::LIGHT_STATE_RED;
            break;
          }
        }
      }
    }
  }
  return;
}

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
  // 建立映射
  {
    auto* p = junctions.data;
    for (auto& pb : map.junctions()) {
      junction_map[pb.id()] = p++;
    }
  }
  // TODO: 添加对driving_lane_groups的处理；现在的信控还是基于lane的
  uint index = 0;
  std::vector<Lane*> output_lanes;
  for (auto& pb : map.junctions()) {
    auto& j = junctions[index];
    // 基本属性
    j.id = pb.id();
    j.index = index++;
    j.lanes.New(S->mem, pb.lane_ids_size());
    {
      uint index = 0;
      for (auto& pb : pb.lane_ids()) {
        auto* l = j.lanes[index++] = S->lane.lane_map.at(pb);
        l->parent_junction = &j;
        if (l->type == LaneType::LANE_TYPE_DRIVING) {
          assert(l->predecessors.size == 1);
          assert(l->successors.size == 1);
          l->next_road_id = l->successor->parent_id;
        }
      }
    }
    if (S->config.enable_junction) {
      j.tl_policy = TlPolicy::FIXED_TIME;
      j.tl.lane_pressure.New(S->mem, j.lanes.size);
      if (S->is_python_api) {
        auto& phases = j.tl.phases;
        auto& phase_duration = j.tl.phase_duration;
        if (!pb.has_fixed_program()) {
          continue;
        }
        auto& tl = pb.fixed_program();
        if (tl.phases_size()) {
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
      } else {
        j.tl.ok = true;
        GenerateAvailablePhases(S, j);
      }
      if (j.tl.phases.size) {
        j.tl.phase_pressure_coeff.New(S->mem, j.tl.phases.size);
        j.tl.phase_pressure_coeff.Fill(1);
      }
    }
  }
  if (S->output.option == output::Option::AGENT) {
    for (auto& j : junctions) {
      double x = 0, y = 0;
      for (auto* l : j.lanes) {
        x += l->center_x;
        y += l->center_y;
      }
      x /= j.lanes.size;
      y /= j.lanes.size;
      if (S->output.X_MIN <= x && x <= S->output.X_MAX &&
          S->output.Y_MIN <= y && y <= S->output.Y_MAX) {
        for (auto* l : j.lanes) {
          output_lanes.push_back(l);
        }
      }
    }
    S->lane.output_lanes.New(S->mem, output_lanes.size());
    uint index = 0;
    for (auto* i : output_lanes) {
      S->lane.output_lanes[index++] = i;
    }
  }
}

void Data::PrepareAsync() {
  if (S->config.enable_junction && junctions.size) {
    Prepare<<<g_prepare, b_prepare, 0, stream>>>(junctions.data,
                                                 junctions.size);
  }
}

void Data::UpdateAsync() {
  if (S->config.enable_junction && junctions.size) {
    Update<<<g_update, b_update, 0, stream>>>(
        junctions.data, junctions.size, S->config.step_interval,
        S->config.junction_yellow_time, S->config.phase_pressure_coeff);
  }
}

void Data::Save(std::vector<JunctionCheckpoint>& state) {
  state.resize(junctions.size);
  for (int i = 0; i < junctions.size; ++i) {
    auto& j = junctions[i];
    auto& tl = j.tl;
    auto& s = state[i];
    s.tl_policy = j.tl_policy;
    s.ok = tl.ok;
    s.phase_index = tl.phase_index;
    tl.yellow_phases.Save(s.yellow_phases);
    s.is_yellow = tl.is_yellow;
    s.next_index = tl.next_index;
    tl.phase_duration.Save(s.phase_duration);
    tl.phase_pressure_coeff.Save(s.phase_pressure_coeff);
    s.runtime = tl.runtime;
    s.snapshot = tl.snapshot;
    s.set_force = tl.set_force;
  }
}

void Data::Load(const std::vector<JunctionCheckpoint>& state) {
  assert(state.size() == junctions.size);
  for (int i = 0; i < junctions.size; ++i) {
    auto& j = junctions[i];
    auto& tl = j.tl;
    auto& s = state[i];
    j.tl_policy = s.tl_policy;
    tl.ok = s.ok;
    tl.phase_index = s.phase_index;
    tl.yellow_phases.Load(s.yellow_phases);
    tl.is_yellow = s.is_yellow;
    tl.next_index = s.next_index;
    tl.phase_duration.Load(s.phase_duration);
    tl.phase_pressure_coeff.Load(s.phase_pressure_coeff);
    tl.runtime = s.runtime;
    tl.snapshot = s.snapshot;
    tl.set_force = s.set_force;
  }
}
}  // namespace moss::junction
