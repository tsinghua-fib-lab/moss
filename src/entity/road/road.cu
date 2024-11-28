#include <stdexcept>
#include "entity/lane/lane.cuh"
#include "entity/road/road.cuh"
#include "moss.cuh"
#include "utils/color_print.h"

namespace moss::road {
void Data::Init(Moss* S, const PbMap& map) {
  roads.New(S->mem, map.roads_size());
  // 建立映射
  {
    auto* p = roads.data;
    for (auto& pb : map.roads()) {
      road_map[pb.id()] = p++;
    }
  }
  uint index = 0;
  for (auto& pb : map.roads()) {
    auto& r = roads[index];
    // 基本属性
    r.id = pb.id();
    r.index = index++;
    r.status = 1;
    r.lanes.New(S->mem, pb.lane_ids_size());
    {
      double s = 0;
      uint index = 0;
      for (auto& i : pb.lane_ids()) {
        auto* l = r.lanes[index] = S->lane.At(i);
        l->parent_is_road = true;
        l->offset_on_road = index;
        s += l->max_speed;
        l->parent_road = &r;
        if (l->type == LaneType::LANE_TYPE_DRIVING) {
          r.right_driving_lane = l;
        } else if (l->type == LaneType::LANE_TYPE_WALKING) {
          r.walking_lane = l;
        }
        ++index;
      }
      r.max_speed = r.v_avg = s / r.lanes.size;
    }
    // init side_lanes
    Lane* last = nullptr;
    for (auto* l : r.lanes) {
      if (l->type != LaneType::LANE_TYPE_DRIVING) {
        // scan into the walking lane, stop
        break;
      }
      l->side_lanes[LEFT] = last;
      if (last) {
        last->side_lanes[RIGHT] = l;
      }
      last = l;
    }
  }
  // next_road_lane
  bool bad_nrl = false;
  index = 0;
  for (auto& pb : map.roads()) {
    auto& r = roads[index++];
    if (pb.next_road_lane_plans_size()) {
      r.nrl_ranges.New(S->mem, 2 * pb.next_road_lane_plans_size());
      int j = 0, c = 0;
      for (auto& pb : pb.next_road_lane_plans()) {
        r.nrl_ranges[j] = c;
        r.nrl_ranges[j + 1] = (c += pb.next_road_lanes_size());
        j += 2;
      }
      r.next_road_lane_groups.New(S->mem, c);
      j = 0;
      for (auto& pb : pb.next_road_lane_plans()) {
        for (auto& pb : pb.next_road_lanes()) {
          r.next_road_lane_groups[j++] = {
              .next_road_id = unsigned(pb.road_id()),
              .offset1 = S->lane.At(pb.lane_id_a())->offset_on_road,
              .offset2 = S->lane.At(pb.lane_id_b())->offset_on_road,
          };
        }
      }
      r.nrl_a = 0;
      r.nrl_b = r.nrl_ranges[1];
    } else {
      std::map<uint, std::set<Lane*>> nrl;
      for (auto* l : r.lanes) {
        if (l->type == LaneType::LANE_TYPE_DRIVING) {
          for (auto& s : l->successors) {
            nrl[s.lane->successor->parent_id].insert(l);
          }
        }
      }
      r.next_road_lane_groups.New(S->mem, nrl.size());
      r.nrl_a = 0;
      r.nrl_b = r.next_road_lane_groups.size;
      int i = 0;
      for (auto&& [rid, ls] : nrl) {
        auto& x = r.next_road_lane_groups[i++];
        assert(ls.size());
        // offset range: [a, b]
        uint a, b;
        a = b = (*ls.begin())->offset_on_road;
        for (auto& i : ls) {
          auto offset = i->offset_on_road;
          a = min(a, offset);
          b = max(b, offset);
        }
        if ((b - a) != ls.size() - 1) {
          Warn("Only continuous next road plans are supported. Road ", r.id,
               "'s successor lanes do not appear to be in continuous order, "
               "please check the data for correctness.");
          bad_nrl = true;
        }
        x.next_road_id = rid;
        x.offset1 = a;
        x.offset2 = b;
      }
    }
  }
  if (bad_nrl) {
    Warn(
        "Some road's successor lanes do not appear to be in continuous order, "
        "please check the data for correctness.");
    Warn(
        "Explain: if a road has 4 lanes [marked as 1,2,3,4], if lane 1's "
        "successor connects to road A and lane 3's successor connects to road "
        "A, but lane 2's successor connects to road B, then the data is "
        "incorrect.");
    throw std::range_error("Only continuous next road plans are supported.");
  }
}

};  // namespace moss::road
