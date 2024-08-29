#include <stdexcept>
#include "entity/lane/lane.cuh"
#include "entity/road/road.cuh"
#include "moss.cuh"

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
        auto* l = r.lanes[index++] = S->lane.At(i);
        l->parent_is_road = true;
        // 除了0号车道外都需要更新支链
        l->need_side_update = index != 1;
        s += l->max_speed;
        l->parent_road = &r;
        if (l->type == LaneType::LANE_TYPE_DRIVING) {
          r.right_driving_lane = l;
        }
      }
      r.max_speed = r.v_avg = s / r.lanes.size;
    }
    // 填充side_lanes
    Lane* last = nullptr;
    for (auto& l : r.lanes) {
      l->side_lanes[0] = last;
      if (last) {
        last->side_lanes[1] = l;
      }
      last = l;
    }
  }
  // next_road_lane
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
      r.next_road_lanes.New(S->mem, c);
      j = 0;
      for (auto& pb : pb.next_road_lane_plans()) {
        for (auto& pb : pb.next_road_lanes()) {
          r.next_road_lanes[j++] = {
              .id = unsigned(pb.road_id()),
              .l1 = S->lane.At(pb.lane_id_a()),
              .l2 = S->lane.At(pb.lane_id_b()) + 1};
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
      r.next_road_lanes.New(S->mem, nrl.size());
      r.nrl_a = 0;
      r.nrl_b = r.next_road_lanes.size;
      int i = 0;
      for (auto&& [rid, ls] : nrl) {
        auto& x = r.next_road_lanes[i++];
        assert(ls.size());
        Lane *a, *b;
        a = b = *ls.begin();
        for (auto& i : ls) {
          a = min(a, i);
          b = max(b, i);
        }
        if ((b - a) != ls.size() - 1) {
          throw std::range_error(
              "Only continuous next road plans are supported.");
        }
        x.id = rid;
        x.l1 = a;
        x.l2 = b + 1;
      }
    }
  }
}

void Data::Save(std::vector<RoadCheckpoint>& state) {
  state.resize(roads.size);
  for (int i = 0; i < roads.size; ++i) {
    auto& s = state[i];
    auto& r = roads[i];
    s.max_speed = r.max_speed;
    s.v_avg = r.v_avg;
    s.status = r.status;
    s.nrl_a = r.nrl_a;
    s.nrl_b = r.nrl_b;
  }
}

void Data::Load(const std::vector<RoadCheckpoint>& state) {
  assert(roads.size == state.size());
  for (int i = 0; i < roads.size; ++i) {
    auto& s = state[i];
    auto& r = roads[i];
    r.max_speed = s.max_speed;
    r.v_avg = s.v_avg;
    r.status = s.status;
    r.nrl_a = s.nrl_a;
    r.nrl_b = s.nrl_b;
  }
}
};  // namespace moss::road
