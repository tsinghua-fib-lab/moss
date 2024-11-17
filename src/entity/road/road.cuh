#ifndef SRC_ENTITY_ROAD_ROAD_CUH_
#define SRC_ENTITY_ROAD_ROAD_CUH_

#include "containers/array.cuh"
#include "fmt/core.h"
#include "protos.h"

namespace moss {

struct Lane;
struct Moss;

struct RoadCheckpoint {
  float max_speed;
  float v_avg;
  float status;
  uint nrl_a, nrl_b;
};

struct NextRoadLaneGroup {
  uint next_road_id;
  // range: [offset1, offset2]
  uint offset1, offset2;
};

struct Road {
  uint id, index;
  float max_speed;
  float v_avg;   // 平均车速
  float status;  // 拥堵程度，1~5表示逐渐拥堵
  MArrZ<Lane*> lanes;
  Lane* right_driving_lane;
  Lane* walking_lane;
  // 可以去往下一条road的车道信息
  // the road lanes that can go to the next road
  MArrZ<NextRoadLaneGroup> next_road_lane_groups;
  // 对于含有动态车道的road，会有多套nrl方案，每套方案对应上述数组的范围在此指定
  // for roads with dynamic lanes, there are multiple next road lane (nrl)
  // plans, each plan
  MArrZ<uint> nrl_ranges;
  // 当前启用的next_road_lanes范围，[a,b)
  // 这是为了于给普通道路代码提供fast-path
  // the current enabled next_road_lanes range, [a,b)
  uint nrl_a, nrl_b;
};

namespace road {
struct Data {
  MArrZ<Road> roads;
  std::unordered_map<uint, Road*> road_map;
  float k_status;  // 平滑系数

  void Init(Moss* S, const PbMap&);
  inline Road* At(uint id) {
    auto iter = road_map.find(id);
    if (iter == road_map.end()) {
      throw std::range_error(fmt::format("road {} not found", id));
    }
    return iter->second;
  }
};
}  // namespace road

}  // namespace moss

#endif
