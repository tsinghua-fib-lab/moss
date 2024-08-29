#ifndef SRC_ENTITY_ROAD_ROAD_CUH_
#define SRC_ENTITY_ROAD_ROAD_CUH_

#include "fmt/core.h"
#include "containers/array.cuh"
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

struct NextRoadLane {
  uint id;
  Lane *l1, *l2;  // [l1, l2)
};

struct Road {
  uint id, index;
  float max_speed;
  float v_avg;   // 平均车速
  float status;  // 拥堵程度，1~5表示逐渐拥堵
  MArrZ<Lane*> lanes;
  Lane* right_driving_lane;
  // 可以去往下一条road的车道信息
  MArrZ<NextRoadLane> next_road_lanes;
  // 对于含有动态车道的road，会有多套nrl方案，每套方案对应上述数组的范围在此指定
  MArrZ<uint> nrl_ranges;
  // 当前启用的next_road_lanes范围，[a,b)
  // 这是为了于给普通道路代码提供fast-path
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
  void Save(std::vector<RoadCheckpoint>&);
  void Load(const std::vector<RoadCheckpoint>&);
};
}  // namespace road

}  // namespace moss

#endif
