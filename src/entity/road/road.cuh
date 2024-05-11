#ifndef SRC_ENTITY_ROAD_ROAD_CUH_
#define SRC_ENTITY_ROAD_ROAD_CUH_

#include "containers/array.cuh"
#include "protos.h"

namespace simulet {

struct Lane;
struct Simulet;

struct RoadCheckpoint {
  float max_speed;
  int v_speed_10000;
  uint v_cnt;
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
  int v_speed_10000;
  uint v_cnt;
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

  void Init(Simulet* S, const PbMap&);
  void Save(std::vector<RoadCheckpoint>&);
  void Load(const std::vector<RoadCheckpoint>&);
};
}  // namespace road

}  // namespace simulet

#endif
