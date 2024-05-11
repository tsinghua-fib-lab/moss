#ifndef SRC_ENTITY_LANE_LANE_CUH_
#define SRC_ENTITY_LANE_LANE_CUH_

#include <utility>
#include <vector>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "protos.h"
#include "utils/geometry.cuh"

namespace simulet {

struct Aoi;
struct Lane;
struct PersonNode;
struct Junction;
struct Road;
struct Simulet;

struct LaneConnection {
  uint type;
  Lane* lane;
};

// 在lane沿途设置多个观测点，lane在更新时会更新观测点位置前后的车辆信息
// 【前】指s>=observer，【后】指s<observer
struct LaneObservation {
  PersonNode *front, *back;
};

struct Overlap {
  // 是否本车道优先
  bool self_first;
  // 冲突车道位置
  float other_s;
  // 冲突车道指针
  Lane* other;
};

struct LaneCheckpoint {
  bool restriction;
  PersonNode *ped_head, *veh_head;
  LightState light_state;
  float light_time;
  uint ped_cnt, veh_cnt;
  float pressure_in, pressure_out;
  std::vector<LaneObservation> observations;
  std::vector<PersonNode*> ped_add_buffer, veh_add_buffer, ped_remove_buffer,
      veh_remove_buffer;
};

struct Lane {
  uint id;
  uint index;  // 在数组中的下标
  uint type;
  uint turn;
  uint parent_id;
  uint next_road_id;  // 后面连接的道路ID；仅对交叉口内部车道有效
  bool parent_is_road;
  bool need_side_update;  // 是否需要做支链更新
  bool need_output;       // 是否需要输出
  float length;
  float center_x, center_y;  // 估计的中心，用于输出
  float max_speed;
  float width;
  bool restriction, in_restriction;  // 限行状态
  MArrZ<Point> line;
  // 长度的累计和，从0开始，n+1项
  MArrZ<float> line_lengths;
  // 每一段的方向，n项
  MArrZ<float> line_directions;
  // 前驱后继
  MArrZ<LaneConnection> predecessors, successors;
  // 首个前驱后继
  Lane *predecessor, *successor;
  Lane* side_lanes[2];
  // 观测点位置(需要确保有序)和结果
  MArrZ<float> observers;
  MArrZ<LaneObservation> observations;
  // 人车链表头
  PersonNode *ped_head, *veh_head;
  DVector<PersonNode*> ped_add_buffer, veh_add_buffer;
  DVector<PersonNode*> ped_remove_buffer, veh_remove_buffer;
  LightState light_state;
  float light_time;
  float pressure_in, pressure_out;
  uint ped_cnt, veh_cnt;
  // overlap
  MArrZ<Overlap> overlaps;
  union {
    Junction* parent_junction;
    Road* parent_road;
  };

  __device__ bool IsNoEntry();
  __device__ __host__ void GetPosition(float s, float& x, float& y);
  __device__ void GetPositionDir(float s, float& x, float& y, float& dir);
  std::tuple<double, double, double> GetPositionDir(float s);
};

namespace lane {
using Type = PbLaneType;
struct Data {
  MArrZ<Lane> lanes;
  MArrZ<Lane*> output_lanes;
  std::unordered_map<uint, Lane*> lane_map;
  cudaStream_t stream;
  Simulet* S;
  int g_prepare0, g_prepare1, g_prepare2, g_update_tl, g_update_rs, g_update_ls;
  int b_prepare0, b_prepare1, b_prepare2, b_update_tl, b_update_rs, b_update_ls;

  void Init(Simulet* S, const PbMap&);
  void PrepareAsync();
  void UpdateAsync();
  void Save(std::vector<LaneCheckpoint>&);
  void Load(const std::vector<LaneCheckpoint>&);
};
}  // namespace lane

}  // namespace simulet

#endif
