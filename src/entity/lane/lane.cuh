#ifndef SRC_ENTITY_LANE_LANE_CUH_
#define SRC_ENTITY_LANE_LANE_CUH_

#include <utility>
#include <vector>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "containers/list.cuh"
#include "fmt/core.h"
#include "output/output.cuh"
#include "protos.h"
#include "utils/geometry.cuh"

namespace moss {

struct Aoi;
struct Lane;
struct PersonNode;
struct Junction;
struct Road;
struct Moss;

struct LaneConnection {
  uint type;
  Lane* lane;
};

struct Lane {
  // lane id
  uint id;
  // lane offset in container
  uint index;
  // lane type: 1 - driving, 2 - walking
  uint type;
  // lane turn: 1 - go straight, 2 - turn left, 3 - turn right, 4 - U-turn
  uint turn;
  // road / junction id
  uint parent_id;
  // parent road / junction pointer
  union {
    Junction* parent_junction;
    Road* parent_road;
  };
  // offset on road's lanes, the leftest lane is 0
  uint offset_on_road : 31;
  // helper to determine if the parent is road or junction
  bool parent_is_road : 1;
  // lane length
  float length;
  // lane width
  float width;
  // lane center coordinates
  float center_x, center_y;
  // lane max speed (m/s)
  float max_speed;
  // restriction status TODO: more details
  bool restriction, in_restriction;
  // geometry of the lane
  // point list
  MArrZ<Point> line;
  // accumulated length of each line segment
  MArrZ<float> line_lengths;
  // direction of each line segment
  MArrZ<float> line_directions;
  // lane connections
  MArrZ<LaneConnection> predecessors, successors;
  // the first predecessor and successor (only for junction lane)
  Lane *predecessor, *successor;
  // the left and right side lanes
  Lane* side_lanes[2];
  // linked list header for vehicle sensing and pedestrian management
  PersonNode *ped_head, *veh_head;
  // vehicle and pedestrian count
  uint ped_cnt, veh_cnt;
  // add buffer for vehicle and pedestrian TODO: lock-free linked list
  DList<PersonNode> ped_add_buffer, veh_add_buffer;
  // remove buffer for vehicle and pedestrian TODO: lock-free linked list
  DList<PersonNode> ped_remove_buffer, veh_remove_buffer;
  // traffic light status (only junction lane)
  LightState light_state;
  // the remaining time of the current light state (only junction lane)
  float light_time;
  // pressure in and out (only junction lane)
  float pressure_in, pressure_out;
  // average vehicle speed (m/s)
  float v_avg;

  __device__ bool IsNoEntry();
  __host__ __device__ void GetPosition(float s, float& x, float& y);
  __device__ void GetPositionDir(float s, float& x, float& y, float& dir);
  std::tuple<double, double, double> GetPositionDir(float s);
};

// 同一道路上两个车道间按照等比例方式进行投影
// project from src_lane to dest_lane in the same road, using an equal ratio
// way.
__host__ __device__ float ProjectFromLane(Lane* src_lane, Lane* dest_lane,
                                          float s);

namespace lane {
using Type = PbLaneType;
struct Data {
  MArrZ<Lane> lanes;
  MArrZ<Lane*> output_lanes;
  MArrZ<TlOutput> outputs;
  std::unordered_map<uint, Lane*> lane_map;
  cudaStream_t stream;
  Moss* S;
  // kernel grid size (g_)
  int g_prepare0, g_prepare1, g_prepare2, g_prepare_output, g_update_rs,
      g_update_ls;
  // kernel block size (b_)
  int b_prepare0, b_prepare1, b_prepare2, b_prepare_output, b_update_rs,
      b_update_ls;

  // init lane data
  void Init(Moss* S, const PbMap&);
  // init remaining data after junction and road initialization
  void InitSizes(Moss* S);
  inline Lane* At(uint id) {
    auto iter = lane_map.find(id);
    if (iter == lane_map.end()) {
      throw std::range_error(fmt::format("lane {} not found", id));
    }
    return iter->second;
  }
  // prepare person related data including linked list, add/remove buffer
  void PrepareAsync(cudaStream_t stream);
  // prepare output data
  void PrepareOutputAsync(cudaStream_t stream);
  void UpdateAsync();
};
}  // namespace lane

}  // namespace moss

#endif
