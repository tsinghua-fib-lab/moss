#ifndef SRC_ENTITY_JUNCTION_JUNCTION_CUH_
#define SRC_ENTITY_JUNCTION_JUNCTION_CUH_

#include <vector>
#include "fmt/core.h"
#include "containers/array.cuh"
#include "entity/junction/trafficlight/trafficlight.cuh"
#include "entity/lane/lane.cuh"
#include "protos.h"

namespace moss {
struct Moss;

enum TlPolicy {
  MANUAL,        // 手动控制，不自动切换
  FIXED_TIME,    // 固定时间轮替
  MAX_PRESSURE,  // 最大压力
  NONE,          // 无信控
};

struct Junction {
  uint id, index;
  MArrZ<Lane*> lanes;               // 车道指针数组
  trafficlight::MPTrafficLight tl;  // 信号灯模块
  TlPolicy tl_policy;               // 信控方案
  int phase_time;                   // 相位切换时长
};

struct JunctionCheckpoint {
  TlPolicy tl_policy;
  bool ok;
  uint phase_index;
  std::vector<LightState> yellow_phases;
  bool is_yellow;
  uint next_index;
  std::vector<float> phase_duration, phase_pressure_coeff;
  trafficlight::MPTLRuntime snapshot, runtime;
  bool set_force;
};

namespace junction {
struct Data {
  MArrZ<Junction> junctions;
  std::unordered_map<uint, Junction*> junction_map;
  cudaStream_t stream;
  Moss* S;
  int g_prepare, g_update;
  int b_prepare, b_update;

  void Init(Moss* S, const PbMap&);
  inline Junction* At(uint id) {
    auto iter = junction_map.find(id);
    if (iter == junction_map.end()) {
      throw std::range_error(fmt::format("junction {} not found", id));
    }
    return iter->second;
  }
  void PrepareAsync();
  void UpdateAsync();
  void Save(std::vector<JunctionCheckpoint>&);
  void Load(const std::vector<JunctionCheckpoint>&);
};
}  // namespace junction

}  // namespace moss

#endif
