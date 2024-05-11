#ifndef SRC_ENTITY_AOI_AOI_CUH_
#define SRC_ENTITY_AOI_AOI_CUH_

#include <vector>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "entity/aoi/crowd/crowd.cuh"
#include "protos.h"
#include "utils/geometry.cuh"

namespace simulet {

struct Person;
struct Lane;
struct LaneObservation;
struct Simulet;

struct AoiGate {
  float s, x, y;
  Lane* lane;
  LaneObservation* observation;
};

struct AoiCheckpoint {
  std::vector<Person*> person_add_buffer;
  std::vector<Pair<float, Person*>> sleeping_person;
  std::vector<Person*> awake_person;
  std::vector<Person*> veh_for_leaving, ped_for_leaving;
  // TODO: crowd
};

struct Aoi {
  uint id;
  float area;
  MArrZ<AoiGate> walking_gates, driving_gates;
  // 其他实体向AOI插入人用person_add_buffer
  DVector<Person*> person_add_buffer;
  // 等候出发时间的人，优先队列，按时间排序
  DHeapNoLock<float, Person*> sleeping_person;
  // 已经到时间但还在等路由的人
  DVectorNoLock<Person*> awake_person;
  // 在门口排队的行人/车
  DVectorNoLock<Person*> ped_for_leaving, veh_for_leaving;
  // TODO: AoiState，对人做计数
  // 人的行走
  bool enable_indoor;
  crowd::Crowd crowd;

  float GetDrivingS(Lane*);
};

namespace aoi {
struct Data {
  bool out_control = true;
  MArrZ<Aoi> aois;
  std::unordered_map<uint, Aoi*> aoi_map;
  cudaStream_t stream;
  Simulet* S;
  int g_prepare, g_update;
  int b_prepare, b_update;

  void Init(Simulet* S, const PbMap&);
  void PrepareAsync();
  void UpdateAsync();
  void Save(std::vector<AoiCheckpoint>&);
  void Load(const std::vector<AoiCheckpoint>&);
};
}  // namespace aoi

}  // namespace simulet

#endif
