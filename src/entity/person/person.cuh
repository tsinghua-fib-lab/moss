#ifndef SRC_ENTITY_PERSON_PERSON_CUH_
#define SRC_ENTITY_PERSON_PERSON_CUH_

#include <map>
#include <vector>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "entity/aoi/crowd/crowd.cuh"
#include "entity/entity.h"
#include "protos.h"
#include "rand/rand.cuh"
#include "rpc/routing.cuh"

namespace moss {

struct Trip {
  TripMode mode;
  Position end;
  float wait, departure;
  // TODO: 添加activity字段
};

struct Schedule {
  MArrZ<Trip> trips;
  uint loop_count;
  float wait, departure;
};

struct PersonAttr {
  // 通用属性
  float length, width;
  float max_speed;
  float max_acc, max_braking_acc;
  float usual_acc, usual_braking_acc;
  // 车辆属性
  float lane_change_length;
  float min_gap;
  float headway;
  // TODO: 限速认知偏差
};

enum class PersonStatus : int {
  INIT,
  TO_INSERT,  // 用于指定home为{lane_id,lane_s}时做插入
  SLEEP,
  // WAITING_FOR_BUS,
  CROWD,
  WAITING_FOR_LEAVING,
  // ON_BUS,
  WALKING,
  DRIVING,
  FINISHED
};

struct Lane;
struct Aoi;

struct PersonState {
  PersonStatus status;
  Lane* lane;
  Aoi* aoi;
  float s, speed, acc, direction, distance_to_end;
  bool is_forward;
  // 变道信息
  bool is_lane_changing;
  float lc_length;
  float shadow_s, lc_total_length, lc_complete_length;
  Lane* shadow_lane;
  // 输出信息
  float x, y, dir;
};

struct Person;

struct PersonNode {
  bool is_shadow : 1;
  bool overwritable : 1;  // 变道时本体的s可以被前车覆写
  uint index : 30;        // 人在数组中的下标
  float s;
  Person* self;
  union {  // 链表的前一个结点(prev)从车的角度来看是后一辆车(back)
    PersonNode *prev, *back;
  };
  union {  // 链表的下一个结点(next)从车的角度来看是前车(front)
    PersonNode *next, *front;
  };
  // 两侧的车：[左|右][前|后]
  PersonNode* sides[2][2];
};

#define LEFT 0
#define RIGHT 1
#define FRONT 0
#define BACK 1

struct PersonCheckpoint {
  PersonState snapshot, runtime;
  // 链表结点
  PersonNode node, shadow_node;
  uint schedule_index, trip_index, loop_counter;
  float start_time_or_last_trip_end_time;
  routing::Response route;
  bool route_changed;
  bool route_in_junction;
  uint route_index;
  int route_lc_offset;
  Lane *route_l1, *route_l2, *next_lane;
  float lc_dir;
  float lc_motivation[2];
  rand::Rng64 rng;
  float traveling_or_departure_time;
  bool enable;
};

enum class AccReason {
  NONE,
  LANE_CHANGE_N,
  LANE_CHANGE_V,
  TO_LIMIT,
  TO_END,
  CAR_FOLLOW,
  CAR_FOLLOW_SHADOW,
  LANE_AHEAD,
  LANE_AHEAD_SHADOW,
  LANE_CHANGE_HARD_STOP,
};

enum class LaneChangeAlgorithm {
  NONE,
  SUMO,
  MOBIL,
};

struct Person {
  uint id;
  PersonAttr attr;
  PersonState snapshot, runtime;
  // 链表结点
  PersonNode node, shadow_node;
  // schedule
  MArrZ<Schedule> schedules;
  uint schedule_index, trip_index, loop_counter;
  union {
    float start_time;  // 当home不是AOI时用这个来记录出发时间
    float last_trip_end_time;
  };
  // 状态标记，由update设置
  bool skip_to_end, is_end;
  // route
  routing::Response route;
  bool route_changed;  // 被外部更新了路由
  bool route_in_junction;
  // 路由index（注意：路由不含路口内道路）
  uint route_index;
  // 需要变道几次，正数向左，负数向右
  int route_lc_offset;
  // 当前route所在车道以及允许的车道范围
  Lane *route_l1, *route_l2;
  // 下一条车道
  Lane* next_lane;
  // 变道方向角
  float lc_dir;
  // 变道意愿，左/右
  float lc_motivation[2];
  // 随机数发生器
  rand::Rng64 rng;
#if STUCK_MONITOR
  uint stuck_cnt;
#endif
  float traveling_or_departure_time;  // [API]记录车辆旅行时间

  AccReason _reason;    // [debug]加速度原因
  uint _reason_detail;  // [debug]加速度详细信息
  float ahead_dist;     // [API]前车距离
  bool enable;          // [API]如果为false则不会更新

  __device__ __host__ Trip& GetTrip();
  __device__ float GetDepartureTime();
  __device__ bool NextTrip(float time);
  // 根据p.runetime.lane更新当前道路许可车道范围
  __device__ __host__ void UpdateLaneRange();
  // 根据lane更新下一条车道
  __device__ __host__ void UpdateNextLane(Lane*);
  // 将车放在车道上并初始化，假定runtime.lane和runtime.s已经设置好
  __device__ void InitVehicleOnLane(float);
};

namespace person {
struct MData {
  uint veh_cnt, ped_cnt, crowd_cnt, finished_cnt;
#if STUCK_MONITOR
  uint stuck_cnt;
#endif
  float traveling_time, finished_traveling_time;
};
struct Data {
  MArrZ<Person> persons;
  std::map<uint, Person*> person_map;
  MArrZ<int> veh_lane;
  MArrZ<float> veh_speed, veh_distance, veh_total_distance;
  cudaStream_t stream;
  MData* M;
  Moss* S;
  int g_prepare, g_update;
  int b_prepare, b_update;

  void Init(Moss* S, const PbAgents&, uint);
  void PrepareAsync();
  void UpdateAsync();
  void Save(std::vector<PersonCheckpoint>&);
  void Load(const std::vector<PersonCheckpoint>&);
  // 重新设定车辆的路由
  void SetRoute(int person_index, std::vector<int> route, int end_lane_id,
                float end_s);
};
__device__ void ClearLaneChange(PersonState& runtime);
__device__ void UpdateVehicle(Person& p, float global_time, float step_interval,
                              uint junction_blocking_count,
                              uint lane_change_algorithm,
                              float mobil_lc_forbidden_distance);
__device__ void UpdatePedestrian(Person& p, float global_time,
                                 float step_interval);
}  // namespace person
}  // namespace moss

#endif
