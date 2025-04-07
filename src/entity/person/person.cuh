#ifndef SRC_ENTITY_PERSON_PERSON_CUH_
#define SRC_ENTITY_PERSON_PERSON_CUH_

#include <map>
#include <vector>
#include "containers/array.cuh"
#include "containers/list.cuh"
#include "entity/person/route.cuh"
#include "fmt/core.h"
#include "output/person.h"
#include "protos.h"
#include "rand/rand.cuh"

namespace moss {

// departure time priority
// Schedule
// 关于出发时间的说明如下：
// The explanation about the departure time is as follows:
// 1. Schedule的开始时刻是 departure_time 或者 参考时刻+wait_time，
// 1. The start time of the Schedule is either departure_time or reference
// time+wait_time,
//    参考时刻定义为上一Schedule的结束时刻(即它最后一个Trip的结束时刻)，
//    The reference time is defined as the end time of the previous Schedule
//    (i.e. the end time of its last Trip),
//    或者当它为第一个Schedule时定义为Person更新Schedule后的首次Update
//    Alternatively, when it is the first Schedule, it can be defined as the
//    first Update time after Person updates the Schedule
//    时刻(当有准确时间要求时建议直接指定departure_time)
//    (it is recommended to specify departuretime directly when there is an
//    accurate time requirement)
// 2. Trip的开始时刻是 departure_time 或者 参考时刻+wait_time，参考
// 2. The start time of the Trip is either departure_time or reference
// time+wait_time,
//    时刻定义为上一Trip的结束时刻，或者当它为第一个Trip时定义为所属的
//    The reference time is defined as the end time of the previous Trip, or
//    when it is the first Trip, Schedule的开始时刻 it is defined as the start
//    time of the Schedule to which it belongs
// 3. Person的实际运行时刻取决于Trip的开始时刻，例如它的首次运行是第一
// 3. The actual running time of a Person depends on the start time of the Trip,
//    个Schedule中第一个Trip的开始时刻
//    for example, its first run is the start time of the first Trip in the
//    first Schedule
// FAQ
// Q1: 同时指定Schedule和第一个Trip的departure_time会怎样？
// Q1: What would happen if both the Schedule and the departuretime of the first
// Trip were specified simultaneously? A1: 参照(2)，只看Trip的departure_time A1:
// Referring to (2), only depend on the departuretime of Trip Q2:
// Schedule和第一个Trip同时指定wait_time=10会怎样？ Q2: What would happen if
// both the Schedule and the first Trip were specified with wait_time=10 at the
// same time? A2: 参照(2)，等待时间为10+10=20 A2: Referring to (2), the waiting
// time is 10+10=20

struct Trip {
  TripMode mode;
  Aoi* end_aoi;
  Lane* end_lane;
  float end_s;
  // departure time related data
  // < 0 means null
  float wait, departure;
  // pre-computed route
  Route route;
};

struct Schedule {
  MArr<Trip> trips;
  // departure time related data
  // < 0 means null
  float wait, departure;
};

struct VehicleAttr {
  // vehicle length (m)
  float length;
  // vehicle width (m)
  float width;
  // vehicle max speed (m/s)
  float max_v;
  // vehicle max acceleration (m/s^2)
  float max_a;
  // vehicle max braking acceleration (m/s^2)
  float max_braking_a;
  // vehicle usual acceleration (m/s^2)
  float usual_a;
  // vehicle usual braking acceleration (m/s^2)
  float usual_braking_a;
  // vehicle lane change length (m)
  float lane_change_length;
  // vehicle min gap (m)
  float min_gap;
  // vehicle headway (s)
  float headway;
  // The deviation of the vehicle's recognition of lane max speed
  float lane_max_v_deviation;
};

struct PedestrianAttr {
  // pedestrian speed (m/s)
  float v;
};

enum class PersonStatus : int {
  // stop, unseen, waiting for departure
  SLEEP,
  // walking (pedestrian)
  WALKING,
  // driving (vehicle)
  DRIVING,
  // finish all the schedules
  FINISHED
};

struct Lane;
struct Aoi;

struct PersonState {
  // status
  PersonStatus status;

  // current position, choose lane+s or aoi

  // current lane (with s)
  Lane* lane;
  float s;
  // current aoi
  Aoi* aoi;

  // speed (m/s)
  float v;

  // lane change related data (only for driving)

  // the lane from which the vehicle is changing
  // if the vehicle is not changing lane, it is nullptr
  Lane* shadow_lane;
  float shadow_s;
  // Angle of deflection of the vehicle's nose relative to the forward direction
  // during a lane change (radians, always positive, 0 means no steering)
  float lc_yaw;
  // the ratio of the lane change completion (0-1)
  float lc_completed_ratio;

  // is walking forward (only for pedestrian)
  bool is_forward;

  // coordinate (for output)
  float x, y, z, dir, pitch;

  __device__ void UpdatePositionDir();
  __device__ __inline__ bool InShadowLane() const {
    return shadow_lane != nullptr && lc_completed_ratio < 0.5;
  }
};

struct Person;

struct PersonNode {
  bool is_shadow : 1;
  uint index : 31;  // index in the array
  float s;
  Person* self;
  // add / remove buffer linked list node
  DNode<PersonNode> add_node, remove_node;
  union {  // the previous node(prev) / the back vehicle(back)
    PersonNode *prev, *back;
  };
  union {  // the next node(next) / the front vehicle(front)
    PersonNode *next, *front;
  };
  // 两侧的车：[左|右][前|后]
  // side vehicles: [left|right][front|back]
  PersonNode* sides[2][2];

  __host__ __device__ void PrintDebugString(bool show_link = true) const;
};

#define LEFT (0)
#define RIGHT (1)
#define FRONT (0)
#define BACK (1)

enum class AccReason {
  NONE,
  LANE_CHANGE_N,
  LANE_CHANGE_V,
  // to the max speed limit
  TO_LIMIT,
  TO_END,
  CAR_FOLLOW,
  CAR_FOLLOW_SHADOW,
  LANE_AHEAD,
  LANE_AHEAD_SHADOW,
  LANE_CHANGE_HARD_STOP,
  // traffic light: red
  RED_LIGHT,
  // traffic light: yellow
  YELLOW_LIGHT,
};

enum class LaneChangeAlgorithm {
  NONE,
  SUMO,
  MOBIL,
};

struct Person {
  // person id
  uint id;
  // vehicle attributes
  VehicleAttr veh_attr;
  // pedestrian attributes
  PedestrianAttr ped_attr;
  // person state (two copy: snapshot and runtime)
  PersonState snapshot, runtime;
  // sensing linked list node
  // when changing lane,
  // the shadow node is used to mark the position of the old lane
  PersonNode node, shadow_node;

  // ===== schedule related data =====

  // schedule
  // SET in Data::Init()
  MArr<Schedule> schedules;
  // current schedule index and trip index
  // SET in NextTrip()
  uint schedule_index, trip_index;
  // current trip pointer to the trip in the schedule
  // SET in NextTrip()
  Trip* trip;
  // departure time
  // SET in NextTrip()
  float departure_time;
  // travelling time
  // INIT in NextTrip() AS 0, UPDATE in vehicle/pedestrian::Update()
  float traveling_time;
  // total distance
  // INIT in NextTrip() AS 0, UPDATE in vehicle/pedestrian::Update()
  float total_distance;

  // ===== route related data =====

  // current route pointer to the route in the schedule
  // SET in NextTrip()
  Route* route;
  // the index of the route (note: the route does not include the junction lane)
  // INIT in NextTrip() AS -1, UPDATE in vehicle/pedestrian::Update()
  // SHOULD CALL NextVehicleRoute() When starting a new route
  uint route_index;
  // flag to indicate whether the route is in the junction or in the road
  // INIT in NextTrip(), UPDATE in vehicle/pedestrian::Update()
  bool route_in_junction;
  // the lane range from where the vehicle can arrive the correct lanes in the
  // next junction: [target_offset1, target_offset2]
  // SET in Person::UpdateLaneRange() <- NextVehicleRoute() / PlanLaneChange()
  uint target_offset1, target_offset2;

  // ===== lane change related data =====
  // SET in PlanLaneChange()

  // flag whether the vehicle is in FORCE LC mode
  bool force_lc;
  // last time of lane change
  float lc_last_t = -1e999;

  // random number generator
  // INIT in Data::Init()
  rand::Rng64 rng;

  // action related data, SET in UpdateAction()

  // acceleration (m/s^2)
  float acc;
  // the target of the lane change (nullptr means no lane change)
  Lane* lc_target;
  // the phi (front wheel steering angle) of the vehicle during lane change
  float lc_phi;

  // ===== api & debug =====

#if STUCK_MONITOR
  uint stuck_cnt;
#endif

  AccReason _reason;    // [debug]加速度原因
  uint _reason_detail;  // [debug]加速度详细信息
  float ahead_dist;     // [API]前车距离
  bool enable;          // [API]如果为false则不会更新

  // ===== functions =====

  __host__ __device__ Trip& GetTrip();
  __host__ __device__ bool NextTrip(float t);

  // ===== vehicle related functions =====

  // Update the lane range of the current road according to p.runetime.lane
  __host__ __device__ void UpdateLaneRange();
  // Update the next junction lane according to the current road lane and the
  // route
  __host__ __device__ Lane* FindNextJunctionLane(Lane* road_lane,
                                                 uint route_index,
                                                 bool ignore_error = false);
  // Go to the next route for the vehicle
  __device__ bool NextVehicleRoute();

  // ===== functions for controlling the vehicle =====

  // set the route of the current schedule and trip
  // in the function, we will check the valid of the person and the route's
  // starting point and ending point
  __host__ void SetVehicleRoute(Moss* S, const std::vector<uint>& route);
};

namespace person {
struct MData {
  uint status_cnts[4];
#if STUCK_MONITOR
  uint stuck_cnt;
#endif
};
struct Data {
  std::vector<int> ids;

  MArr<Person> persons;

  // use columns to store the snapshot data
  MArr<int8_t> s_enable;
  MArr<int> s_status;
  MArr<int> s_lane_id;
  MArr<int> s_lane_parent_id;
  MArr<float> s_s;
  MArr<int> s_aoi_id;
  MArr<float> s_v;
  MArr<int> s_shadow_lane_id;
  MArr<float> s_shadow_s;
  MArr<float> s_lc_yaw;
  MArr<float> s_lc_completed_ratio;
  MArr<int8_t> s_is_forward;
  MArr<float> s_x;
  MArr<float> s_y;
  MArr<float> s_z;
  MArr<float> s_dir;
  MArr<float> s_pitch;
  MArr<int> s_schedule_index;
  MArr<int> s_trip_index;
  MArr<float> s_departure_time;
  MArr<float> s_traveling_time;
  MArr<float> s_total_distance;

  MArr<PersonOutput> outputs;
  std::map<uint, Person*> person_map;
  cudaStream_t stream;
  MData* M;
  Moss* S;
  int g_prepare, g_update;
  int b_prepare, b_update;

  void Init(Moss* S, const PbPersons&, uint);
  inline Person* At(uint id) {
    auto iter = person_map.find(id);
    if (iter == person_map.end()) {
      throw std::range_error(fmt::format("person {} not found", id));
    }
    return iter->second;
  }
  void PrepareAsync();
  void UpdateAsync();
};
// update vehicle and return is_end
__device__ bool UpdateVehicle(Person& p, float t, float dt);
// update pedestrian and return is_end
__device__ bool UpdatePedestrian(Person& p, float t, float dt);
}  // namespace person
}  // namespace moss

#endif
