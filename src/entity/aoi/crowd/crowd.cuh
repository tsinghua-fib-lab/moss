#ifndef SRC_ENTITY_AOI_CROWD__CROWD_CUH_
#define SRC_ENTITY_AOI_CROWD__CROWD_CUH_
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "entity/lane/lane.cuh"
#include "protos.h"
#include "rand/rand.cuh"
#include "utils/geometry.cuh"

namespace simulet {

struct Person;
struct AoiGate;
struct Simulet;

namespace crowd {

const float DEFAULT_DESIRED_SPEED = 1.34;  // 默认期望速度
const float GATE_SIZE = 3;                 // 出入口宽度
const int TRY_GET_RANDOM_POINT = 3;        // 随机取点最大尝试次数
const float STEP_INTERVAL_INDOOR = .1;     // 室内行人模拟的更新间隔
const auto P_PAUSE_TO_IDLE = .09;

// 行人在追踪的点的类型
enum class InterestType : int {
  SLEEP,         // 去睡觉
  WALKING_GATE,  // 行人出口
  DRIVING_GATE,  // 车辆出口
};

// 行人闲逛的中间目标点
enum class StrollTarget : int {
  UNSET,
  NONE,             // 无中间目标
  RANDOM_POSITION,  // 去往随机地点
  PERSON,           // 跟着别人
};

struct CrowdPerson;

// 闲逛所用数据
struct StrollInfo {
  StrollTarget target;  // 目标类型
  uint person_id;       // 跟随的人的id
  CrowdPerson* person;  // 跟随的行人(目标类型为StrollTarget_PERSON时有效)
};

enum class Status : int {
  IDLE,          // 运动中
  PAUSE,         // 立定不动，做障碍物处理
  REACH_TARGET,  // 到达终点
};

// 室内状态
struct CrowdState {
  Status status;   // 状态
  Point position;  // 位置
  Point velocity;  // 速度
};

// 室内数据
struct CrowdPerson {
  uint id;
  CrowdState snapshot, runtime;
  Point destination;       // 目的地
  float desired_speed;     // 期望速度
  InterestType interest;   // 兴趣点类型，决定从crowd离开后去哪
  Person* person;          // 行人自身指针
  StrollInfo stroll_info;  // 闲逛所用数据
  AoiGate* gate;           // 出去的门
};

struct Crowd {
  Aoi* aoi;            // 所在Aoi
  MArrZ<Point> gates;  // 出入口列表（包括driving_gates和walking_gates）
  MArrZ<Point>
      boundary;  // Aoi 边界点列表。各点顺序给出，注意第一点与最后一点相同
  MArrZ<Point>
      sleep_points;  // 楼电梯间位置（行人走到此处后送入sleeping_person，不再有计算任务）
  MArrZ<float> area_cdf;  // 三角划分后的n-2个三角形面积的CDF，用于随机取点
  DVectorNoLock<CrowdPerson> persons;  // 待更新的室内行人
  rand::Rng64 rng;

  __device__ void WalkToSleep(Person* p);
  __device__ void WalkToGate(Person* p, AoiGate* gate, bool is_veh);
};

void Init(Simulet* S, const PbAoi&, Aoi&);
__device__ void Prepare(Aoi& a);
__device__ void Update(Aoi& a, float step_interval);
}  // namespace crowd
}  // namespace simulet
#endif
