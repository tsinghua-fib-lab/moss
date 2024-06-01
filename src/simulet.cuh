#ifndef SRC_SIMULET_H_
#define SRC_SIMULET_H_

#include <cuda.h>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "entity/aoi/aoi.cuh"
#include "entity/junction/junction.cuh"
#include "entity/lane/lane.cuh"
#include "entity/person/person.cuh"
#include "entity/road/road.cuh"
#include "mem/mem.cuh"
#include "output.cuh"

namespace simulet {
struct Config {
  std::string map_file, agent_file;
  uint agent_limit;
  std::string output_file, output_type;
  float speed_stat_interval;  // 车辆平均速度统计间隔；等于0则不启用统计
  uint start_step, total_step;
  float step_interval;
  std::string routing_url;
  bool pre_routing;        // 预先对所有个体计算路由
  bool enable_aoi_indoor;  // 开启室内行走
  bool enable_junction;    // 启用路口信控
  int seed;
  float x_min, y_min, x_max, y_max;
  int verbose_level;
  // 禁用aoi车辆放行控制
  bool disable_aoi_out_control;
  // 并行初始化的worker数，0表示等于可用CPU数
  uint n_workers;
  // 路口禁止驶入的车辆数阈值
  uint junction_blocking_count;
  // 路口黄灯时间（仅限非手动控制情形）
  float junction_yellow_time;
  // 路口MP信控压力系数（未放行的相位压力会随时间增长，从而让相位切换更公平）
  float phase_pressure_coeff;
  // 变道算法
  uint lane_change_algorithm;
  // MOBIL变道算法下禁止变道的距离
  float mobil_lc_forbidden_distance;
  // 各类缓冲大小
  uint lane_veh_add_buffer_size;
  uint lane_veh_remove_buffer_size;
  // CUDA设备ID
  uint device;
};

// 用于恢复模拟器状态的记录
struct Checkpoint {
  int step;
  float time;
  uint veh_cnt, ped_cnt, crowd_cnt, finished_cnt;
  float traveling_time, finished_traveling_time;
  std::vector<LaneCheckpoint> lanes;
  std::vector<RoadCheckpoint> roads;
  std::vector<JunctionCheckpoint> junctions;
  std::vector<AoiCheckpoint> aois;
  std::vector<PersonCheckpoint> persons;
  std::vector<int> veh_lane;
  std::vector<float> veh_speed;
  std::vector<float> veh_distance;
  std::vector<float> veh_total_distance;
};

class Simulet {
 private:
  std::map<size_t, Checkpoint*> checkpoints;
  uint64_t last_step_t;
  int id;

 public:
  Config config;
  bool verbose;
  float time;
  uint step;
  bool is_python_api;
  uint64_t seed;  // 在初始化各个对象时种子会自增
  bool enable_api_output;

  int sm_count;  // GPU的SM数量
  MemManager* mem;
  output::Data output;
  routing::Data routing;

  aoi::Data aoi;
  lane::Data lane;
  road::Data road;
  junction::Data junction;
  person::Data person;

  // 初始化
  void Init(const Config&);
  // 运行
  void Run();
  // 仿真一步
  void Step();
  // 停止
  void Stop();
  // 创建当前模拟器状态的存档
  size_t Save();
  // 恢复模拟器状态为指定的存档
  void Load(size_t);
};
};  // namespace simulet

#endif
