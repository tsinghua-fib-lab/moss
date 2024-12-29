#ifndef SRC_MOSS_CUH_
#define SRC_MOSS_CUH_

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
#include "output/output.cuh"

namespace moss {

struct OutputConfig {
  // enable
  bool enable;
  // output directory (empty for no output)
  std::string dir;
  // output traffic light per file duration
  float tl_file_duration;
};

struct Config {
  // map data file, agent data file
  std::string map_file, person_file;
  // agent limit, -1 for no limit (INF)
  uint person_limit;
  // interval for speed statistics
  float speed_stat_interval;
  // simulation range [start_step, start_step + total_step)
  uint start_step, total_step;
  // simulation step interval, means how many seconds per step
  float step_interval;
  // seed for random number generator
  int seed;
  // output config
  OutputConfig output;
  // output bounding box
  float x_min, y_min, x_max, y_max;
  int verbose_level;
  // CPU worker count
  uint n_workers;
  // 路口黄灯时间（仅限非手动控制情形）
  // Junction yellow light time (only for non-manual control)
  float junction_yellow_time;
  // 路口MP信控压力系数（未放行的相位压力会随时间增长，从而让相位切换更公平）
  // Junction MP signal control pressure coefficient (the pressure of the phase
  // that has not been released will increase over time, making the phase
  // switching more fair)
  float phase_pressure_coeff;
  // CUDA device ID
  uint device;
};

class Moss {
 private:
  // save / restore data structure
  struct Checkpoint {
    uint step;
    int mem_save_id;
  };

  // static data that is not changed during the simulation

  int id;

  // dynamic data that is changed during the simulation

  StepOutput step_output;
  uint64_t last_step_t;

  // save / restore data
  std::vector<Checkpoint> checkpoints;

 public:
  // static data that is not changed during the simulation

  Config config;
  bool verbose;
  // the number of SMs on the GPU
  int sm_count;
  // for coordinate transformation
  float map_west, map_east, map_south, map_north;
  // 在初始化各个对象时种子会自增
  // The seed will be incremented when initializing each object
  uint64_t seed;

  // dynamic data that is changed during the simulation

  float time;
  uint step;

  // global memory pool
  MemManager* mem;

  // data containers
  aoi::Data aoi;
  lane::Data lane;
  road::Data road;
  junction::Data junction;
  person::Data person;

  // Initialize the simulator
  void Init(const std::string&, const Config&);
  // Run the simulation by start_step and total_step for executable mode
  void Run();
  // Run one step
  void Step();
  // Stop the simulation
  void Stop();
  // Close the simulator
  void Close();
  // Save the current state
  int Save();
  // Restore the state by id
  void Restore(int checkpoint_id);
};
};  // namespace moss

#endif
