#ifndef SRC_OUTPUT_CUH_
#define SRC_OUTPUT_CUH_

#include <fstream>
#include <string>
#include <thread>
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "utils/barrier.h"

namespace simulet {
struct Simulet;
struct Lane;

namespace output {

// 输出类型选项
enum class Option : int {
  DISABLE,
  AGENT,
  LANE,
};

enum class AgentOutputType : int {
  CONTROL_INFO,
  VEHICLE,
  PEDESTRIAN,
  TRAFFIC_LIGHT,
};

struct AgentOutput {
  AgentOutputType type;
  union {
    // 控制信息
    struct {
      uint step;
    };
    // 人/车
    struct {
      uint p_id, lane_id;
      float s, x, y, dir;
    };
    // 信号灯
    struct {
      uint l_id, light_state;
      float light_time, cx, cy;
    };
  };
};

struct MData {
  DVector<AgentOutput> agent_output;
  MArrZ<char> road_output;
};

struct Data {
 private:
  std::ofstream file;
  std::thread worker_t;
  bool stop;
  Barrier barrier;
  std::vector<Lane*> output_lanes;
  Simulet* S;
  void worker();

 public:
  Option option;
  float X_MIN, X_MAX, Y_MIN, Y_MAX;
  MData* M;

  // 注意：MData在Update中更新，因此应该在Update后Start，Update前Wait

  // 初始化输出
  void Init(Simulet* S, const std::string& filename);
  // 开始并行输出MData中的内容
  void Start();
  // 等待并行输出结束
  void Wait();
  // 终止并行线程
  void Stop();
};
}  // namespace output
}  // namespace simulet

#endif
