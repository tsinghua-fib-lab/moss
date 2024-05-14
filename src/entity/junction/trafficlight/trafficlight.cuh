#ifndef SRC_ENTITY_JUNCTION_TRAFFICLIGHT_TRAFFICLIGHT_CUH_
#define SRC_ENTITY_JUNCTION_TRAFFICLIGHT_TRAFFICLIGHT_CUH_

#include <cstdint>
#include "containers/array.cuh"
#include "containers/queue.cuh"
#include "entity//lane/lane.cuh"
#include "protos.h"

namespace simulet::trafficlight {
struct MPTLRuntime {
  float remaining_time;  // 当前相位剩余时间
};

struct MPTrafficLight {
  bool ok;  // 是否启用
  // TODO: 二维矩阵？
  MArrZ<MArrZ<LightState>>
      phases;  // 可供最大压力算法选择的相位列表（如果nil，则没有信控）
  uint phase_index;                   // 当前相位
  MArrZ<LightState> yellow_phases;    // 黄灯状态下的相位
  bool is_yellow;                     // 为nil代表当前不处在黄灯状态
  uint next_index;                    // 黄灯状态后的下一个相位
  MArrZ<float> lane_pressure;         // 车道压力
  MArrZ<float> phase_pressure_coeff;  // 相位压力系数，随着时间增长
  MArrZ<float> phase_duration;        // 相位时长[API]
  MPTLRuntime snapshot, runtime;      // 运行时数据
  bool set_force;                     // 强制切换到下一个相位[API]
};

__device__ void Prepare(Junction& jc, MPTrafficLight& tl);
__device__ void Update(Junction& jc, MPTrafficLight& tl, float step_interval,
                       float yellow_time, float phase_pressure_coeff);
__device__ void UpdateApi(Junction& jc, MPTrafficLight& tl,
                          float step_interval);

}  // namespace simulet::trafficlight

#endif
