#include <cmath>
#include <cassert>
#include <cstring>
#include "containers/vector.cuh"
#include "entity/entity.h"
#include "entity/junction/junction.cuh"
#include "entity/junction/trafficlight/trafficlight.cuh"
#include "entity/lane/lane.cuh"
#include "protos.h"
#include "utils/color_print.h"
#include "utils/macro.h"
#include "utils/utils.cuh"

namespace simulet::trafficlight {
__device__ void SetLight(Lane* l, LightState ls, float remaining_time) {
  l->light_state = ls;
  l->light_time = remaining_time;
}

__device__ void Prepare(Junction& jc, MPTrafficLight& tl) {
  // 写入snapshot
  tl.snapshot = tl.runtime;
  // 写入lane中数据
  if (tl.phases.size == 0 || !tl.ok) {
    // 无信控，全绿
    for (auto* l : jc.lanes) {
      SetLight(l, LightState::LIGHT_STATE_GREEN, 1e999);
    }
  } else {
    // 设置相位与时间
    if (tl.is_yellow) {
      auto& phase = tl.yellow_phases;
      auto& next_phase = tl.phases[tl.next_index];
      uint index = 0;
      for (auto* l : jc.lanes) {
        // 如果下个相位还是绿灯，则把下个相位的时间也加上
        if (phase[index] == LightState::LIGHT_STATE_GREEN &&
            next_phase[index] == LightState::LIGHT_STATE_GREEN) {
          SetLight(l, phase[index],
                   tl.runtime.remaining_time +
                       (jc.phase_time == 0 ? tl.phase_duration[tl.next_index]
                                           : jc.phase_time));
        } else {
          SetLight(l, phase[index], tl.runtime.remaining_time);
        }
        ++index;
      }
    } else {
      uint index = 0;
      auto& phase = tl.phases[tl.phase_index];
      for (auto* lane : jc.lanes) {
        SetLight(lane, phase[index++], tl.runtime.remaining_time);
      }
    }
  }
}

__device__ void Update(Junction& jc, MPTrafficLight& tl, float step_interval,
                       float yellow_time, float phase_pressure_coeff) {
  if (tl.phases.size == 0 || !tl.ok) {
    return;
  }
  if (jc.tl_policy == TlPolicy::NONE) {
    tl.runtime.remaining_time = 0;
    return;
  }
  // 强制切换相位
  if (tl.set_force) {
    tl.phase_index = tl.next_index;
    tl.runtime.remaining_time =
        jc.phase_time == 0 ? tl.phase_duration[tl.phase_index] : jc.phase_time;
    tl.is_yellow = false;
    tl.set_force = false;
    tl.next_index = (tl.phase_index + 1) % tl.phases.size;
    return;
  }
  // 手动模式，不倒计时
  if (jc.tl_policy == TlPolicy::MANUAL) {
    return;
  }
  tl.runtime.remaining_time -= step_interval;
  if (tl.runtime.remaining_time > 0) {
    return;
  }
  if (tl.is_yellow) {
    // 从黄灯切换到下一相位
    tl.phase_index = tl.next_index;
    tl.runtime.remaining_time +=
        jc.phase_time == 0 ? tl.phase_duration[tl.phase_index] : jc.phase_time;
    tl.is_yellow = false;
  } else {
    // 计算下一相位
    uint next_index;
    switch (jc.tl_policy) {
      case TlPolicy::FIXED_TIME: {
        next_index = (tl.phase_index + 1) % tl.phases.size;
      } break;
      case TlPolicy::MAX_PRESSURE: {
        // 找到最大压力的相位
        uint index = 0;
        for (auto* l : jc.lanes) {
          tl.lane_pressure[index++] =
              l->restriction
                  ? 0
                  : l->predecessor->pressure_out - l->successor->pressure_in;
        }
        float max_pressure = -1;
        for (int i = 0; i < tl.phases.size; ++i) {
          auto& phase = tl.phases[i];
          // 统计所有绿灯junction lane的压力和
          auto pressure = 0.f;
          uint j = 0;
          for (auto& state : phase) {
            if (state == LightState::LIGHT_STATE_GREEN) {
              pressure += tl.lane_pressure[j];
            }
            j++;
          }
          pressure *= tl.phase_pressure_coeff[i];
          if (pressure > max_pressure) {
            max_pressure = pressure;
            next_index = i;
          }
        }
      } break;
      default:
        assert(false);
    }
    for (auto& i : tl.phase_pressure_coeff) {
      i *= phase_pressure_coeff;
    }
    tl.phase_pressure_coeff[next_index] = 1;
    if (next_index == tl.phase_index) {
      // 再续一个周期
      tl.runtime.remaining_time += jc.phase_time == 0
                                       ? tl.phase_duration[tl.phase_index]
                                       : jc.phase_time;
    } else if (yellow_time == 0) {
      // 直接切换到下一相位
      tl.phase_index = next_index;
      tl.runtime.remaining_time += jc.phase_time == 0
                                       ? tl.phase_duration[tl.phase_index]
                                       : jc.phase_time;

    } else {
      // 切换到黄灯
      tl.runtime.remaining_time += yellow_time;
      tl.next_index = next_index;
      // 生成黄灯相位，把当前为绿灯、下一时刻为红灯的变为黄灯
      tl.is_yellow = true;
      tl.yellow_phases.CopyFrom(tl.phases[tl.phase_index]);
      auto& next_phase = tl.phases[next_index];
      uint index = 0;
      for (auto& state : tl.yellow_phases) {
        if (state == LightState::LIGHT_STATE_GREEN &&
            next_phase[index] == LightState::LIGHT_STATE_RED) {
          tl.yellow_phases[index] = LightState::LIGHT_STATE_YELLOW;
        }
        index++;
      }
    }
  }
  if (tl.runtime.remaining_time <= 0) {
    printf("traffic light %d remaining time %f <= 0\n", jc.id,
           tl.runtime.remaining_time);
  }
}

}  // namespace simulet::trafficlight
