#include <cassert>
#include <stdexcept>
#include "entity/person/person.cuh"
#include "entity/road/road.cuh"
#include "fmt/core.h"
#include "moss.cuh"

namespace moss {

// 计算指定lane到路由许可车道范围的差距
__device__ __host__ int LcOffset(const Person& p, const Lane* l) {
  return l < p.route_l1    ? l - p.route_l1
         : l >= p.route_l2 ? l - p.route_l2 + 1
                           : 0;
}

// 将给定车道转换为路由中的车道
__device__ __host__ Lane* ToRouteLane(const Person& p, Lane* l) {
  assert(l->parent_id == p.route.veh->route[p.route_index]);
  return l < p.route_l1 ? p.route_l1 : l >= p.route_l2 ? p.route_l2 - 1 : l;
}

// 路由前进，如果走完了返回false
__device__ bool NextRoute(Person& p) {
  if (p.route_index + 1 == p.route.veh->route.size) {
    return false;
  }
  p.route_in_junction = !p.route_in_junction;
  auto* lane = p.runtime.lane = p.next_lane;
  if (p.route_in_junction) {
    p.route_lc_offset = 0;
    p.next_lane = lane->successor;
  } else {
    ++p.route_index;
    p.UpdateLaneRange();
    p.route_lc_offset = LcOffset(p, lane);
    p.UpdateNextLane(p.route_lc_offset ? ToRouteLane(p, lane) : lane);
  }
  return true;
}

// 更新路由许可车道范围
__device__ __host__ void Person::UpdateLaneRange() {
  // TODO: 现在这个函数只在每次进入道路时才调用
  // 真实情况是，动态车道会在离路口一定距离的位置放置指示牌，车辆在到达指示牌前应该始终检查
  if (route_index + 1 == route.veh->route.size) {
    route_l1 = route.veh->end_lane;
    route_l2 = route_l1 + 1;
  } else {
    auto& r = *runtime.lane->parent_road;
    auto nr = route.veh->route[route_index + 1];
    bool found = false;
    for (int i = r.nrl_a; i < r.nrl_b; ++i) {
      auto& x = r.next_road_lanes[i];
      if (nr == x.id) {
        route_l1 = x.l1;
        route_l2 = x.l2;
        found = true;
        break;
      }
    }
    if (!found) {
      printf(RED("[Error] Person[%d] cannot find lane from Road[%d] to "
                 "Road[%d]\n"),
             id, r.id, nr);
      SetError(ErrorCode(ErrorType::ANY, 1));
      route_l1 = runtime.lane;
      route_l2 = route_l1 + 1;
    }
#ifndef __CUDA_ARCH__
    throw std::runtime_error(
        fmt::format("Cannot find lane from Road[{}] to Road[{}]", r.id, nr));
#endif
  }
}

// 更新next_lane
__device__ __host__ void Person::UpdateNextLane(Lane* lane) {
  next_lane = nullptr;
  if (route_index + 1 == route.veh->route.size) {
    return;
  }
  // 当有多个路口内车道可供选择时，选择后继车道上车最少的
  uint best = unsigned(-1);
  uint rid = route.veh->route[route_index + 1];
  for (auto& i : lane->successors) {
    if (i.lane->next_road_id == rid) {
      auto c = (i.lane->successor->veh_cnt & 0xffffff) +
               (i.lane->restriction ? 0x1000000 : 0);
      if (c < best) {
        best = c;
        next_lane = i.lane;
      }
    }
  }
  if (!next_lane) {
    printf(RED("[Error] Cannot reach road %d from lane %d\n"), rid,
           runtime.lane->id);
    assert(false);
  }
}

// 支持按lane_position初始化车辆
__device__ void Person::InitVehicleOnLane(float global_time) {
  runtime.status = PersonStatus::DRIVING;
  runtime.distance_to_end = max(0.f, route.veh->distance_to_end[0] - runtime.s);
  UpdateLaneRange();
  route_lc_offset = LcOffset(*this, runtime.lane);
  UpdateNextLane(route_lc_offset ? ToRouteLane(*this, runtime.lane)
                                 : runtime.lane);
  runtime.lane->veh_add_buffer.Append(&node);
  traveling_or_departure_time = -global_time;  // 记录出发时间
}

namespace person {
// 判定到达终点的距离
const float CLOSE_TO_END = 5;
// IDM参数
const float IDM_THETA = 4;
// 车道前后不能变道的长度
const float LC_FORBIDDEN_DISTANCE = 5;
// 检查(前后不能变道的)车道长度的最小值
const float LC_CHECK_LANE_MIN_LENGTH = 20;
// 变道长度与车速的关系
const float LC_LENGTH_FACTOR = 3;
// 随机发起变道概率下限（25秒内90%变道）
// const float LC_MIN_P = 0.1;
// 在道路尽头时，5秒内60%变道
// const float LC_DISTANCE_P_FACTOR = 0.6;
// 自主变道后期望最少在目标车道停留的长度
// const float LC_MIN_INTERVAL = 20;
// 触发变道决策的最小速度增益（m/s）
const float LC_MIN_CHANGE_SPEED_GAIN = 10 / 3.6;
// 自主变道估计车速的距离
const float VIEW_DISTANCE = 200;
// 自主变道意愿阈值
__constant__ const float MOTIVATION_THRESHOLD[2] = {6, 7};
// 自主变道能接受的减速阈值
const float LC_BRAKING_TOLORENCE = -0.2;
// 自主变道让后车刹车的加速度阈值相对其最大刹车加速度的偏差
const float LC_SAFE_BRAKING_BIAS = 1;

void Data::SetRoute(int person_index, std::vector<int> route, int end_lane_id,
                    float end_s) {
  // 合法性检查
  if (route.empty()) {
    throw std::invalid_argument("Route is empty");
  }
  for (auto i : route) {
    if (S->road.road_map.count(i) == 0) {
      throw std::invalid_argument(fmt::format("Road {} does not exist", i));
    }
  }
  if (person_index >= persons.size) {
    throw std::out_of_range("Person index out of range");
  }
  auto& p = persons[person_index];
  if (p.runtime.status == PersonStatus::DRIVING ||
      p.runtime.status == PersonStatus::TO_INSERT) {
    if (!p.runtime.lane->parent_is_road) {
      throw std::runtime_error(
          "Cannot set vehicle route when it is inside junctions");
    }
    if (p.runtime.lane->parent_road->id != route.at(0)) {
      throw std::invalid_argument(
          "The first road in the route must be the current road");
    }
  } else {
    throw std::runtime_error(
        fmt::format("Cannot set vehicle route when the status is {}",
                    (int)p.runtime.status));
  }
  Lane* end_lane;
  if (end_lane_id == -1) {
    end_lane = S->road.road_map.at(route[route.size() - 1])->right_driving_lane;
    if (!end_lane) {
      throw std::invalid_argument(fmt::format(
          "Road {} does not have a drivable lane", route[route.size() - 1]));
    }
  } else {
    auto it = S->lane.lane_map.find(end_lane_id);
    if (it == S->lane.lane_map.end()) {
      throw std::invalid_argument(
          fmt::format("Lane {} does not exist", end_lane_id));
    }
    end_lane = it->second;
  }
  if (end_s < 0) {
    end_s += end_lane->length;
  }
  end_s = max(0, min(end_s, end_lane->length));

  // 更新路由数据
  auto* v = S->mem->MValueZero<routing::VehicleRoute>();
  auto& r = v->route;
  auto n = route.size();
  r.New(S->mem, n);
  for (int i = 0; i < n; ++i) {
    r[i] = route[i];
  }
  auto& d = v->distance_to_end;
  d.New(S->mem, 2 * n - 1);
  v->end_s = end_s;
  v->end_lane = end_lane;
  v->end_lane->GetPosition(v->end_s, v->end_x, v->end_y);
  for (int i = n - 2; i >= 0; --i) {
    uint nr = route[i + 1];
    bool reachable = false;
    for (auto* l : S->road.road_map.at(route[i])->lanes) {
      if (l->type == LaneType::LANE_TYPE_DRIVING) {
        for (auto& ll : l->successors) {
          if (ll.lane->next_road_id == nr) {
            d[2 * i + 1] = ll.lane->length;
            d[2 * i] = l->length;
            reachable = true;
            break;
          }
        }
        if (reachable) {
          break;
        }
      }
    }
    if (!reachable) {
      throw std::invalid_argument(fmt::format(
          "Wrong route: cannot reach Road[{}] from Road[{}]", nr, route[i]));
    }
  }
  double s = v->end_s;
  d.back() = (float)s;
  for (int i = d.size - 2; i >= 0; --i) {
    d[i] = s += d[i];
  }

  // 更新路由状态
  p.route.veh = v;
  p.route_changed = true;
  if (p.runtime.status == PersonStatus::DRIVING) {
    p.route_index = 0;
    p.runtime.distance_to_end =
        max(0.f, p.route.veh->distance_to_end[0] - p.runtime.s);
    p.UpdateLaneRange();
    auto* lane =
        p.runtime.is_lane_changing ? p.runtime.shadow_lane : p.runtime.lane;
    p.route_lc_offset = LcOffset(p, lane);
    p.UpdateNextLane(p.route_lc_offset ? ToRouteLane(p, lane) : lane);
  }
}

__device__ float ProjectFromLane(Lane* src_lane, Lane* dest_lane, float s) {
  assert(src_lane->parent_id == dest_lane->parent_id);
  return Clamp(s / src_lane->length * dest_lane->length, 0.f,
               dest_lane->length);
}

__device__ float ProjectToLaneSide(Lane* lane, uint side, float s) {
  auto* l2 = lane->side_lanes[side];
  return Clamp(s / lane->length * l2->length, 0.f, l2->length);
}

__device__ void ClearLaneChange(PersonState& runtime) {
  runtime.shadow_lane = nullptr;
  runtime.shadow_s = 0;
  runtime.lc_total_length = 0;
  runtime.lc_complete_length = 0;
  runtime.is_lane_changing = false;
}

// 变道完成
__device__ void FinishLaneChange(Person& p, Lane* lane, float s) {
  p.runtime.lane = lane;
  p.runtime.s = s;
  if (p.route_changed) {
    return;
  }
  assert(p.route_lc_offset);
  if (p.route_lc_offset > 0) {
    --p.route_lc_offset;
  } else {
    ++p.route_lc_offset;
  }
  if (!p.route_lc_offset) {
    p.UpdateNextLane(lane);
  }
}

// 从(lane,s)直行ds的距离并更新位置
__device__ void DriveStraightAndRefreshLocation(Person& p, float ds) {
  auto& s = p.runtime.s;
  auto& lane = p.runtime.lane;
  s += ds;
  // 如果走完了当前车道
  if (s > lane->length) {
    // 清除变道
    if (p.runtime.shadow_lane) {
#if !PERF
      printf(RED("[Warning]vehicle: vehicle %u skipped the change to lane %u "
                 "(status=%u)\n"),
             p.id, p.runtime.shadow_lane->id, p.runtime.is_lane_changing);
#endif
      ClearLaneChange(p.runtime);
    }
    while (s > lane->length) {
      s -= lane->length;
#if !PERF
      if (p.route_lc_offset != 0) {
        printf(RED("[Warning] teleport: veh %d lane %d length=%.2f s=%.2f "
                   "ds=%.2f\n"),
               p.id, lane->id, lane->length, s, ds);
      }
#endif
      // 前进一步
      if (!NextRoute(p)) {
        p.skip_to_end = true;
        return;
      }
    }
  }
}

// 计算在给定距离内改变速度所需的加速度
// __device__ float ComputeAcc(float v_end, float v_now, float distance) {
//   return (v_end * v_end - v_now * v_now) / 2.f / max(distance, 1e-6f);
// }
// 计算采用给定加速度刹停所需的距离
// __device__ float ComputeBreakingDistance(float v, float acc) {
//   assert(acc < 0);
//   return v * v * .5f / -acc;
// }
// __device__ float ComputeBreakingDistance(float v1, float v2, float acc) {
//   assert(acc < 0);
//   return (v1 * v1 - v2 * v2) * .5f / -acc;
// }
// Krauss跟车模型
// 记本车为B，前车为A，本时刻速度为v_B和v_A，那么本车下一时刻的速度u需要满足
// 下一时刻本车普通刹车的刹车距离 + 本时刻距离 <= 前车急刹的距离+现有距离
// u^2/(2a_B)+(v_B+u)t/2 ≤ (v_A^2)/(2a_A)+d
// distance: 本车车头到前车车尾的距离
// __device__ float KraussCarFollowAcc(Person& p, float step_interval,
//                                     float ahead_speed,
//                                     float ahead_max_breaking_acc,
//                                     float distance) {
//   if (distance <= 0) {
//     return p.attr.max_braking_acc;
//   }
//   float a = .5f / -p.attr.usual_braking_acc;
//   float b = .5f * step_interval;
//   float c = .5f * p.snapshot.speed * step_interval -
//             ahead_speed * ahead_speed * .5f / -ahead_max_breaking_acc +
//             p.attr.min_gap - distance;
//   float delta = b * b - 4 * a * c;
//   if (delta < 0) {
//     // 紧急刹车
//     return p.attr.max_braking_acc;
//   }
//   float target_speed = max(0.f, (-b + sqrt(delta)) * .5f / a);
//   return Clamp((target_speed - p.snapshot.speed) / step_interval,
//                p.attr.usual_braking_acc, p.attr.usual_acc);
// }

// IDM跟车模型
__device__ float IDMCarFollowAcc(const Person& p, float target_speed,
                                 float ahead_speed, float distance,
                                 float headway) {
  if (distance <= 0) {
    return p.attr.max_braking_acc;
  }
  auto v = p.snapshot.speed;
  auto s = p.attr.min_gap +
           max(0, v * (headway +
                       (v - ahead_speed) / 2.f /
                           sqrt(-p.attr.usual_braking_acc * p.attr.max_acc)));
  return p.attr.max_acc *
         (1 - pow(v / target_speed, IDM_THETA) - pow(s / distance, 2));
}

__device__ void SetAcc(Person& p, float acc, AccReason reason,
                       uint reason_detail = 0) {
  if (acc < p.runtime.acc) {
    p.runtime.acc = acc;
    p._reason = reason;
    p._reason_detail = reason_detail;
  }
}

// 限速
__device__ void PolicyToLimit(Person& p, float step_interval) {
  float max_speed = min(p.attr.max_speed, p.snapshot.lane->max_speed);
  if (p.snapshot.is_lane_changing) {
    max_speed = min(max_speed, p.snapshot.shadow_lane->max_speed);
  }
  SetAcc(p, IDMCarFollowAcc(p, max_speed, 1e999, 1e999, 0),
         AccReason::TO_LIMIT);
}

// 跟车
__device__ float _CarFollow(Person& p, PersonNode& node, Lane* lane,
                            float step_interval, uint& ahead_id) {
  // 感知前车
  ahead_id = (uint)-1;
  p.ahead_dist = -1;
  if (node.front) {
    auto* v = node.front->self;
    ahead_id = v->id;
    p.ahead_dist = node.front->s - node.s - v->attr.length;
    return IDMCarFollowAcc(p, min(p.attr.max_speed, lane->max_speed),
                           v->snapshot.speed, p.ahead_dist, p.attr.headway);
  } else if (p.next_lane) {
    auto* n = p.next_lane->veh_head;
    if (n) {
      auto* v = n->self;
      ahead_id = v->id;
      p.ahead_dist = n->s + lane->length - node.s - v->attr.length;
      return IDMCarFollowAcc(p, min(p.attr.max_speed, lane->max_speed),
                             v->snapshot.speed, p.ahead_dist, p.attr.headway);
    }
  }
  //  KraussCarFollowAcc(p, step_interval, ahead_veh->snapshot.speed,
  //                     ahead_veh->attr.max_braking_acc,
  //                     ahead_s - node.s - ahead_veh->attr.length)
  return 1e999;
}
__device__ void PolicyCarFollow(Person& p, float step_interval) {
  uint ahead_id;
  float acc = _CarFollow(p, p.node, p.snapshot.lane, step_interval, ahead_id);
  SetAcc(p, acc, AccReason::CAR_FOLLOW, ahead_id);
}
__device__ void PolicyShadowCarFollow(Person& p, float step_interval) {
  uint ahead_id;
  float acc = _CarFollow(p, p.shadow_node, p.snapshot.shadow_lane,
                         step_interval, ahead_id);
  SetAcc(p, acc, AccReason::CAR_FOLLOW_SHADOW, ahead_id);
}

// 下一车道
__device__ float LaneAhead(Person& p, float step_interval, float distance,
                           uint junction_blocking_count, uint& reason_detail) {
  if (!p.next_lane) {
    return 1e999;
  }
  // 前路段限行
  bool stop = p.next_lane->restriction;
  if (stop) {
    reason_detail = 0;
  } else if (!p.next_lane->parent_is_road) {  // 信控
    stop = p.next_lane->veh_cnt > junction_blocking_count;
    if (stop) {
      reason_detail = 1;
    } else
      switch (p.next_lane->light_state) {
        case LightState::LIGHT_STATE_RED:
          // 红灯减速停车
          reason_detail = 2;
          stop = true;
          break;
        case LightState::LIGHT_STATE_YELLOW:
          // 黄灯，倒计时结束前不可过线，减速停车
          if (p.next_lane->light_time * p.snapshot.speed <= distance) {
            reason_detail = 3;
            stop = true;
          }
          break;
        default:
          break;
          // 绿灯或没灯，跳过
      }
  }
  if (stop) {
    return IDMCarFollowAcc(p, 1e999, 0, distance - 1.f, step_interval);
  }
  // 下一车道限速
  float v1 = p.snapshot.speed;
  float v2 = p.next_lane->max_speed;
  if (p.next_lane->parent_junction && p.next_lane->successor) {
    v2 = min(v2, p.next_lane->successor->max_speed);
  }
  if (v1 > v2 && v1 * max(step_interval, p.attr.headway) > distance) {
    reason_detail = 4;
    return (v2 * v2 - v1 * v1) / 2 / max(0.1, distance);
  }
  return 1e999;
}
__device__ void PolicyLaneAhead(Person& p, float step_interval,
                                uint junction_blocking_count) {
  uint reason_detail;
  float acc =
      LaneAhead(p, step_interval, p.snapshot.lane->length - p.snapshot.s,
                junction_blocking_count, reason_detail);
  SetAcc(p, acc, AccReason::LANE_AHEAD, reason_detail);
}
__device__ void PolicyShadowLaneAhead(Person& p, float step_interval,
                                      uint junction_blocking_count) {
  uint reason_detail;
  float acc =
      LaneAhead(p, step_interval, p.snapshot.shadow_lane->length - p.snapshot.s,
                junction_blocking_count, reason_detail);
  SetAcc(p, acc, AccReason::LANE_AHEAD_SHADOW, reason_detail);
}

// 终点前减速
__device__ void PolicyToEnd(Person& p, float step_interval) {
  SetAcc(p, IDMCarFollowAcc(p, 1e999, 0, p.snapshot.distance_to_end, 0),
         AccReason::TO_END);
}

/*变道相关*/
struct LaneChangeEnv {
  // 变道距离
  float distance;
  // 变道目标车道
  Lane* side_lane;
  // 变道目标车道的s、前后车
  float side_s;
  PersonNode *side_front, *side_back;
};

__device__ void GetLaneChangeEnv(const Person& p, LaneChangeEnv& env,
                                 float step_interval) {
  // 变道距离 = 刹车距离 + 变道次数 * 变道预留距离
  env.distance =
      p.snapshot.speed *
          (p.snapshot.speed / -p.attr.usual_braking_acc / 2 + step_interval) +
      abs(p.route_lc_offset) * p.attr.lane_change_length;
  if (p.route_lc_offset) {
    auto side = p.route_lc_offset > 0 ? LEFT : RIGHT;
    env.side_lane = p.snapshot.lane->side_lanes[side];
    env.side_s = ProjectFromLane(p.snapshot.lane, env.side_lane, p.snapshot.s);
    auto& fb = p.node.sides[side];
    env.side_front = fb[FRONT];
    env.side_back = fb[BACK];
  }
}

// 检测是否会让后车追尾
__device__ bool TestBackCrash(Person& p, float s, PersonNode* back) {
  auto& v = *back->self;
  return IDMCarFollowAcc(v, v.snapshot.lane->max_speed, p.snapshot.speed,
                         s - p.attr.length - v.snapshot.s,
                         v.attr.headway) < v.attr.max_braking_acc;
}

// 路由强制变道
__device__ void PlanNecessaryLaneChange(Person& p, LaneChangeEnv& env,
                                        float step_interval) {
  auto l = p.snapshot.lane->length, remain_s = l - p.snapshot.s;
  // 如果剩余距离足够且车道长度超过20m，那么车道前后5m不变道
  if (remain_s >= env.distance && l > LC_CHECK_LANE_MIN_LENGTH &&
      (p.snapshot.s < LC_FORBIDDEN_DISTANCE ||
       remain_s < LC_FORBIDDEN_DISTANCE)) {
    return;
  }
  // 设置lane_change_length>0之后，会在外层修改状态
  p.runtime.lc_length = Clamp(p.snapshot.speed * LC_LENGTH_FACTOR,
                              p.attr.length * 3, p.attr.lane_change_length);
  if (abs(p.route_lc_offset) == 1) {
    p.UpdateNextLane(env.side_lane);
  }
  if (env.side_front) {
    // auto d = env.side_front->s - env.side_s -
    // env.side_front->self->attr.length; if (d <
    //     1 + p.runtime.speed * p.runtime.speed / 2 / -p.attr.max_braking_acc)
    //     {
    //   p.runtime.speed = 0;
    //   p._reason_detail = 0;
    //   SetAcc(p, -1e999, AccReason::LANE_CHANGE_HARD_STOP);
    // }
    //  KraussCarFollowAcc(p, step_interval,
    //                     env.side_front->self->snapshot.speed,
    //                     env.side_front->self->attr.max_braking_acc,
    //                     env.side_front->s -
    //                     env.side_s-env.side_front->self->attr.length)
    SetAcc(
        p,
        IDMCarFollowAcc(
            p, p.snapshot.lane->max_speed, env.side_front->self->snapshot.speed,
            env.side_front->s - env.side_s - env.side_front->self->attr.length,
            p.attr.headway),
        AccReason::LANE_CHANGE_N, env.side_front->self->id);
  }
}

// 变道后的期望速度
__device__ float ExpectedSpeed(Lane* lane, PersonNode* ahead, float v,
                               float s) {
  float end = min(s + VIEW_DISTANCE, lane->length);
  float max_v = lane->max_speed;
  if (ahead) {
    float ahead_v = ahead->self->snapshot.speed;
    float ahead_s = ahead->s;
    float k = (ahead_s - s) * (v / (v - ahead_v)) / (end - s);
    if (ahead_s <= end && ahead_v < v && k < 1) {
      return max_v * k + ahead_v * (1 - k);
    }
  }
  return max_v;
}

// 自主变道
__device__ float PlanVoluntaryLaneChange(Person& p, float step_interval) {
  // 前方车道距离过近
  if (p.snapshot.lane->length - p.snapshot.s < p.attr.min_gap) {
    return 1e999;
  }
  float acc = 1e999;
  float v = ExpectedSpeed(p.snapshot.lane, p.node.front, p.snapshot.speed,
                          p.snapshot.s);
  for (auto&& side : {LEFT, RIGHT}) {
    auto& target = p.snapshot.lane->side_lanes[side];
    // 主动变道的目标车道应该允许执行DelayRoute，如果不能够delay的话，不应该纳入候选集内
    if (!target || target->restriction || LcOffset(p, target) != 0) {
      continue;
    }
    auto s = ProjectFromLane(p.snapshot.lane, target, p.snapshot.s);
    auto dv =
        ExpectedSpeed(target, p.node.sides[side][FRONT], s, p.snapshot.speed) -
        v - LC_MIN_CHANGE_SPEED_GAIN;
    if (dv > 0) {
      p.lc_motivation[side] += dv * step_interval;
    } else {
      p.lc_motivation[side] *= pow(.5, step_interval);
    }
    if (p.lc_motivation[side] >= MOTIVATION_THRESHOLD[side]) {
      // 执行主动变道
      auto* n = p.node.sides[side][BACK];
      if (n &&
          TestBackCrash(
              p, ProjectToLaneSide(p.snapshot.lane, side, p.snapshot.s), n)) {
        // 变道会让后车撞车
        continue;
      }
      n = p.node.sides[side][FRONT];
      if (n) {
        auto* v = n->self;
        // float acc = KraussCarFollowAcc(p, step_interval, v->snapshot.speed,
        //                                v->attr.max_braking_acc,
        //                                n->s - p.node.s - v->attr.length);
        acc = IDMCarFollowAcc(p, min(p.attr.max_speed, target->max_speed),
                              v->snapshot.speed,
                              n->s - p.node.s - v->attr.length, p.attr.headway);
        if (acc < LC_BRAKING_TOLORENCE) {
          // 变道会减速
          continue;
        }
      }
      p.lc_motivation[LEFT] = p.lc_motivation[RIGHT] = 0;
      // 开启变道
      p.route_lc_offset = side == LEFT ? 1 : -1;
      p.runtime.lc_length = Clamp(p.snapshot.speed * LC_LENGTH_FACTOR,
                                  p.attr.length * 3, p.attr.lane_change_length);
      p.UpdateNextLane(target);
      break;
    }
  }
  return acc;
}

// MOBIL自主变道算法
__device__ float PlanMOBILLaneChange(Person& p, float a0, float step_interval,
                                     float mobil_lc_forbidden_distance) {
  // 考虑[0]->[5]的变道
  // ---------------------
  //   [3] -> [5] -> [4]
  // ----------↑----------
  //   [2] -> [0] -> [1]
  // ---------------------
  // 要求变道后：
  // 1. [3]不会追尾[5]：预期加速度不能小于安全刹车加速度+LC_SAFE_BRAKING_BIAS
  // 2. 整体加速度提升大于阈值: da = da0 + α*(da2+da3) - β > 0

  const float alpha = 0.1, beta = 0;

  // 前方车道距离过近
  if (p.snapshot.lane->length - p.snapshot.s < LC_CHECK_LANE_MIN_LENGTH) {
    return 1e999;
  }
  float v_max = p.snapshot.lane->max_speed;
  float v0 = p.snapshot.speed;
  float s0 = p.snapshot.s - p.attr.length;
  // [1]的位置和速度
  float v1 = 1e999, s1 = 1e999;
  if (p.node.front) {
    auto& p1 = *p.node.front->self;
    v1 = p1.snapshot.speed;
    s1 = p1.snapshot.s - p1.attr.length;
  }
  // [2]的加速度变化量
  float da2 = 0;
  if (p.node.back) {
    auto& p2 = *p.node.back->self;
    da2 = IDMCarFollowAcc(p2, min(v_max, p2.attr.max_speed), v1,
                          s1 - p2.snapshot.s, p2.attr.headway) -
          IDMCarFollowAcc(p2, min(v_max, p2.attr.max_speed), v0,
                          s0 - p2.snapshot.s, p2.attr.headway);
  }
  // [5]的加速度（考虑左右两侧变道）
  float a5[] = {0, 0};
  // 加速度判别式（考虑左右两侧变道）
  float da[] = {-1, -1};
  for (auto&& side : {LEFT, RIGHT}) {
    auto& target = p.snapshot.lane->side_lanes[side];
    // 主动变道的目标车道应该在可选车道范围内
    if (!target || target->restriction || LcOffset(p, target) != 0) {
      continue;
    }
    v_max = target->max_speed;
    // [4]的位置和速度
    float v4 = 1e999, s4 = 1e999;
    if (p.node.sides[side][FRONT]) {
      auto& p4 = *p.node.sides[side][FRONT]->self;
      v4 = p4.snapshot.speed;
      s4 = p4.snapshot.s - p4.attr.length;
    }
    float s5 = ProjectFromLane(p.snapshot.lane, target, p.snapshot.s);
    // 太近了不变道
    if (s4 - s5 <
        mobil_lc_forbidden_distance + p.snapshot.speed * step_interval) {
      continue;
    }
    a5[side] = IDMCarFollowAcc(p, min(p.attr.max_speed, v_max), v4, s4 - s5,
                               p.attr.headway);
    float da3 = 0;
    if (p.node.sides[side][BACK]) {
      auto& p3 = *p.node.sides[side][BACK]->self;
      float a3 =
          IDMCarFollowAcc(p3, min(p3.attr.max_speed, v_max), v0,
                          s5 - p.attr.length - p3.snapshot.s, p3.attr.headway);
      if (a3 < p3.attr.max_braking_acc + LC_SAFE_BRAKING_BIAS) {
        continue;
      }
      da3 = a3 - IDMCarFollowAcc(p3, min(p3.attr.max_speed, v_max), v4,
                                 s4 - p3.snapshot.s, p3.attr.headway);
    }
    da[side] = max(0, a5[side] - a0 + alpha * (da2 + da3) - beta);
  }
  if (da[0] > 0 || da[1] > 0) {
    // 决定变道方向
    int side;
    if (da[0] <= 0) {
      side = 1;
    } else if (da[1] <= 0) {
      side = 0;
    } else {
      side = (da[0] + da[1]) * p.rng.Rand() > da[0] ? 1 : 0;
    }
    // 按概率变道
    if (p.rng.PTrue(0.9 * min(1, da[side]))) {
      p.route_lc_offset = side == LEFT ? 1 : -1;
      p.runtime.lc_length = Clamp(p.snapshot.speed * LC_LENGTH_FACTOR,
                                  p.attr.length * 3, p.attr.lane_change_length);
      p.UpdateNextLane(p.snapshot.lane->side_lanes[side]);
      return a5[side];
    }
  }
  return 1e999;
}

// 加速度和变道决策 (entity/person/vehicle/controller.go)
__device__ void UpdateAction(Person& p, float step_interval,
                             uint junction_blocking_count,
                             LaneChangeAlgorithm lane_change_algorithm,
                             float mobil_lc_forbidden_distance) {
  float acc;
  p.runtime.acc = 1e999;
  p._reason = AccReason::NONE;
  // 变道长度，也用于标记是否发起变道
  p.runtime.lc_length = 0;
  // 加速度决策
  PolicyToLimit(p, step_interval);
  PolicyToEnd(p, step_interval);
  // 变道时只考虑目标车道的情况
  if (p.snapshot.is_lane_changing) {
    // 变道撞车情况下强制刹车
    // if (p.shadow_node.front && p.shadow_node.front->s - p.shadow_node.s <
    //                                3 + p.runtime.speed * p.runtime.speed / 2
    //                                /
    //                                        -p.attr.max_braking_acc) {
    //   p.runtime.speed = 0;
    //   p.runtime.acc = p.attr.max_braking_acc;
    //   p._reason = AccReason::LANE_CHANGE_HARD_STOP;
    //   p._reason_detail = 1;
    // } else {
    PolicyShadowCarFollow(p, step_interval);
    PolicyShadowLaneAhead(p, step_interval, junction_blocking_count);
    // }
  } else {
    PolicyCarFollow(p, step_interval);
    PolicyLaneAhead(p, step_interval, junction_blocking_count);
  }
  // 变道决策
  if (!p.snapshot.is_lane_changing && p.snapshot.lane->parent_is_road) {
    LaneChangeEnv env{};
    GetLaneChangeEnv(p, env, step_interval);
    if (p.route_lc_offset) {
      PlanNecessaryLaneChange(p, env, step_interval);
    } else if (lane_change_algorithm != LaneChangeAlgorithm::NONE) {
      if (lane_change_algorithm == LaneChangeAlgorithm::SUMO) {
        acc = PlanVoluntaryLaneChange(p, step_interval);
      } else {
        acc = PlanMOBILLaneChange(p, p.runtime.acc, step_interval,
                                  mobil_lc_forbidden_distance);
      }
      if (acc < p.runtime.acc) {
        p.runtime.acc = acc;
        p._reason = AccReason::LANE_CHANGE_V;
      }
    }
  }

  // TODO: 路口内的处理
  Clamp_(p.runtime.acc, p.attr.max_braking_acc, p.attr.max_acc);
}

// 车辆位置更新
__device__ void RefreshRuntime(Person& p, float step_interval) {
  // 计算位移和速度
  float dv = p.runtime.acc * step_interval;
  float ds;
  if (p.runtime.speed + dv < 0) {
    // 计算减速到停止的移动距离（不能倒车）
    ds = -p.runtime.speed * p.runtime.speed / (2 * p.runtime.acc);
    p.runtime.speed = 0;
  } else {
    ds = (p.runtime.speed + dv / 2) * step_interval;
    p.runtime.speed += dv;
  }
  // if (p.runtime.lc_length > 0) {
  //   // 发起变道
  //   Lane* target_lane =
  //       p.snapshot.lane->side_lanes[p.route_lc_offset > 0 ? LEFT : RIGHT];
  //   //  --------------------------------------------
  //   //   [2] → → (lane_change_length / ds) → → [3]
  //   //  --↑-----------------------------------------
  //   //   [1]     (ignore the width)
  //   //  --------------------------------------------
  //   // 1: (snapshot.lane, snapshot.s)
  //   // 2: (target_lane, neighbor_s)
  //   // 3: (target_lane, target_s)
  //   float neighbor_s =
  //       ProjectFromLane(p.runtime.lane, target_lane, p.runtime.s);
  //   // 变道必须在当前道路内完成
  //   float target_s = min(neighbor_s + p.runtime.lc_length,
  //   target_lane->length); if (neighbor_s + ds >= target_s) {
  //     // 如果距离不足则直接完成变道
  //     FinishLaneChange(p, target_lane, neighbor_s);
  //     DriveStraightAndRefreshLocation(p, ds);
  //   } else {
  //     //  --------------------------------------------
  //     //   [ns] → → → → [ns+ds] → → → → [ts]
  //     //  --------------------------------------------
  //     //   [1]            [s]
  //     //  --------------------------------------------
  //     // ns: neighbor_s
  //     // ds: ds
  //     // ts: target_s
  //     // s: motion.s
  //     p.runtime.is_lane_changing = true;
  //     p.runtime.shadow_lane = target_lane;
  //     p.runtime.shadow_s = neighbor_s + ds;
  //     p.runtime.s = ProjectFromLane(p.runtime.shadow_lane, p.runtime.lane,
  //                                   p.runtime.shadow_s);
  //     p.runtime.lc_total_length = target_s - neighbor_s;
  //     p.runtime.lc_complete_length = ds;
  //     p.lc_dir = atan2((p.runtime.lane->width + target_lane->width) / 2,
  //                      p.runtime.lc_total_length);
  //     if (p.route_lc_offset < 0) {
  //       p.lc_dir = -p.lc_dir;
  //     }
  //   }
  // } else if (p.runtime.is_lane_changing) {
  //   // 正在变道
  //   if (p.runtime.lc_complete_length + ds >= p.runtime.lc_total_length) {
  //     // 变道完成
  //     FinishLaneChange(p, p.runtime.shadow_lane, p.runtime.shadow_s);
  //     DriveStraightAndRefreshLocation(p, ds);
  //     ClearLaneChange(p.runtime);
  //   } else {
  //     p.runtime.shadow_s += ds;
  //     p.runtime.s = ProjectFromLane(p.runtime.shadow_lane, p.runtime.lane,
  //                                   p.runtime.shadow_s);
  //     p.runtime.lc_complete_length += ds;
  //   }
  // } else {
  //   // 直行
  //   DriveStraightAndRefreshLocation(p, ds);
  // }
  if (p.runtime.lc_length > 0) {
    Lane* target_lane =
        p.snapshot.lane->side_lanes[p.route_lc_offset > 0 ? LEFT : RIGHT];
    float neighbor_s =
        ProjectFromLane(p.runtime.lane, target_lane, p.runtime.s);
    FinishLaneChange(p, target_lane, neighbor_s);
  }
  DriveStraightAndRefreshLocation(p, ds);
  p.runtime.distance_to_end = max(
      p.route.veh->distance_to_end[2 * p.route_index + p.route_in_junction] -
          p.runtime.s,
      0.f);
}

__device__ void UpdateVehicle(Person& p, float global_time, float step_interval,
                              uint junction_blocking_count,
                              uint lane_change_algorithm,
                              float mobil_lc_forbidden_distance) {
  // TODO: 限行重新申请路由
#if not NDEBUG
  assert(p.runtime.lane->parent_is_road == !p.route_in_junction);
  if (!p.route_in_junction) {
    assert(p.runtime.lane->parent_id == p.route.veh->route[p.route_index]);
  }
#endif
  UpdateAction(p, step_interval, junction_blocking_count,
               (LaneChangeAlgorithm)lane_change_algorithm,
               mobil_lc_forbidden_distance);
  RefreshRuntime(p, step_interval);
#if not NDEBUG
  if (p.route_index < p.route.veh->route.size) {
    assert(p.runtime.lane->parent_is_road == !p.route_in_junction);
    if (!p.route_in_junction) {
      assert(p.runtime.lane->parent_id == p.route.veh->route[p.route_index]);
    }
  }
#endif
#if STUCK_MONITOR
  if (abs(p.runtime.speed) < 0.01) {
    p.stuck_cnt += 1;
    if (p.stuck_cnt > 100) {
      atomicInc(&stuck_cnt, ALL_BIT);
      assert(!p.node.front || p.node.s <= p.node.front->s);
      assert(!(p.runtime.shadow_lane && p.shadow_node.front) ||
             p.shadow_node.s <= p.shadow_node.front->s);
      // printf("%d (%d,%.3f,%d,%.3f,%d) (%d,%.3f,%d,%.3f,%d)\n", p.id,
      //        p.runtime.lane->id, p.node.s,
      //        p.node.front ? p.node.front->self->id : -1,
      //        p.node.front ? p.node.front->s : -1,
      //        p.node.front ? p.node.front->is_shadow : -1,
      //        p.runtime.shadow_lane ? p.runtime.shadow_lane->id : -1,
      //        p.shadow_node.s,
      //        p.shadow_node.front ? p.shadow_node.front->self->id : -1,
      //        p.shadow_node.front ? p.shadow_node.front->s : -1,
      //        p.shadow_node.front ? p.shadow_node.front->is_shadow : -1);
    }
  } else {
    p.stuck_cnt = 0;
  }
#endif
  // printf(
  //     "Veh %d Lane %d s=%.3f/%.3f Shadow:%d, Route=%d/%d v=%.3f a = % "
  //     ".3f\n",
  //     p.id, p.runtime.lane->id, p.runtime.s, p.runtime.lane->length,
  //     p.runtime.shadow_lane ? p.runtime.shadow_lane->id : -1,
  //     p.route_index, p.route.veh->route.size, p.runtime.speed,
  //     p.runtime.acc);
  if (p.skip_to_end || p.runtime.distance_to_end <= CLOSE_TO_END) {
    p.is_end = true;
    return;
  }

  // TODO: 路口
  // 通过junction的车辆进行重路由，lane -> lane
  // 检查是否通过路口，且不能是最后一条路

  // 链表更新
  if (p.snapshot.lane != p.runtime.lane) {
    p.snapshot.lane->veh_remove_buffer.Append(&p.node);
    p.runtime.lane->veh_add_buffer.Append(&p.node);
  }
  bool last_lc = p.snapshot.is_lane_changing,
       now_lc = p.runtime.is_lane_changing;
  if (last_lc) {
    if (now_lc) {
      if (p.snapshot.shadow_lane != p.runtime.shadow_lane) {
        p.snapshot.shadow_lane->veh_remove_buffer.Append(&p.shadow_node);
        p.runtime.shadow_lane->veh_add_buffer.Append(&p.shadow_node);
      }
    } else {
      p.snapshot.shadow_lane->veh_remove_buffer.Append(&p.shadow_node);
    }
  } else if (now_lc) {
    p.runtime.shadow_lane->veh_add_buffer.Append(&p.shadow_node);
  }
}
}  // namespace person
}  // namespace moss
