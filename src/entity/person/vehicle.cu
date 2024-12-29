#include <cassert>
#include <stdexcept>
#include "entity/person/person.cuh"
#include "entity/person/vehicle_car_follow.cuh"
#include "entity/road/road.cuh"
#include "fmt/core.h"
#include "mem/mem.cuh"
#include "moss.cuh"
#include "utils/debug.cuh"
#include "utils/utils.cuh"

namespace moss {

// 计算指定lane到路由许可车道范围的差距
// calculate the offset of the given lane to the route lane range
__host__ __device__ int LcOffset(const Person& p, const Lane* l) {
  auto offset = l->offset_on_road;
  if (offset < p.target_offset1) {
    return p.target_offset1 - offset;
  } else if (offset > p.target_offset2) {
    return p.target_offset2 - offset;
  } else {
    return 0;
  }
}

// 将给定车道转换为路由中的车道
// find out the lane in the route that corresponds to the given lane
__host__ __device__ Lane* ToRouteLane(const Person& p, Lane* l) {
  auto* road = l->parent_road;
  assert(road == p.route->veh->route[p.route_index]);
  auto offset = l->offset_on_road;
  if (offset < p.target_offset1) {
    return road->lanes[p.target_offset1];
  } else if (offset > p.target_offset2) {
    return road->lanes[p.target_offset2];
  } else {
    return l;
  }
}

// 路由前进，如果走完了返回false
// next route, if the route is finished, return false
__device__ bool Person::NextVehicleRoute() {
  if (route_index + 1 == route->veh->route.size) {
    return false;
  }
  // update the runtime lane
  if (route_index != uint(-1)) {  // not starting the route
    if (route_in_junction) {
      // junction lane -> road lane
      runtime.lane = runtime.lane->successor;
    } else {
      // road lane -> junction lane
      UpdateLaneRange();
      runtime.lane =
          FindNextJunctionLane(ToRouteLane(*this, runtime.lane), route_index);
    }
    route_in_junction = !route_in_junction;
  }
  if (!route_in_junction) {
    ++route_index;
  }
  return true;
}

// 更新路由许可车道范围
// update the target lane range of the current route segment
__host__ __device__ void Person::UpdateLaneRange() {
  if (route_index + 1 == route->veh->route.size) {
    // if the route is finished, set the lane range to the end lane
    target_offset1 = target_offset2 = trip->end_lane->offset_on_road;
  } else {
    auto* road = runtime.lane->parent_road;
    auto* next_road = route->veh->route[route_index + 1];
    bool found = false;
    for (int i = road->nrl_a; i < road->nrl_b; ++i) {
      auto& g = road->next_road_lane_groups[i];
      if (next_road->id == g.next_road_id) {
        target_offset1 = g.offset1;
        target_offset2 = g.offset2;
        found = true;
        break;
      }
    }
    if (!found) {
      printf(RED("[Error] Person[%d] cannot find lane from Road[%d] to "
                 "Road[%d]\n"),
             id, road->id, next_road->id);
      SetError(ErrorCode(ErrorType::ANY, 1));
      assert(false);
    }
  }
}

// Find the next junction lane by the current road lane and the route
__host__ __device__ Lane* Person::FindNextJunctionLane(Lane* road_lane,
                                                       uint route_index,
                                                       bool ignore_error) {
  Lane* next_lane = nullptr;
  if (route_index + 1 == route->veh->route.size) {
    if (!ignore_error) {
      printf(RED("[Error] Cannot find next lane at the end of the route\n"));
    }
    return nullptr;
  }
  // 当有多个路口内车道可供选择时，选择后继车道上车最少的
  // When there are multiple lanes to choose from in the junction,
  // choose the one with the fewest vehicles
  uint best = uint(-1);
  auto* road = route->veh->route[route_index + 1];
  for (auto& i : road_lane->successors) {
    if (i.lane->successor->parent_road == road) {
      auto c = (i.lane->successor->veh_cnt & 0xffffff) +
               (i.lane->restriction ? 0x1000000 : 0);
      if (c < best) {
        best = c;
        next_lane = i.lane;
      }
    }
  }
  if (!ignore_error && !next_lane) {
    printf(RED("[Error] Cannot reach road %d from lane %d\n"), road->id,
           runtime.lane->id);
    return nullptr;
  }

  return next_lane;
}

__host__ void Person::SetVehicleRoute(Moss* S,
                                      const std::vector<uint>& route) {
  // 1. check the person is at DRIVING state
  if (runtime.status != PersonStatus::DRIVING) {
    printf(RED("[Error] Person[%d] is not at DRIVING state\n"), id);
    throw std::runtime_error("Person is not at DRIVING state");
    return;
  }
  // 2. check the route is valid
  // 2.1. the starting road of the route is the same as the current road
  if (route_in_junction) {
    printf(RED("[Error] Person[%d] is in junction lane\n"), id);
    throw std::runtime_error("Person is in junction lane");
    return;
  }
  if (route.size() == 0) {
    printf(RED("[Error] Empty route for Person[%d]\n"), id);
    throw std::runtime_error("Empty route");
    return;
  }
  if (route[0] != runtime.lane->parent_road->id) {
    printf(RED("[Error] Person[%d] is not at the starting road of the route\n"),
           id);
    throw std::runtime_error("Person is not at the starting road of the route");
    return;
  }
  // 2.2. the ending road of the route is the same as the trip's ending road
  if (route.back() != trip->end_lane->parent_road->id) {
    printf(RED("[Error] Person[%d] is not at the ending road of the route\n"),
           id);
    throw std::runtime_error("Person is not at the ending road of the route");
    return;
  }
  // 3. set the route
  // 3.1 clear the current route
  this->route->veh->route.Free(S->mem);
  route_index = 0;
  route_in_junction = false;
  force_lc = false;
  // 3.2 set the new route
  this->route->veh->route.New(S->mem, route.size());
  for (int i = 0; i < route.size(); ++i) {
    this->route->veh->route[i] = S->road.At(route[i]);
  }
}

namespace person {
// maximum noise of acceleration (m/s^2)
const float MAX_NOISE_A = 0.5;
// threshold of zero acceleration checking (m/s^2)
const float ZERO_A_THRESHOLD = 0.1;
// the distance to judge the end of the route (unit: meter)
const float CLOSE_TO_END = 5;
// the time factor to sense ahead vehicle (unit: second)
// https://jtgl.beijing.gov.cn/jgj/94220/aqcs/139634/index.html
const float VIEW_DISTANCE_FACTOR = 12;
// the minimum distance to sense ahead vehicle (unit: meter)
const float MIN_VIEW_DISTANCE = 50;
// the distance that the vehicle is not allowed to change lane (unit: meter)
// only at the end of the lane
const float LC_FORBIDDEN_DISTANCE = 20;
// 变道长度与车速的关系
// the multiple factor of the vehicle speed to calculate the lane change
// length (unit: second)
const float LC_LENGTH_FACTOR = 3;
// 自主变道让后车刹车的加速度阈值相对其最大刹车加速度的偏差
// the threshold of the deviation of the acceleration that makes the back
// vehicle brake
const float LC_SAFE_BRAKING_BIAS = 1;

// compute the phi (front wheel steering angle) of the vehicle when changing
// lane
__host__ __device__ float GetLCPhi(float v) {
  // constant values
  // we assume the phi is 30 degrees when the vehicle is stopped (0 m/s)
  // is 5 degrees when the vehicle is driving at 25 m/s
  const float k = (5.0 - 25.0) / (25.0 - 0.0);
  const float b = 30.0;
  return max(k * v + b, 5) * PI / 180;
}

// person runtime updater
__device__ void SetAcc(Person& p, float acc, AccReason reason,
                       uint reason_detail = 0) {
  if (acc < p.acc) {
    p.acc = acc;
    p._reason = reason;
    p._reason_detail = reason_detail;
  }
}

__device__ float GetLaneMaxV(const Person& p, const Lane* lane) {
  return min(p.veh_attr.max_v,
             lane->max_speed * p.veh_attr.lane_max_v_deviation);
}

// Look ahead: sense + decision, including
// car-following, traffic light, lane restriction
__device__ void LookAhead(Person& p, PersonNode* ahead_node, Lane* lane,
                          float s, float dt) {
  // check if you are not fully enter the lane
  // but the traffic light of the lane becomes red
  // STOP!!!
  if (lane->light_state == LightState::LIGHT_STATE_RED &&
      p.snapshot.s < p.veh_attr.length) {
    SetAcc(p, p.veh_attr.max_braking_a, AccReason::RED_LIGHT);
    return;
  }

  // ahead vehicle result
  uint ahead_id = (uint)-1;
  p.ahead_dist = 1e999;

  // total view distance
  auto view_distance =
      max(MIN_VIEW_DISTANCE, p.snapshot.v * VIEW_DISTANCE_FACTOR);
  // scanned distance
  auto scan_distance = lane->length - p.snapshot.s;
  // flag to indicate whether the car following action is done.
  bool followed = false;

  auto max_v = GetLaneMaxV(p, lane);

  // if ahead_node is too close to the vehicle, maybe the two vehicles are
  // overlapped, skip the ahead_node
  if (ahead_node && ahead_node->s - s < 1e-2) {
    ahead_node = nullptr;
  }

  // check the front vehicle on the same lane
  if (ahead_node) {
    auto* ahead = ahead_node->self;
    ahead_id = ahead->id;
    p.ahead_dist = ahead_node->s - s - ahead->veh_attr.length;
    // ACC [*] consider the lane max speed
    // but ignore the lane's restriction and traffic light
    auto acc = IDMCarFollowAcc(p, max_v, ahead->snapshot.v, p.ahead_dist,
                               p.veh_attr.min_gap, p.veh_attr.headway);
    SetAcc(p, acc, AccReason::CAR_FOLLOW, ahead_id);
    followed = true;
  } else {
    SetAcc(p, IDMCarFollowAcc(p, max_v, 1e999, 1e999, 0, 0),
           AccReason::TO_LIMIT);
  }
  auto route_index = lane->parent_is_road ? p.route_index : p.route_index + 1;
  // check the next lanes
  while (scan_distance < view_distance) {
    // check both lane and vehicle
    // lane: max speed, restriction, light state
    // vehicle: car-following

    if (lane->parent_is_road) {
      lane = p.FindNextJunctionLane(lane, route_index, true);
      ++route_index;
    } else {
      lane = lane->successor;
    }
    if (!lane) {
      break;
    }
    max_v = GetLaneMaxV(p, lane);
    if (lane->restriction || !lane->parent_is_road) {
      // restriction or junction lane
      // try to keep more safe distance to the line (2m)
      // use dt as headway to predict the future position
      auto stop_acc = IDMCarFollowAcc(p, max_v, 0, scan_distance,
                                      p.veh_attr.min_gap + 2, dt);
      if (lane->restriction) {
        // ACC [*]: lane restriction
        SetAcc(p, stop_acc, AccReason::LANE_AHEAD, lane->id);
      } else {
        assert(!lane->parent_is_road);
        // traffic light
        switch (lane->light_state) {
          case LightState::LIGHT_STATE_RED: {
            // ACC [*]: red light
            SetAcc(p, stop_acc, AccReason::RED_LIGHT, lane->id);
          } break;
          case LightState::LIGHT_STATE_YELLOW: {
            // ACC [*]: yellow light
            // if the vehicle can stop before the line, stop
            if (lane->light_time * p.snapshot.v <= scan_distance) {
              SetAcc(p, stop_acc, AccReason::YELLOW_LIGHT, lane->id);
            }
          } break;
          default:
            // green light or no light, skip
            break;
        }
      }
    }

    // check the front vehicle on the new lane
    if (!followed) {
      // ACC [*]
      auto* ahead_node = lane->veh_head;
      if (ahead_node) {
        auto* ahead = ahead_node->self;
        ahead_id = ahead->id;
        p.ahead_dist =
            scan_distance + ahead->snapshot.s - ahead->veh_attr.length;
        auto acc = IDMCarFollowAcc(p, max_v, ahead->snapshot.v, p.ahead_dist,
                                   p.veh_attr.min_gap, p.veh_attr.headway);
        SetAcc(p, acc, AccReason::CAR_FOLLOW, ahead_id);
        followed = true;
      }
    }
    scan_distance += lane->length;
  }
  assert(p._reason != AccReason::NONE);
}

__device__ void PlanLaneChange(Person& p, float t, float dt) {
  // the remaining distance to the end of the lane
  float reverse_s = p.snapshot.lane->length - p.snapshot.s;
  // lane max speed
  float max_v = p.snapshot.lane->max_speed;
  // lane change distance (>= vehicle length)
  float lc_length = max(p.veh_attr.length, p.snapshot.v * LC_LENGTH_FACTOR);
  // update target lane range
  p.UpdateLaneRange();
  // route_lc_offset > 0 means the vehicle is on the left side of the target
  // lane, and should change lane to the right side
  // route_lc_offset < 0 means the vehicle is on the right side of the target
  // lane, and should change lane to the left side
  auto route_lc_offset = LcOffset(p, p.snapshot.lane);
  // check necessary lane change
  // condition 1: the vehicle should change lane
  // condition 2: there are no enough space to drive
  if (route_lc_offset != 0 && reverse_s <= lc_length * abs(route_lc_offset)) {
    // START FORCE LANE CHANGE
    p.force_lc = true;
  } else if (route_lc_offset == 0) {
    // EXIT FORCE LANE CHANGE
    p.force_lc = false;
  }
  if (p.force_lc) {
    // at the left/right side of the target lane, the vehicle should change
    // lane to the right/left side
    auto side = route_lc_offset > 0 ? RIGHT : LEFT;
    auto target = p.snapshot.lane->side_lanes[side];
    p.lc_last_t = t;
    p.lc_target = target;
    float target_s = ProjectFromLane(p.snapshot.lane, target, p.snapshot.s);
    LookAhead(p, p.node.sides[side][FRONT], target, target_s, dt);
    // slow down when forcing lane change
    SetAcc(p, p.veh_attr.usual_braking_a, AccReason::LANE_CHANGE_N);
    p.lc_phi = 0;
    // check the vehicle behind
    // because you are forcing to change lane, you should check the back
    // vehicle and drive slow to avoid rear-end
    if (p.node.sides[side][BACK]) {
      auto& back = *p.node.sides[side][BACK]->self;
      auto a3 =
          IDMCarFollowAcc(back, min(back.veh_attr.max_v, max_v), p.snapshot.v,
                          target_s - p.veh_attr.length - back.snapshot.s,
                          back.veh_attr.min_gap, back.veh_attr.headway);
      // if the back vehicle cannot stop before the vehicle, slow down
      // TODO: not good enough
      if (a3 < min(back.veh_attr.usual_braking_a + LC_SAFE_BRAKING_BIAS, -1)) {
        SetAcc(p, p.veh_attr.max_braking_a, AccReason::LANE_CHANGE_N);
      }
    }
    return;
  }

  // MOBIL

  // too close to the end of the lane
  if (reverse_s < LC_FORBIDDEN_DISTANCE) {
    return;
  }
  // too short lane changing interval (4s~6s)
  if (t - p.lc_last_t < p.rng.Rand() * 2 + 4) {
    return;
  }
  // no side lanes
  if (!p.snapshot.lane->side_lanes[LEFT] &&
      !p.snapshot.lane->side_lanes[RIGHT]) {
    return;
  }

  // GO TO MOBIL [*]
  // Consider the lane change motivation from [0] to [5]
  // ---------------------
  //   [3] -> [5] -> [4]
  // ----------↑----------
  //   [2] -> [0] -> [1]
  // ---------------------
  // 要求变道后：
  // Required after lane change:
  // 1. [3]不会追尾[5]：预期加速度不能小于安全刹车加速度+LC_SAFE_BRAKING_BIAS
  // 1. [3] will not rear-end [5]: the expected acceleration cannot be less
  // than the safe braking acceleration + LC_SAFE_BRAKING_BIAS
  // 2. 整体加速度提升大于阈值: da = da0 + α*(da2+da3) > 0
  // 2. The overall acceleration increase is greater than the threshold: da =
  // da0 + α*(da2+da3) > 0
  // 3. 变道后只能接近可达到下一道路的车道集合，不能远离或离开
  // 3. After changing lanes, you can only approach the set of lanes that can
  // reach the next road, not move away or leave

  const float alpha = 0.1;
  // [0]
  float v0 = p.snapshot.v;
  float s0_head = p.snapshot.s;
  float s0_tail = s0_head - p.veh_attr.length;
  // [1]
  float v1 = 1e999, s1_tail = 1e999;
  if (p.node.front) {
    auto& p1 = *p.node.front->self;
    v1 = p1.snapshot.v;
    s1_tail = p1.snapshot.s - p1.veh_attr.length;
  }
  float a0 =
      IDMCarFollowAcc(p, min(max_v, p.veh_attr.max_v), v1, s1_tail - s0_head,
                      p.veh_attr.min_gap, p.veh_attr.headway);
  // [2]
  float da2 = 0;
  if (p.node.back) {
    auto& p2 = *p.node.back->self;
    // if [0] changes lane, compute the acceleration difference for [2]
    da2 = IDMCarFollowAcc(p2, min(max_v, p2.veh_attr.max_v), v1,
                          s1_tail - p2.snapshot.s, p2.veh_attr.min_gap,
                          p2.veh_attr.headway) -
          IDMCarFollowAcc(p2, min(max_v, p2.veh_attr.max_v), v0,
                          s0_tail - p2.snapshot.s, p2.veh_attr.min_gap,
                          p2.veh_attr.headway);
  }
  // [5] (both side)
  float a5[] = {0, 0};
  float s5[] = {0, 0};
  // // 加速度判别式（考虑左右两侧变道）
  float da[] = {0, 0};
  for (auto&& side : {LEFT, RIGHT}) {
    auto& target = p.snapshot.lane->side_lanes[side];
    if (!target || target->restriction) {
      // no side lane or the side lane is restricted, skip
      continue;
    }
    if (route_lc_offset == 0) {
      if (LcOffset(p, target) != 0) {
        // if the vehicle is in the route lane range,
        // the target lane is out of the route lane range, skip
        continue;
      }
    } else {
      if (route_lc_offset * (side == LEFT ? 1 : -1) > 0) {
        // if the vehicle is changing lane to the opposite side, skip
        // means more far from the target lane
        continue;
      }
    }
    float max_v = target->max_speed;
    // [4]
    float v4 = 1e999, s4_tail = 1e999;
    if (p.node.sides[side][FRONT]) {
      auto& p4 = *p.node.sides[side][FRONT]->self;
      v4 = p4.snapshot.v;
      s4_tail = p4.snapshot.s - p4.veh_attr.length;
    }
    float s5_head = ProjectFromLane(p.snapshot.lane, target, p.snapshot.s);
    s5[side] = s5_head;
    a5[side] =
        IDMCarFollowAcc(p, min(p.veh_attr.max_v, max_v), v4, s4_tail - s5_head,
                        p.veh_attr.min_gap, p.veh_attr.headway);
    // [3]
    float da3 = 0;
    if (p.node.sides[side][BACK]) {
      auto& p3 = *p.node.sides[side][BACK]->self;
      float s5_tail = s5_head - p.veh_attr.length;
      float a3 = IDMCarFollowAcc(p3, min(p3.veh_attr.max_v, max_v), v0, s5_tail,
                                 p3.veh_attr.min_gap, p3.veh_attr.headway);
      if (a3 < p3.veh_attr.max_braking_a + LC_SAFE_BRAKING_BIAS) {
        // [3] will rear-end [5], skip
        continue;
      }
      da3 = a3 - IDMCarFollowAcc(p3, min(p3.veh_attr.max_v, max_v), v4,
                                 s4_tail - p3.snapshot.s, p3.veh_attr.min_gap,
                                 p3.veh_attr.headway);
    }
    da[side] = max(0, a5[side] - a0 + alpha * (da2 + da3));
  }

  // Follow: Shuo Feng, Xintao Yan, Haowei Sun, Yiheng Feng, and Henry X Liu.
  // Intelligent driving intelligence test for autonomous vehicles with
  // naturalistic and adversarial environment. Nature communications,
  // 12(1):748, 2021.
  float u = da[0] + da[1];
  float p_lc = 2e-8;
  if (u >= 1) {
    p_lc = 0.9;
  } else if (u > 0) {
    p_lc = (0.9 - 2e-8) * u;
  } else {
    // da[0] == da[1] == 0
    if (p.snapshot.lane->side_lanes[LEFT]) {
      da[LEFT] = 1;
    }
    if (p.snapshot.lane->side_lanes[RIGHT]) {
      da[RIGHT] = 1;
    }
  }
  // start lane change by probability
  if (p.rng.PTrue(p_lc)) {
    // choose side by probability
    u = da[LEFT] + da[RIGHT];
    float random = p.rng.Rand() * u;
    int side = random < da[LEFT] ? LEFT : RIGHT;
    p.lc_target = p.snapshot.lane->side_lanes[side];
    SetAcc(p, a5[side], AccReason::LANE_CHANGE_V);
    LookAhead(p, p.node.sides[side][FRONT], p.lc_target, s5[side], dt);
    p.lc_last_t = t;
    p.lc_phi = 0;
  }
}

// 加速度和变道决策 (entity/person/vehicle/controller.go)
__device__ void UpdateAction(Person& p, float t, float dt) {
  p.acc = p.veh_attr.max_a + MAX_NOISE_A;
  p.lc_target = nullptr;
  p.lc_phi = 0;
  p._reason = AccReason::NONE;

  // decide acceleration
  LookAhead(p, p.node.front, p.snapshot.lane, p.snapshot.s, dt);
  if (p.snapshot.shadow_lane) {
    // if changing lane, check the shadow lane
    LookAhead(p, p.shadow_node.front, p.snapshot.shadow_lane,
              p.snapshot.shadow_s, dt);
  }
  // decide whether to change lane
  if (!p.snapshot.shadow_lane && p.snapshot.lane->parent_is_road) {
    PlanLaneChange(p, t, dt);
  }
  // decide angle when lane changing
  if (p.snapshot.shadow_lane) {
    p.lc_phi = GetLCPhi(p.snapshot.v);
  }

  Clamp_(p.acc, p.veh_attr.max_braking_a, p.veh_attr.max_a);
  // add small noise to break perfect symmetry
  auto noise = MAX_NOISE_A * (2 * p.rng.Rand() - 1);
  // do not disturb zero acceleration, do not change the sign of acceleration
  if (abs(p.acc) >= ZERO_A_THRESHOLD && p.acc * (p.acc + noise) > 0) {
    p.acc += noise;
  }
}

// refresh vehicle runtime
__device__ void RefreshRuntime(Person& p, float dt, float* out_ds,
                               bool* out_skip_to_end) {
  p.traveling_time += dt;
  bool skip_to_end = false;
  // compute ds and dv
  float dv = p.acc * dt;
  float d;
  if (p.runtime.v + dv < 0) {
    // compute the distance to stop (no reverse)
    p.runtime.v = 0;
    d = -p.runtime.v * p.runtime.v / (2 * p.acc);
  } else {
    p.runtime.v += dv;
    d = (p.runtime.v + dv / 2) * dt;
  }
  p.total_distance += d;

  // adopt Ackermann steering geometry

  float width = (p.runtime.lane->width + (p.lc_target ? p.lc_target->width
                                          : p.runtime.shadow_lane
                                              ? p.runtime.shadow_lane->width
                                              : p.runtime.lane->width)) /
                2;
  float max_yaw = min(PI / 6, asinf(width / p.veh_attr.length));
  float dyaw = d / (p.veh_attr.length / 2) * tan(p.lc_phi);

  float yaw = p.snapshot.shadow_lane ? p.snapshot.lc_yaw : 0;
  float old_yaw = yaw;
  yaw += dyaw;
  if (yaw > max_yaw) {
    yaw = max_yaw;
  }
  float mean_yaw = (old_yaw + yaw) / 2;
  // compute vertical offset
  float dw = d * sinf(mean_yaw);
  // compute forward distance
  float ds = d * cosf(mean_yaw);

  // update runtime

  if (p.lc_target) {
    assert(p.lc_target->type == LaneType::LANE_TYPE_DRIVING);
    // prepare to change lane
    if (p.runtime.shadow_lane) {
      // changing lane, reset it
      // Status 1: the new target is the current lane, do nothing
      // Status 2: the new target is the shadow lane, completed_ratio = 1 -
      // completed_ratio
      // Status 3: the new target is the other neighbor of the
      // shadow lane, put the vehicle back to the shadow lane (revert the lane
      // changing)
      // Status 4: the new target is the other neighbor of the lane, finish
      // the lane changing
      if (p.lc_target == p.runtime.lane) {
        // status 1
      } else if (p.lc_target == p.runtime.shadow_lane) {
        // status 2
        p.runtime.lc_completed_ratio = 1 - p.runtime.lc_completed_ratio;
        // swap lane and shadow lane
        auto tmp = p.runtime.lane;
        p.runtime.lane = p.runtime.shadow_lane;
        p.runtime.shadow_lane = tmp;
        // swap s and shadow_s
        auto tmp_s = p.runtime.s;
        p.runtime.s = p.runtime.shadow_s;
        p.runtime.shadow_s = tmp_s;
      } else if (p.lc_target == p.runtime.shadow_lane->side_lanes[LEFT] ||
                 p.lc_target == p.runtime.shadow_lane->side_lanes[RIGHT]) {
        // status 3
        p.runtime.lc_completed_ratio = 0;
        p.runtime.lane = p.lc_target;
        p.runtime.s = ProjectFromLane(p.runtime.shadow_lane, p.runtime.lane,
                                      p.runtime.shadow_s);
        // do not need to update shadow_s
      } else if (p.lc_target == p.runtime.lane->side_lanes[LEFT] ||
                 p.lc_target == p.runtime.lane->side_lanes[RIGHT]) {
        // status 4
        p.runtime.shadow_lane = p.runtime.lane;
        // do not need to update shadow_s
        p.runtime.lc_completed_ratio = 0;
        p.runtime.lane = p.lc_target;
        p.runtime.s = ProjectFromLane(p.runtime.shadow_lane, p.runtime.lane,
                                      p.runtime.shadow_s);
      } else {
        printf(RED("[Error]vehicle: invalid lane change target\n"));
        assert(false);
      }
    } else {
      // start lane changing, map the current vehicle position to the target
      //  --------------------------------------------
      //   [2] → → (lane_change_length / ds) → → [3]
      //  --↑-----------------------------------------
      //   [1]     (ignore the width)
      //  --------------------------------------------
      // 1: motion.lane + motion.s
      // 2: target_lane + neighbor_s
      // 3: target_lane + target_s
      p.runtime.shadow_lane = p.runtime.lane;
      p.runtime.shadow_s = p.runtime.s;
      p.runtime.lc_completed_ratio = 0;
      p.runtime.lane = p.lc_target;
      p.runtime.s = ProjectFromLane(p.runtime.shadow_lane, p.runtime.lane,
                                    p.runtime.shadow_s);
    }
  }

  // Drive straight and refresh location
  auto& s = p.runtime.s;
  auto& lane = p.runtime.lane;
  s += ds;
  // if go out of the lane, go into the next lane
  if (s > lane->length) {
    // clean lane change related data
    p.runtime.shadow_lane = nullptr;
    p.runtime.shadow_s = 0;
    p.runtime.lc_yaw = 0;
    p.runtime.lc_completed_ratio = 0;
    while (s > lane->length) {
      s -= lane->length;
      // next route
      if (!p.NextVehicleRoute()) {
        skip_to_end = true;
        break;
      }
    }
    p.runtime.s = s;
  }
  if (p.runtime.shadow_lane) {
    // is lane changing
    float total_width =
        (p.runtime.lane->width + p.runtime.shadow_lane->width) / 2;
    float ratio = p.runtime.lc_completed_ratio + dw / total_width;
    if (ratio >= 1) {
      // finish lane changing
      p.runtime.shadow_lane = nullptr;
      p.runtime.shadow_s = 0;
      p.runtime.lc_yaw = 0;
      p.runtime.lc_completed_ratio = 0;
    } else {
      p.runtime.lc_completed_ratio = ratio;
      p.runtime.shadow_s =
          ProjectFromLane(p.runtime.lane, p.runtime.shadow_lane, p.runtime.s);
      p.runtime.lc_yaw = yaw;
    }
  }

  *out_ds = ds;
  *out_skip_to_end = skip_to_end;
}

__device__ bool UpdateVehicle(Person& p, float t, float dt) {
#if not NDEBUG
  assert(p.runtime.lane->parent_is_road == !p.route_in_junction);
  if (!p.route_in_junction) {
    assert(p.runtime.lane->parent_road == p.route->veh->route[p.route_index]);
  }
#endif
  UpdateAction(p, t, dt);
  float ds;
  bool skip_to_end;
  RefreshRuntime(p, dt, &ds, &skip_to_end);
#if not NDEBUG
  if (p.route_index < p.route->veh->route.size) {
    assert(p.runtime.lane->parent_is_road == !p.route_in_junction);
    if (!p.route_in_junction) {
      assert(p.runtime.lane->parent_road == p.route->veh->route[p.route_index]);
    }
  }
#endif
#if STUCK_MONITOR
  if (abs(p.runtime.v) < 0.01) {
    p.stuck_cnt += 1;
    if (p.stuck_cnt > 50) {
      // atomicInc(&stuck_cnt, ALL_BIT);
      assert(!p.node.front || p.node.s <= p.node.front->s);
      assert(!(p.runtime.shadow_lane && p.shadow_node.front) ||
             p.shadow_node.s <= p.shadow_node.front->s);
      printf("%d %d (%d,%.3f,%d,%.3f,%d) (%d,%.3f,%d,%.3f,%d) (%.3f,%d,%d)\n",
             p.id, p.runtime.status, p.runtime.lane->id, p.node.s,
             p.node.front ? p.node.front->self->id : -1,
             p.node.front ? p.node.front->s : -1,
             p.node.front ? p.node.front->is_shadow : -1,
             p.runtime.shadow_lane ? p.runtime.shadow_lane->id : -1,
             p.shadow_node.s,
             p.shadow_node.front ? p.shadow_node.front->self->id : -1,
             p.shadow_node.front ? p.shadow_node.front->s : -1,
             p.shadow_node.front ? p.shadow_node.front->is_shadow : -1, p.acc,
             p._reason, p._reason_detail);
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
  //     p.route_index, p.route->veh->route.size, p.runtime.speed,
  //     p.runtime.acc);

  // check if the vehicle is at the end
  if (skip_to_end ||
      p.runtime.lane->parent_road == p.trip->end_lane->parent_road &&
          p.trip->end_s - p.runtime.s <= CLOSE_TO_END) {
    // END!
    p.runtime = {
        .lane = p.trip->end_lane,
        .s = p.trip->end_s,
        .aoi = p.trip->end_aoi,
        .v = 0,
    };
    // delete vehicle from the lane's linked list
    p.snapshot.lane->veh_remove_buffer.Add(&p.node.remove_node);
    if (p.snapshot.shadow_lane) {
      p.snapshot.shadow_lane->veh_remove_buffer.Add(&p.shadow_node.remove_node);
    }
    return true;
  }

  // update the linked list
  if (p.snapshot.lane != p.runtime.lane) {
    p.snapshot.lane->veh_remove_buffer.Add(&p.node.remove_node);
    p.runtime.lane->veh_add_buffer.Add(&p.node.add_node);
  }
  if (!p.snapshot.InShadowLane() && !p.runtime.InShadowLane()) {
    // do nothing
  } else if (p.snapshot.InShadowLane() && !p.runtime.InShadowLane()) {
    // remove old
    p.snapshot.shadow_lane->veh_remove_buffer.Add(&p.shadow_node.remove_node);
  } else if (!p.snapshot.InShadowLane() && p.runtime.InShadowLane()) {
    // add new
    p.runtime.shadow_lane->veh_add_buffer.Add(&p.shadow_node.add_node);
  } else {
    if (p.snapshot.shadow_lane != p.runtime.shadow_lane) {
      // delete old
      p.snapshot.shadow_lane->veh_remove_buffer.Add(&p.shadow_node.remove_node);
      // add new
      p.runtime.shadow_lane->veh_add_buffer.Add(&p.shadow_node.add_node);
    }
  }

  return false;
}
}  // namespace person
}  // namespace moss
