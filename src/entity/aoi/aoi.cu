#include <algorithm>
#include <unordered_map>
#include <utility>
#include "entity/aoi/aoi.cuh"
#include "entity/aoi/crowd/crowd.cuh"
#include "entity/lane/lane.cuh"
#include "entity/person/person.cuh"
#include "output.cuh"
#include "protos.h"
#include "rpc/routing.cuh"
#include "simulet.cuh"
#include "utils/color_print.h"
#include "utils/macro.h"
#include "utils/utils.cuh"

namespace simulet {

float Aoi::GetDrivingS(Lane* lane) {
  for (auto& g : driving_gates) {
    if (g.lane == lane) {
      return g.s;
    }
  }
  assert(false);
  return -1;
}

namespace aoi {

__device__ float CheckDrivingLaneAvailable(Person* p, Aoi& a,
                                           bool out_control) {
  // 根据导航信息读出从aoi离开后的lane
  auto* lane = p->route.veh->start_lane;
  for (auto& g : a.driving_gates) {
    if (g.lane == lane) {
      if (out_control) {
        // 检查前车
        if (g.observation->front) {
          auto* front = g.observation->front;
          if (front->s - g.s < front->self->attr.length + p->attr.min_gap) {
            return -1;
          }
        }
        // 检查后车
        if (g.observation->back) {
          auto* back = g.observation->back;
          auto& speed = back->self->snapshot.speed;
          auto& acc = back->self->attr.usual_braking_acc;
          if (g.s - p->attr.length - back->s <= speed * speed / 2 / acc) {
            return -1;
          }
        }
      }
      return g.s;
    }
  }
  assert(false);
  return -1;
}

__global__ void Prepare(Aoi* aois, uint size) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& a = aois[id];
#if DETERMINISTIC
  SortById(a.person_add_buffer);
#endif
  // 处理person_add_buffer中插入的人送入sleeping
  for (auto* p : a.person_add_buffer) {
    if (a.enable_indoor) {
      if (p->runtime.status == PersonStatus::INIT) {
        auto& pos = a.crowd.rng.Choose(a.crowd.sleep_points);
        p->runtime.x = pos.x;
        p->runtime.y = pos.y;
        HeapPush(a.sleeping_person, {p->GetDepartureTime(), p});
      } else {
        a.crowd.WalkToSleep(p);
      }
    } else {
      HeapPush(a.sleeping_person, {p->GetDepartureTime(), p});
    }
  }
  a.person_add_buffer.Clear();
  if (a.enable_indoor) {
    crowd::Prepare(a);
  }
}

__device__ void MakeRouteRequest(Person* p, uint aoi_id,
                                 DVector<routing::Request>* d_post) {
  auto& trip = p->GetTrip();
  RouteType type;
  switch (trip.mode) {
    case TripMode::TRIP_MODE_WALK_ONLY:
    case TripMode::TRIP_MODE_BIKE_WALK:
      type = RouteType::ROUTE_TYPE_WALKING;
      break;
    case TripMode::TRIP_MODE_BUS_WALK:
      type = RouteType::ROUTE_TYPE_BY_BUS;
      break;
    case TripMode::TRIP_MODE_DRIVE_ONLY:
      type = RouteType::ROUTE_TYPE_DRIVING;
      break;
    default:
      assert(false);
  }
  auto& r = p->route;
  r.waiting = true;
  r.ready = false;
  d_post->Append({.type = type,
                  .start = {.is_aoi = true, .id = aoi_id, .s = 0},
                  .end = trip.end,
                  .resp = &r});
}

__global__ void Update(Aoi* aois, uint size, float global_time,
                       float step_interval, bool out_control,
                       DVector<routing::Request>* d_post) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& a = aois[id];
  auto& awake = a.awake_person;
  auto& sleep = a.sleeping_person;
  // 检查awake_person导航是否好了，加入for_leaving
  for (int i = 0; i < awake.size; ++i) {
    auto* p = awake[i];
    auto& r = p->route;
    if (!r.ready) {
      continue;
    }
    if (r.ok) {
      if (a.enable_indoor) {
        if (r.is_veh) {
          auto* l = r.veh->start_lane;
          bool check = false;
          for (auto& g : a.driving_gates) {
            if (g.lane == l) {
              a.crowd.WalkToGate(p, &g, true);
              check = true;
              break;
            }
          }
          ASSERT(check);
        } else {
          auto* l = r.ped->route[0].lane;
          bool check = false;
          for (auto& g : a.walking_gates) {
            if (g.lane == l) {
              a.crowd.WalkToGate(p, &g, false);
              check = true;
              break;
            }
          }
          ASSERT(check);
        }
        p->runtime.status = PersonStatus::CROWD;
      } else {
        if (r.is_veh) {
          a.veh_for_leaving.Append(p);
        } else {
          a.ped_for_leaving.Append(p);
        }
        p->runtime.status = PersonStatus::WAITING_FOR_LEAVING;
      }
      p->route_index = 0;
    } else {
      if (p->NextTrip(global_time)) {
        // 跳过当前trip并进入下一trip，重新请求路由
        MakeRouteRequest(p, a.id, d_post);
        continue;
      } else {
        // 走完了，直接在sleeping中休眠
        HeapPush(sleep, {1e999, p});
      }
    }
    // 从列表中交换删除
    --awake.size;
    if (i != awake.size) {
      awake[i] = awake.data[awake.size];
    }
    --i;
  }

  // sleeping中到时间的发出导航请求并送入awake_person
  while (sleep.size && sleep[0].first <= global_time) {
    auto* p = HeapPop(sleep);
    if (!p->route.waiting && !p->route.ready) {
      MakeRouteRequest(p, a.id, d_post);
    }
    awake.Append(p);
  }

  // ped_for_leaving放行
  for (auto* p : a.ped_for_leaving) {
    p->runtime.status = PersonStatus::WALKING;
    p->runtime.aoi = nullptr;
    auto& r = p->route.ped->route[0];
    auto* l = p->runtime.lane = r.lane;
    p->runtime.dir = r.dir;
    bool check = false;
    for (auto& g : a.walking_gates) {
      if (g.lane == l) {
        p->runtime.s = g.s;
        p->runtime.is_forward = p->route.ped->route[0].dir ==
                                MovingDirection::MOVING_DIRECTION_FORWARD;
        check = true;
        break;
      }
    }
    ASSERT(check);
    l->ped_add_buffer.Append(&p->node);
  }
  a.ped_for_leaving.Clear();

  // veh_for_leaving检查AoiGate的路况放行
  // TODO: 这里暂时只考虑第一辆
  if (a.veh_for_leaving.size) {
    auto* v = a.veh_for_leaving[0];
    auto s = CheckDrivingLaneAvailable(v, a, out_control);
    if (s >= 0) {
      a.veh_for_leaving.Delete(0);
      v->runtime.lane = v->route.veh->start_lane;
      v->runtime.s = s;
      v->InitVehicleOnLane(global_time);
    }
  }
  // crowd Update
  if (a.enable_indoor) {
    crowd::Update(a, step_interval);
  }
}

void Data::Init(Simulet* S, const PbMap& map) {
  this->S = S;
  stream = NewStream();
  aois.New(S->mem, map.aois_size());
  SetGridBlockSize(Prepare, aois.size, S->sm_count, g_prepare, b_prepare);
  SetGridBlockSize(Update, aois.size, S->sm_count, g_update, b_update);
  std::unordered_map<uint, std::vector<std::pair<float, AoiGate*>>> observers;
  // 建立映射
  {
    auto* p = aois.data;
    for (auto& pb : map.aois()) {
      aoi_map[pb.id()] = p++;
    }
  }
  uint index = 0;
  for (auto& pb : map.aois()) {
    auto& a = aois[index++];
    // 基本属性
    a.id = pb.id();
    a.area = pb.area();
    // 步行出口
    a.walking_gates.New(S->mem, pb.walking_positions_size());
    {
      uint index = 0;
      for (auto& p : pb.walking_positions()) {
        auto& o = a.walking_gates[index];
        o.lane = S->lane.lane_map.at(p.lane_id());
        o.s = min(p.s(), max(0.f, o.lane->length - 1));
        o.observation = nullptr;
        auto& q = pb.walking_gates(index);
        o.x = q.x();
        o.y = q.y();
        ++index;
      }
    }
    // 开车出口
    a.driving_gates.New(S->mem, pb.driving_positions_size());
    {
      uint index = 0;
      for (auto& p : pb.driving_positions()) {
        auto& o = a.driving_gates[index];
        o.lane = S->lane.lane_map.at(p.lane_id());
        o.s = min(p.s(), max(0.f, o.lane->length - 1));
        observers[p.lane_id()].push_back({o.s, &o});
        auto& q = pb.driving_gates(index);
        o.x = q.x();
        o.y = q.y();
        ++index;
      }
    }
    // 初始化容器
    a.person_add_buffer.mem = S->mem;
    a.sleeping_person.mem = S->mem;
    a.awake_person.mem = S->mem;
    a.ped_for_leaving.mem = S->mem;
    a.veh_for_leaving.mem = S->mem;
    // crowd Init
    if (S->config.enable_aoi_indoor && a.area > 0) {
      a.enable_indoor = true;
      crowd::Init(S, pb, a);
      // if (output::enabled) {
      //   auto x = pb.positions(0).x(), y = pb.positions(0).y();
      //   if (output::X_MIN <= x && x <= output::X_MAX && output::Y_MIN <= y &&
      //       y <= output::Y_MAX) {
      //     a.enable_indoor = true;
      //     crowd::Init(pb, a);
      //   }
      // }
    }
  }
  // 为所有aoi的开车出口登记lane上的观测点
  for (auto&& [lane_id, sas] : observers) {
    auto* l = S->lane.lane_map.at(lane_id);
    std::sort(sas.begin(), sas.end(),
              [](std::pair<float, AoiGate*>& a, std::pair<float, AoiGate*>& b) {
                return a.first < b.first;
              });
    l->observers.New(S->mem, sas.size());
    l->observations.New(S->mem, sas.size());
    uint index = 0;
    for (auto&& [s, a] : sas) {
      l->observers[index] = s;
      a->observation = l->observations.data + index;
      ++index;
    }
  }
}

void Data::PrepareAsync() {
  if (!aois.size) {
    return;
  }
  Prepare<<<g_prepare, b_prepare, 0, stream>>>(aois.data, aois.size);
}

void Data::UpdateAsync() {
  if (!aois.size) {
    return;
  }
  Update<<<g_update, b_update, 0, stream>>>(aois.data, aois.size, S->time,
                                            S->config.step_interval,
                                            out_control, S->routing.d_post);
}

void Data::Save(std::vector<AoiCheckpoint>& state) {
  state.resize(aois.size);
  for (int i = 0; i < aois.size; ++i) {
    auto& a = aois[i];
    auto& s = state[i];
    a.person_add_buffer.Save(s.person_add_buffer);
    a.sleeping_person.Save(s.sleeping_person);
    a.awake_person.Save(s.awake_person);
    a.veh_for_leaving.Save(s.veh_for_leaving);
    a.ped_for_leaving.Save(s.ped_for_leaving);
  }
}

void Data::Load(const std::vector<AoiCheckpoint>& state) {
  assert(state.size() == aois.size);
  for (int i = 0; i < aois.size; ++i) {
    auto& a = aois[i];
    auto& s = state[i];
    a.person_add_buffer.Load(s.person_add_buffer);
    a.sleeping_person.Load(s.sleeping_person);
    a.awake_person.Load(s.awake_person);
    a.veh_for_leaving.Load(s.veh_for_leaving);
    a.ped_for_leaving.Load(s.ped_for_leaving);
  }
}
}  // namespace aoi
}  // namespace simulet
