#include <cmath>
#include <cuda.h>
#include <cassert>
#include <stdexcept>
#include "containers/vector.cuh"
#include "entity/aoi/aoi.cuh"
#include "entity/entity.h"
#include "entity/lane/lane.cuh"
#include "entity/person/person.cuh"
#include "entity/road/road.cuh"
#include "mem/mem.cuh"
#include "moss.cuh"
#include "output.cuh"
#include "protos.h"
#include "rpc/routing.cuh"
#include "utils/color_print.h"
#include "utils/macro.h"
#include "utils/utils.cuh"

namespace moss {
__device__ __host__ Trip& Person::GetTrip() {
  return schedules[schedule_index].trips[trip_index];
}

__device__ float Person::GetDepartureTime() {
  if (schedules.size == 0) {
    return 1e999;
  }
  auto& t = GetTrip();
  if (t.departure > 0) {
    if (loop_counter > 0) {
      printf(RED("Warning: departure time used in loop.\n"));
    }
    return t.departure;
  }
  return last_trip_end_time + t.wait;
}

__device__ bool Person::NextTrip(float time) {
  if (schedules.size == 0) {
    return false;
  }
  last_trip_end_time = time;
  auto& s = schedules[schedule_index];
  ++trip_index;
  if (trip_index == s.trips.size) {
    trip_index = 0;
    ++loop_counter;
    if (s.loop_count > 0 && loop_counter >= s.loop_count) {
      loop_counter = 0;
      ++schedule_index;
      if (schedule_index == schedules.size) {
        // TODO: 释放内存？
        schedules.size = 0;
        schedule_index = 0;
        return false;
      } else {
        auto t = schedules[schedule_index].departure;
        if (t > 0) {
          last_trip_end_time = t;
        } else {
          last_trip_end_time += schedules[schedule_index].wait;
        }
      }
    }
  }
  return true;
}

// 处理预定义的路由，代码与routing中的基本一致
void ProcessPredefinedRoute(Moss* S, Person& p, const PbPosition& home,
                            routing::Response& res, const PbSchedule& pb) {
  res.ok = res.ready = res.is_veh = true;
  if (pb.trips().size() == 0) {
    Fatal("Error: person ", p.id, " has no trips.");
  }
  if (pb.trips(0).routes().size() == 0) {
    Fatal("Error: person ", p.id, " has no pre-defined routes, please use mosstool.trip.route.pre_route or mosstool.trip.gmns.STA to pre-compute the route.");
  }
  if (pb.trips(0).routes(0).driving().road_ids().size() == 0) {
    Fatal("Error: person ", p.id, "'s route has no road ids.");
  }
  auto& rids = pb.trips(0).routes(0).driving().road_ids();
  auto* v = S->mem->MValueZero<routing::VehicleRoute>();
  res.veh = v;
  auto& r = v->route;
  auto n = rids.size();
  r.New(S->mem, n);
  for (int i = 0; i < n; ++i) {
    r[i] = rids[i];
  }
  auto& d = v->distance_to_end;
  d.New(S->mem, 2 * n - 1);
  auto& end = pb.trips(0).end();
  if (end.has_aoi_position()) {
    Aoi* a = v->end_aoi = S->aoi.At(end.aoi_position().aoi_id());
    auto road_id = rids[n - 1];
    bool flag = false;
    for (auto& g : a->driving_gates) {
      if (g.lane->parent_id == road_id) {
        v->end_s = g.s;
        v->end_x = g.x;
        v->end_y = g.y;
        v->end_lane = g.lane;
        flag = true;
        break;
      }
    }
    if (!flag) {
      Fatal("Error: person ", p.id, ": Aoi[", a->id,
            "] has no driving gate to Road[", road_id, "]");
    }
  } else if (end.has_lane_position()) {
    v->end_s = end.lane_position().s();
    v->end_lane = S->lane.At(end.lane_position().lane_id());
    v->end_lane->GetPosition(v->end_s, v->end_x, v->end_y);
  } else {
    Fatal("Error: person ", p.id,
          "has neither aoi_position nor lane position for end.");
  }
  if (home.has_aoi_position()) {
    auto* a = S->aoi.At(home.aoi_position().aoi_id());
    auto road_id = rids[0];
    auto flag = false;
    for (auto& g : a->driving_gates) {
      if (g.lane->parent_id == road_id) {
        p.runtime.lane = v->start_lane = g.lane;
        flag = true;
        break;
      }
    }
    if (!flag) {
      Fatal("Error: person ", p.id, ": Aoi[", a->id,
            "] has no driving gate to Road[", road_id, "]");
    }
  } else if (home.has_lane_position()) {
    p.runtime.status = PersonStatus::TO_INSERT;
    p.runtime.lane = S->lane.At(home.lane_position().lane_id());
    p.runtime.s = home.lane_position().s();
    p.start_time = pb.departure_time();
  } else {
    Fatal("Error: person ", p.id,
          "has neither aoi_position nor lane position for home.");
  }
  for (int i = n - 2; i >= 0; --i) {
    uint nr = rids[i + 1];
    bool reachable = false;
    for (auto* l : S->road.At(rids[i])->lanes) {
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
      Fatal("Error: person ", p.id, ": Cannot reach Road[", nr, "] from Road[",
            rids[i], "]");
    }
  }
  double s = v->end_s;
  d.back() = (float)s;
  for (int i = d.size - 2; i >= 0; --i) {
    d[i] = s += d[i];
  }
}

namespace person {
__global__ void Prepare(Person* persons, uint size, uint* veh_cnt,
                        uint* ped_cnt, uint* crowd_cnt, float* traveling_time) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& p = persons[id];
  if (!p.enable) {
    return;
  }
  switch (p.runtime.status) {
    case PersonStatus::WALKING:
      atomicInc(ped_cnt, ALL_BIT);
      break;
    case PersonStatus::DRIVING:
      atomicInc(veh_cnt, ALL_BIT);
      atomicAdd(traveling_time, p.traveling_or_departure_time);
      break;
    case PersonStatus::CROWD:
      atomicInc(crowd_cnt, ALL_BIT);
      break;
    default:
      p.snapshot.status = p.runtime.status;
      return;
  }
  p.snapshot = p.runtime;
  // 更新结点位置
  p.node.s = p.runtime.s;
  if ((p.node.overwritable = p.runtime.is_lane_changing)) {
    p.shadow_node.s = p.runtime.shadow_s;
    p.shadow_node.sides[0][0] = p.shadow_node.sides[0][1] =
        p.shadow_node.sides[1][0] = p.shadow_node.sides[0][1] = nullptr;
  }
  // 清空支链
  p.node.sides[0][0] = p.node.sides[0][1] = p.node.sides[1][0] =
      p.node.sides[0][1] = nullptr;
  // TODO: ResetScheduleIfNeed
}

__global__ void Update(Person* persons, uint size, float global_time,
                       float step_interval, float X_MIN, float X_MAX,
                       float Y_MIN, float Y_MAX,
                       DVector<output::AgentOutput>* agent_output,
                       bool is_python_api, bool enable_api_output,
                       int* veh_lane, float* veh_speed, float* veh_distance,
                       float* veh_total_distance, uint junction_blocking_count,
                       uint* finished_cnt, float* finished_traveling_time,
                       uint lane_change_algorithm,
                       float mobil_lc_forbidden_distance) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& p = persons[id];
  if (!p.enable) {
    return;
  }
  bool update_xydir = false;
  // 因为其他的update可能会并行修改runtime.status，这里需要使用snapshot
  switch (p.snapshot.status) {
    case PersonStatus::DRIVING: {
      assert(p.runtime.status == p.snapshot.status);
      assert(p.runtime.lane == p.snapshot.lane);
      UpdateVehicle(p, global_time, step_interval, junction_blocking_count,
                    lane_change_algorithm, mobil_lc_forbidden_distance);
      if (is_python_api) {
        // 更新车辆走过的距离
        // 由于变道误差以及路口内走不通车道的误差，结果可能是负的
        veh_total_distance[id] +=
            max(0.f, p.snapshot.distance_to_end - p.runtime.distance_to_end);
      }
#if !PERF
      if (p.skip_to_end) {
        printf(RED("[Warning]skipToEnd: vehicle %u from snapshot to runtime\n"),
               p.id);
      }
#endif
      // 处理到达终点的情况
      if (p.is_end) {
        p.is_end = false;
        p.runtime.lane = nullptr;
        p.runtime.s = p.route.veh->end_s;
        p.runtime.x = p.route.veh->end_x;
        p.runtime.y = p.route.veh->end_y;
        p.runtime.speed = 0;
        // 清除变道
        ClearLaneChange(p.runtime);
        // 删除车辆
        p.snapshot.lane->veh_remove_buffer.Append(&p.node);
        if (p.snapshot.shadow_lane) {
          p.snapshot.shadow_lane->veh_remove_buffer.Append(&p.shadow_node);
        }
        if (is_python_api || !p.route.veh->end_aoi) {
          // 不进入AOI
          p.traveling_or_departure_time += global_time;
          p.runtime.status = PersonStatus::FINISHED;
          atomicInc(finished_cnt, ALL_BIT);
          atomicAdd(finished_traveling_time, p.traveling_or_departure_time);
          veh_lane[id] = -1;
          veh_speed[id] = -1;
          veh_distance[id] = -1;
        } else {
          // 进入AOI
          p.NextTrip(global_time);
          auto* a = p.runtime.aoi = p.route.veh->end_aoi;
          a->person_add_buffer.Append(&p);
          p.runtime.status =
              a->enable_indoor ? PersonStatus::CROWD : PersonStatus::SLEEP;
        }
      } else {
        update_xydir = true;
      }
    } break;
    case PersonStatus::WALKING: {
      assert(!is_python_api);  // TODO
      assert(p.runtime.status == p.snapshot.status);
      assert(p.runtime.lane == p.snapshot.lane);
      UpdatePedestrian(p, global_time, step_interval);
      if (p.is_end) {
        p.is_end = false;
        p.runtime.lane = nullptr;
        p.runtime.s = p.route.ped->end_s;
        p.runtime.x = p.route.ped->end_x;
        p.runtime.y = p.route.ped->end_y;
        p.runtime.speed = 0;
        // 删除行人
        p.snapshot.lane->ped_remove_buffer.Append(&p.node);
        // 进入AOI
        p.NextTrip(global_time);
        auto* a = p.runtime.aoi = p.route.ped->end_aoi;
        a->person_add_buffer.Append(&p);
        p.runtime.status =
            a->enable_indoor ? PersonStatus::CROWD : PersonStatus::SLEEP;
      } else if (agent_output || enable_api_output) {
        // 计算(x,y,dir)
        p.runtime.lane->GetPositionDir(p.runtime.s, p.runtime.x, p.runtime.y,
                                       p.runtime.dir);
        if (!p.runtime.is_forward) {
          p.runtime.dir += PI;
        }
      }
    } break;
    case PersonStatus::CROWD:
      break;  // TODO: output
    case PersonStatus::TO_INSERT: {
      // 不从AOI出发，直接插入到路上
      if (global_time >= p.start_time) {
        p.InitVehicleOnLane(global_time);
        update_xydir = true;
      }
    } break;
    default:
      p.route_changed = false;
      return;
  }
  p.route_changed = false;
  if ((agent_output || enable_api_output) && update_xydir) {
    // 计算(x,y,dir)
    p.runtime.lane->GetPositionDir(p.runtime.s, p.runtime.x, p.runtime.y,
                                   p.runtime.dir);
    if (p.runtime.is_lane_changing) {
      float x, y, dir,
          k = p.runtime.lc_complete_length / p.runtime.lc_total_length;
      p.runtime.shadow_lane->GetPositionDir(p.runtime.shadow_s, x, y, dir);
      p.runtime.x = p.runtime.x * (1 - k) + x * k;
      p.runtime.y = p.runtime.y * (1 - k) + y * k;
      p.runtime.dir += k > 0.5 ? AngleLerp(AngleNorm(dir - p.runtime.dir),
                                           p.lc_dir, 4 * (1 - k) * (1 - k))
                               : AngleLerp(0.f, p.lc_dir, 4 * k * k);
    }
  }
  // 文件输出
  if (agent_output) {
    auto x = p.runtime.x, y = p.runtime.y;
    if (X_MIN <= x && x <= X_MAX && Y_MIN <= y && y <= Y_MAX) {
      auto& o = agent_output->GetAppend();
      o.type = p.runtime.status == PersonStatus::DRIVING
                   ? output::AgentOutputType::VEHICLE
                   : output::AgentOutputType::PEDESTRIAN;
      o.p_id = p.id;
      o.lane_id = p.runtime.lane ? p.runtime.lane->id : unsigned(-1);
      o.s = p.runtime.s;
      o.x = p.runtime.x;
      o.y = p.runtime.y;
      o.dir = p.runtime.dir;
    }
  }
  if (enable_api_output && p.runtime.status == PersonStatus::DRIVING) {
    veh_lane[id] = p.runtime.lane ? int(p.runtime.lane->index) : -1;
    veh_speed[id] = p.runtime.speed;
    veh_distance[id] = p.runtime.s;
  }
}

void Data::Init(Moss* S, const PbAgents& agents, uint agent_limit) {
  M = S->mem->MValueZero<MData>();
  this->S = S;
  stream = NewStream();
  assert(agent_limit <= agents.persons_size());
  persons.New(S->mem, agent_limit);
  SetGridBlockSize(Prepare, persons.size, S->sm_count, g_prepare, b_prepare);
  SetGridBlockSize(Update, persons.size, S->sm_count, g_update, b_update);
  M->finished_cnt = 0;
  M->finished_traveling_time = 0;
  if (S->is_python_api) {
    veh_lane.New(S->mem, agent_limit);
    veh_lane.Fill(-1);
    veh_speed.New(S->mem, agent_limit);
    veh_speed.Fill(-1);
    veh_distance.New(S->mem, agent_limit);
    veh_distance.Fill(-1);
    veh_total_distance.New(S->mem, agent_limit);
    veh_total_distance.Fill(0);
  }
  uint index = 0;
  float earliest_departure = 1e999;
  bool warn_aoi = false;
  for (auto& pb : agents.persons()) {
    if (S->is_python_api) {
      if (pb.schedules_size() == 0 || pb.schedules(0).trips_size() == 0 ||
          pb.schedules(0).trips(0).mode() != TripMode::TRIP_MODE_DRIVE_ONLY) {
        continue;
      }
    }
    if (index == agent_limit) break;
    auto& p = persons[index++];
    if (S->verbose && index % 1000 == 0) {
      printf("%d\r", index);
    }
    p.id = pb.id();
    p.enable = true;
    person_map[p.id] = &p;
    p.node.index = p.shadow_node.index = index - 1;
    // 初始化属性
    auto& attr = pb.attribute();
    // p.attr.type = attr.type();
    p.attr.length = attr.length();
    p.attr.width = attr.width();
    p.attr.max_speed = attr.max_speed();
    p.attr.max_acc = attr.max_acceleration();
    p.attr.max_braking_acc = attr.max_braking_acceleration();
    p.attr.usual_acc = attr.usual_acceleration();
    p.attr.usual_braking_acc = attr.usual_braking_acceleration();
    p.attr.lane_change_length = pb.vehicle_attribute().lane_change_length();
    p.attr.min_gap = pb.vehicle_attribute().min_gap();
    // TODO: 暂时以width项代替headway
    p.attr.headway = p.attr.width;
    p.rng.SetSeed(S->seed++);
    // TODO: 为数值属性添加随机扰动

    p.node.self = p.shadow_node.self = &p;
    p.shadow_node.is_shadow = true;

    // 初始化schedule
    {
#if PERF
      p.schedules.New(S->mem, 1);
#else
      p.schedules.New(S->mem, pb.schedules_size());
#endif
      uint index = 0;
      for (auto& pb : pb.schedules()) {
        auto& s = p.schedules[index++];
        s.loop_count = pb.loop_count();
        s.wait = pb.wait_time();
        s.departure = pb.departure_time();
#if PERF
        s.trips.New(S->mem, 1);
#else
        s.trips.New(S->mem, pb.trips_size());
#endif
        {
          uint index = 0;
          for (auto& pb : pb.trips()) {
            auto& t = s.trips[index++];
            t.mode = pb.mode();
            t.wait = pb.wait_time();
            t.departure = pb.departure_time();
            if ((t.end.is_aoi = pb.end().has_aoi_position())) {
              t.end.id = pb.end().aoi_position().aoi_id();
              t.end.s = -1;
            } else {
              t.end.id = pb.end().lane_position().lane_id();
              t.end.s = pb.end().lane_position().s();
            }
#if PERF
            break;
#endif
          }
        }
#if PERF
        break;
#endif
      }
    }
    auto t = pb.schedules(0).departure_time();
    p.last_trip_end_time = t;
    earliest_departure = min(earliest_departure, t);
    // 把人放到AOI里面
    auto& home = pb.home();
    uint aoi_id = 0;
    if (home.has_aoi_position()) {
      aoi_id = home.aoi_position().aoi_id();
      if (S->aoi.aoi_map.find(aoi_id) == S->aoi.aoi_map.end()) {
        Fatal("Error: person ", p.id, "'s home aoi ", aoi_id, " not found.");
      }
      p.runtime.status = PersonStatus::INIT;
      p.runtime.aoi = S->aoi.aoi_map[aoi_id];
      p.runtime.aoi->person_add_buffer.AppendCpu(&p);
    } else if (!S->is_python_api) {
      // TODO: 支持不以AOI为起点和终点的导航
      Fatal("Error: person ", p.id, " has no home aoi position.");
    }
    if (S->is_python_api) {
      if (!warn_aoi && home.has_aoi_position()) {
        Warn("Using aoi_position in API mode may lead to unexpected results!");
        warn_aoi = true;
      }
      ProcessPredefinedRoute(S, p, home, p.route, pb.schedules(0));
    } else if (S->config.pre_routing) {
      auto& trip = p.GetTrip();
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
      auto& r = p.route;
      r.waiting = true;
      r.ready = false;
      (*S->routing.d_post)[index - 1] = {
          .type = type,
          .start = {.is_aoi = true, .id = aoi_id, .s = 0},
          .end = trip.end,
          .resp = &r};
    }
  }
  if (index < agents.persons_size()) {
    Warn("Actual persons: ", agents.persons_size(), " -> ", index);
  }
  persons.size = index;
  CHECK;
  Info("Earliest person departure: ", earliest_departure);
  if (S->is_python_api) {
    // 车辆不会再次进入AOI，无需分配空间
    for (auto& a : S->aoi.aois) {
      a.person_add_buffer.is_fixed = true;
    }
  } else {
    for (auto& a : S->aoi.aois) {
      a.person_add_buffer.ExpandCpu(100);
      a.person_add_buffer.is_fixed = true;
    }
  }
}

void Data::PrepareAsync() {
  if (!persons.size) {
    return;
  }
  M->veh_cnt = M->ped_cnt = M->crowd_cnt = M->traveling_time = 0;
  Prepare<<<g_prepare, b_prepare, 0, stream>>>(
      persons.data, persons.size, &M->veh_cnt, &M->ped_cnt, &M->crowd_cnt,
      &M->traveling_time);
}

void Data::UpdateAsync() {
#if STUCK_MONITOR
  stuck_cnt = 0;
#endif
  if (!persons.size) {
    return;
  }
  Update<<<g_update, b_update, 0, stream>>>(
      persons.data, persons.size, S->time, S->config.step_interval,
      S->config.x_min, S->config.x_max, S->config.y_min, S->config.y_max,
      S->output.option == output::Option::AGENT ? &S->output.M->agent_output
                                                : nullptr,
      S->is_python_api, S->enable_api_output, veh_lane.data, veh_speed.data,
      veh_distance.data, veh_total_distance.data,
      S->config.junction_blocking_count, &M->finished_cnt,
      &M->finished_traveling_time, S->config.lane_change_algorithm,
      S->config.mobil_lc_forbidden_distance);
}

void Data::Save(std::vector<PersonCheckpoint>& state) {
  state.resize(persons.size);
  for (int i = 0; i < persons.size; ++i) {
    auto& p = persons[i];
    auto& s = state[i];
    s.snapshot = p.snapshot;
    s.runtime = p.runtime;
    s.node = p.node;
    s.shadow_node = p.shadow_node;
    s.schedule_index = p.schedule_index;
    s.trip_index = p.trip_index;
    s.loop_counter = p.loop_counter;
    s.start_time_or_last_trip_end_time = p.start_time;
    s.route = p.route;
    s.route_changed = p.route_changed;
    s.route_in_junction = p.route_in_junction;
    s.route_index = p.route_index;
    s.route_lc_offset = p.route_lc_offset;
    s.route_l1 = p.route_l1;
    s.route_l2 = p.route_l2;
    s.next_lane = p.next_lane;
    s.lc_dir = p.lc_dir;
    s.lc_motivation[0] = p.lc_motivation[0];
    s.lc_motivation[1] = p.lc_motivation[1];
    s.rng = p.rng;
    s.traveling_or_departure_time = p.traveling_or_departure_time;
    s.enable = p.enable;
  }
}

void Data::Load(const std::vector<PersonCheckpoint>& state) {
  assert(state.size() == persons.size);
  for (int i = 0; i < persons.size; ++i) {
    auto& p = persons[i];
    auto& s = state[i];
    p.snapshot = s.snapshot;
    p.runtime = s.runtime;
    p.node = s.node;
    p.shadow_node = s.shadow_node;
    p.schedule_index = s.schedule_index;
    p.trip_index = s.trip_index;
    p.loop_counter = s.loop_counter;
    p.start_time = s.start_time_or_last_trip_end_time;
    p.route = s.route;
    p.route_changed = s.route_changed;
    p.route_in_junction = s.route_in_junction;
    p.route_index = s.route_index;
    p.route_lc_offset = s.route_lc_offset;
    p.route_l1 = s.route_l1;
    p.route_l2 = s.route_l2;
    p.next_lane = s.next_lane;
    p.lc_dir = s.lc_dir;
    p.lc_motivation[0] = s.lc_motivation[0];
    p.lc_motivation[1] = s.lc_motivation[1];
    p.rng = s.rng;
    p.traveling_or_departure_time = s.traveling_or_departure_time;
    p.enable = s.enable;
  }
}
}  // namespace person
}  // namespace moss
