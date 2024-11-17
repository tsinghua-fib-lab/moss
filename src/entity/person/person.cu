#include <cmath>
#include <cuda.h>
#include <cassert>
#include <stdexcept>
#include "containers/vector.cuh"
#include "entity/aoi/aoi.cuh"
#include "entity/lane/lane.cuh"
#include "entity/person/person.cuh"
#include "entity/person/route.cuh"
#include "entity/road/road.cuh"
#include "mem/mem.cuh"
#include "moss.cuh"
#include "protos.h"
#include "utils/color_print.h"
#include "utils/macro.h"
#include "utils/utils.cuh"

namespace moss {

__device__ void PersonState::UpdatePositionDir() {
  switch (status) {
    case PersonStatus::DRIVING: {
      lane->GetPositionDir(s, x, y, dir);
      if (shadow_lane) {
        float shadow_x, shadow_y, shadow_dir;
        shadow_lane->GetPositionDir(shadow_s, shadow_x, shadow_y, shadow_dir);
        x = shadow_x * (1 - lc_completed_ratio) + x * lc_completed_ratio;
        y = shadow_y * (1 - lc_completed_ratio) + y * lc_completed_ratio;
        dir = shadow_dir;
        if (lane == shadow_lane->side_lanes[LEFT]) {
          dir += lc_yaw;
        } else if (lane == shadow_lane->side_lanes[RIGHT]) {
          dir -= lc_yaw;
        } else {
          printf(RED("Error: shadow lane is not side lane of lane\n"));
          assert(false);
        }
        dir = AngleNorm(dir);
      }
    } break;
    case PersonStatus::WALKING: {
      lane->GetPositionDir(s, x, y, dir);
      if (!is_forward) {
        dir += PI;
      }
    } break;
    case PersonStatus::SLEEP:
    case PersonStatus::FINISHED: {
      // do nothing
    } break;
    default: {
      printf(RED("Error: unknown person status: %d\n"), status);
      assert(false);
    }
  }
}

__host__ __device__ void PersonNode::PrintDebugString() const {
  printf(
      "PersonNode: is_shadow=%d, overwritable=%d, id=%d, index=%d, "
      "person.lane.id=%d, s=%.4f, person.shadow_lane.id=%d, shadow_s=%.4f\n",
      is_shadow, overwritable, index, self->id, self->runtime.lane->id,
      self->runtime.s,
      self->runtime.shadow_lane ? int(self->runtime.shadow_lane->id) : -1,
      self->runtime.shadow_s);
}

__host__ __device__ Trip& Person::GetTrip() {
  return schedules[schedule_index].trips[trip_index];
}

__host__ __device__ bool Person::NextTrip(float t) {
  if (schedules.size == 0) {
    departure_time = 1e999;
    route = nullptr;
    return false;
  }
  departure_time = t;
  auto& s = schedules[schedule_index];
  ++trip_index;
  if (trip_index == s.trips.size) {
    trip_index = 0;
    ++schedule_index;
    if (schedule_index == schedules.size) {
      schedules.size = 0;
      schedule_index = 0;
      departure_time = 1e999;
      route = nullptr;
      return false;
    } else {
      // go to the next schedule
      s = schedules[schedule_index];
      if (s.departure >= 0) {
        departure_time = s.departure;
      } else if (s.wait >= 0) {
        departure_time += s.wait;
      }
    }
  }
  // finish schedule_index and trip_index update
  // set departure time
  trip = &s.trips[trip_index];
  if (trip->departure >= 0) {
    departure_time = trip->departure;
  } else if (trip->wait >= 0) {
    departure_time += trip->wait;
  }
  route = &trip->route;
  route_index = uint(-1);
  route_in_junction = false;
  traveling_time = 0;
  total_distance = 0;
  return true;
}

namespace person {
__global__ void Prepare(Person* persons, PersonOutput* outputs, uint size,
                        float t, uint* status_cnts) {
  uint id = THREAD_ID;
  if (id >= size) {
    return;
  }
  auto& p = persons[id];
  if (!p.enable) {
    return;
  }
  switch (p.runtime.status) {
    case PersonStatus::SLEEP: {
      atomicInc(status_cnts, ALL_BIT);
    } break;
    case PersonStatus::WALKING: {
      atomicInc(status_cnts + 1, ALL_BIT);
    } break;
    case PersonStatus::DRIVING: {
      atomicInc(status_cnts + 2, ALL_BIT);
    } break;
    case PersonStatus::FINISHED: {
      atomicInc(status_cnts + 3, ALL_BIT);
    } break;
  }
  p.snapshot = p.runtime;
  // write output
  auto& o = outputs[id];
  o.t = t;
  o.id = p.id;
  o.x = p.runtime.x;
  o.y = p.runtime.y;
  o.v = p.runtime.v;
  if (p.runtime.aoi) {
    o.parent_id = p.runtime.aoi->id;
  } else if (p.runtime.lane) {
    o.parent_id = p.runtime.lane->id;
  }
  o.direction = p.runtime.dir;
  o.status = int(p.runtime.status);
  // update position saved in node
  p.node.s = p.runtime.s;
  if ((p.node.overwritable = p.runtime.shadow_lane != nullptr)) {
    p.shadow_node.s = p.runtime.shadow_s;
    p.shadow_node.sides[0][0] = p.shadow_node.sides[0][1] =
        p.shadow_node.sides[1][0] = p.shadow_node.sides[0][1] = nullptr;
  }
  // clear side link pointers
  p.node.sides[0][0] = p.node.sides[0][1] = p.node.sides[1][0] =
      p.node.sides[0][1] = nullptr;
}

__global__ void Update(Person* persons, uint size, float t, float dt,
                       float X_MIN, float X_MAX, float Y_MIN, float Y_MAX) {
  uint index = THREAD_ID;
  if (index >= size) {
    return;
  }
  auto& p = persons[index];
  if (!p.enable) {
    return;
  }
  // 因为其他的update可能会并行修改runtime.status，这里需要使用snapshot
  switch (p.snapshot.status) {
    case PersonStatus::DRIVING: {
      assert(p.runtime.status == p.snapshot.status);
      assert(p.runtime.lane == p.snapshot.lane);
      auto is_end = UpdateVehicle(p, t, dt);
      if (is_end) {
        // go to the next trip
        auto has_next = p.NextTrip(t);
        if (!has_next) {
          p.runtime.status = PersonStatus::FINISHED;
        }
      }
    } break;
    case PersonStatus::WALKING: {
      assert(p.runtime.status == p.snapshot.status);
      assert(p.runtime.lane == p.snapshot.lane);
      auto is_end = UpdatePedestrian(p, t, dt);
      if (is_end) {
        // go to the next trip
        auto has_next = p.NextTrip(t);
        if (!has_next) {
          p.runtime.status = PersonStatus::FINISHED;
        }
      }
    } break;
    case PersonStatus::SLEEP: {
      if (t >= p.departure_time) {
        auto& next_trip = p.GetTrip();
        switch (next_trip.mode) {
          case TripMode::TRIP_MODE_DRIVE_ONLY: {
            auto* start_lane = p.runtime.lane;
            auto start_s = p.runtime.s;
            if (start_lane && start_lane->type != LaneType::LANE_TYPE_DRIVING) {
              start_lane = start_lane->parent_road->right_driving_lane;
              start_s = ProjectFromLane(p.runtime.lane, start_lane, start_s);
            }
            if (p.runtime.aoi) {
              // get start lane and s by route's road
              auto* start_road = p.route->veh->route[0];
              start_lane = start_road->right_driving_lane;
              start_s = p.runtime.aoi->GetDrivingS(start_lane);
            }
            assert(start_lane);
            assert(start_s >= 0);
            p.runtime = {.status = PersonStatus::DRIVING,
                         .lane = start_lane,
                         .s = start_s};
            p.NextVehicleRoute();
            p.runtime.lane->veh_add_buffer.Add(&p.node.add_node);
          } break;
          case TripMode::TRIP_MODE_WALK_ONLY: {
            auto* start_lane = p.runtime.lane;
            auto start_s = p.runtime.s;
            if (start_lane && start_lane->type != LaneType::LANE_TYPE_WALKING) {
              start_lane = start_lane->parent_road->walking_lane;
              start_s = ProjectFromLane(p.runtime.lane, start_lane, start_s);
            }
            if (p.runtime.aoi) {
              // get s by route's lane
              start_lane = p.route->ped->route[0].lane;
              start_s = p.runtime.aoi->GetWalkingS(start_lane);
            }
            assert(start_lane);
            assert(start_s >= 0);
            p.runtime = {.status = PersonStatus::WALKING,
                         .lane = start_lane,
                         .s = start_s};
            ++p.route_index;
            p.runtime.lane->ped_add_buffer.Add(&p.node.add_node);
          } break;
          default: {
            printf(RED("Error: unknown trip mode: %d\n"), next_trip.mode);
            assert(false);
          }
        }
      }
    } break;
    default: {
      return;
    }
  }
  p.runtime.UpdatePositionDir();
}

void Data::Init(Moss* S, const PbPersons& pb, uint person_limit) {
  M = S->mem->MValueZero<MData>();
  this->S = S;
  stream = NewStream();
  assert(person_limit <= pb.persons_size());
  persons.New(S->mem, person_limit);
  SetGridBlockSize(Prepare, persons.size, S->sm_count, g_prepare, b_prepare);
  SetGridBlockSize(Update, persons.size, S->sm_count, g_update, b_update);
  uint index = 0;
  float earliest_departure = 1e999;
  for (auto& pb : pb.persons()) {
    if (index == person_limit) break;
    auto& p = persons[index++];
    if (S->verbose && index % 1000 == 0) {
      printf("%d\r", index);
    }
    p.id = pb.id();
    p.enable = true;
    person_map[p.id] = &p;
    p.node.index = p.shadow_node.index = index - 1;
    p.node.add_node.data = &p.node;
    p.node.add_node.next = nullptr;
    p.node.remove_node.data = &p.node;
    p.node.remove_node.next = nullptr;
    p.shadow_node.add_node.data = &p.shadow_node;
    p.shadow_node.add_node.next = nullptr;
    p.shadow_node.remove_node.data = &p.shadow_node;
    p.shadow_node.remove_node.next = nullptr;
    // init attribute
    if (!pb.has_vehicle_attribute()) {
      Fatal("Error: person ", p.id, " has no vehicle attribute");
    }
    auto& veh_attr = pb.vehicle_attribute();
    p.veh_attr.length = veh_attr.length();
    if (p.veh_attr.length <= 0) {
      Fatal("Error: person ", p.id, " has non-positive length ",
            p.veh_attr.length);
    }
    p.veh_attr.width = veh_attr.width();
    if (p.veh_attr.width <= 0) {
      Fatal("Error: person ", p.id, " has non-positive width ",
            p.veh_attr.width);
    }
    p.veh_attr.max_v = veh_attr.max_speed();
    if (p.veh_attr.max_v <= 0) {
      Fatal("Error: person ", p.id, " has non-positive max speed ",
            p.veh_attr.max_v);
    }
    p.veh_attr.max_a = veh_attr.max_acceleration();
    if (p.veh_attr.max_a <= 0) {
      Fatal("Error: person ", p.id, " has non-positive max acceleration ",
            p.veh_attr.max_a);
    }
    p.veh_attr.max_braking_a = veh_attr.max_braking_acceleration();
    if (p.veh_attr.max_braking_a >= 0) {
      Fatal("Error: person ", p.id, " has positive max braking acceleration ",
            p.veh_attr.max_braking_a);
    }
    p.veh_attr.usual_a = veh_attr.usual_acceleration();
    if (p.veh_attr.usual_a <= 0) {
      Fatal("Error: person ", p.id, " has non-positive usual acceleration ",
            p.veh_attr.usual_a);
    }
    p.veh_attr.usual_braking_a = veh_attr.usual_braking_acceleration();
    if (p.veh_attr.usual_braking_a >= 0) {
      Fatal("Error: person ", p.id, " has positive usual braking acceleration ",
            p.veh_attr.usual_braking_a);
    }
    p.veh_attr.lane_change_length = pb.vehicle_attribute().lane_change_length();
    if (p.veh_attr.lane_change_length <= 0) {
      Fatal("Error: person ", p.id, " has non-positive lane change length ",
            p.veh_attr.lane_change_length);
    }
    p.veh_attr.min_gap = pb.vehicle_attribute().min_gap();
    if (p.veh_attr.min_gap <= 0) {
      Fatal("Error: person ", p.id, " has non-positive min gap ",
            p.veh_attr.min_gap);
    }
    p.veh_attr.headway = veh_attr.headway();
    if (p.veh_attr.headway <= 0) {
      Fatal("Error: person ", p.id, " has non-positive headway ",
            p.veh_attr.headway);
    }
    p.veh_attr.lane_max_v_deviation =
        veh_attr.lane_max_speed_recognition_deviation();
    if (!pb.has_pedestrian_attribute()) {
      Fatal("Error: person ", p.id, " has no pedestrian attribute");
    }
    auto& ped_attr = pb.pedestrian_attribute();
    p.ped_attr.v = ped_attr.speed();
    p.rng.SetSeed(S->seed++);

    p.node.self = p.shadow_node.self = &p;
    p.shadow_node.is_shadow = true;

    // init home
    if (pb.home().has_aoi_position()) {
      p.runtime.aoi = S->aoi.At(pb.home().aoi_position().aoi_id());
    } else if (pb.home().has_lane_position()) {
      p.runtime.lane = S->lane.At(pb.home().lane_position().lane_id());
      p.runtime.s = pb.home().lane_position().s();
    } else {
      Fatal("Error: person ", p.id, " has no home position");
    }

    // init schedule
    {
#if PERF
      p.schedules.New(S->mem, 1);
#else
      p.schedules.New(S->mem, pb.schedules_size());
#endif
      // schedule index
      uint s_index = 0;
      for (auto& pb : pb.schedules()) {
        if (pb.loop_count() != 1) {
          Fatal(
              "Error: moss does not support loop_count != 1, because all route "
              "should be pre-computed. But it can not put current pre-computed "
              "route for loop_count != 1 status. Person id: ",
              p.id);
        }
        auto& s = p.schedules[s_index++];
        if (pb.has_wait_time()) {
          s.wait = pb.wait_time();
        } else {
          s.wait = -1;
        }
        if (pb.has_departure_time()) {
          s.departure = pb.departure_time();
        } else {
          s.departure = -1;
        }
#if PERF
        s.trips.New(S->mem, 1);
#else
        s.trips.New(S->mem, pb.trips_size());
#endif
        {
          // trip index
          uint t_index = 0;
          for (auto& pb : pb.trips()) {
            // copy trip data
            auto& t = s.trips[t_index++];
            t.mode = pb.mode();
            if (pb.has_wait_time()) {
              t.wait = pb.wait_time();
            } else {
              t.wait = -1;
            }
            if (pb.has_departure_time()) {
              t.departure = pb.departure_time();
            } else {
              t.departure = -1;
            }
            // copy end position
            if (pb.end().has_aoi_position()) {
              t.end_aoi = S->aoi.At(pb.end().aoi_position().aoi_id());
              t.end_lane = nullptr;
              t.end_s = -1;
            } else {
              assert(pb.end().has_lane_position());
              t.end_aoi = nullptr;
              t.end_lane = S->lane.At(pb.end().lane_position().lane_id());
              t.end_s = pb.end().lane_position().s();
            }
            // copy route
            if (pb.routes_size() == 0) {
              Fatal("Error: person ", p.id, " has no routes for schedule ",
                    s_index, " trip ", t_index,
                    ". Please use mosstool.trip.route.pre_route or "
                    "mosstool.trip.gmns.STA to pre-compute the route.");
            }
            switch (t.mode) {
              case TripMode::TRIP_MODE_DRIVE_ONLY: {
                if (!pb.routes(0).has_driving() ||
                    pb.routes(0).driving().road_ids_size() == 0) {
                  Fatal("Error: person ", p.id,
                        " has no driving route for "
                        "schedule ",
                        s_index, " trip ", t_index);
                }
                auto& driving_route = pb.routes(0).driving();
                t.route.is_veh = true;
                t.route.veh = S->mem->MValueZero<VehicleRoute>();
                t.route.veh->route.New(S->mem, driving_route.road_ids_size());
                for (int i = 0; i < driving_route.road_ids_size(); ++i) {
                  t.route.veh->route[i] = S->road.At(driving_route.road_ids(i));
                }
                if (t.end_lane == nullptr) {
                  t.end_lane = t.route.veh->route.back()->right_driving_lane;
                  t.end_s = t.end_aoi->GetDrivingS(t.end_lane);
                }
              } break;
              case TripMode::TRIP_MODE_WALK_ONLY: {
                if (!pb.routes(0).has_walking() ||
                    pb.routes(0).walking().route_size() == 0) {
                  Fatal("Error: person ", p.id,
                        " has no walking route for "
                        "schedule ",
                        s_index, " trip ", t_index);
                }
                auto& walking_route = pb.routes(0).walking();
                t.route.is_veh = false;
                t.route.ped = S->mem->MValueZero<PedestrianRoute>();
                t.route.ped->route.New(S->mem, walking_route.route_size());
                for (int i = 0; i < walking_route.route_size(); ++i) {
                  auto& pb_seg = walking_route.route(i);
                  auto& seg = t.route.ped->route[i];
                  seg.lane = S->lane.At(pb_seg.lane_id());
                  seg.dir = pb_seg.moving_direction();
                }
                if (t.end_lane == nullptr) {
                  t.end_lane = t.route.ped->route.back().lane;
                  t.end_s = t.end_aoi->GetWalkingS(t.end_lane);
                }
              } break;
              default:
                Fatal("Error: unknown trip mode: ", t.mode);
                break;
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
    // init person state
    p.trip_index = uint(-1);
    p.NextTrip(S->time);
    earliest_departure = min(earliest_departure, p.departure_time);
    p.runtime.status = PersonStatus::SLEEP;
  }
  if (index < pb.persons_size()) {
    Warn("Actual persons: ", pb.persons_size(), " -> ", index);
  }
  persons.size = index;
  CHECK;
  Info("Earliest person departure: ", earliest_departure);

  outputs.New(S->mem, persons.size);
}

void Data::PrepareAsync() {
  if (!persons.size) {
    return;
  }
  M->status_cnts[0] = M->status_cnts[1] = M->status_cnts[2] =
      M->status_cnts[3] = 0;
  Prepare<<<g_prepare, b_prepare, 0, stream>>>(
      persons.data, outputs.data, persons.size, S->time, M->status_cnts);
}

void Data::UpdateAsync() {
  // #if STUCK_MONITOR
  //   stuck_cnt = 0;
  // #endif
  if (!persons.size) {
    return;
  }
  Update<<<g_update, b_update, 0, stream>>>(
      persons.data, persons.size, S->time, S->config.step_interval,
      S->config.x_min, S->config.x_max, S->config.y_min, S->config.y_max);
}

}  // namespace person
}  // namespace moss
