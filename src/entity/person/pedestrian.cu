#include <cassert>
#include "entity/person/person.cuh"

namespace simulet::person {
// const float BIKE_SPEED = 4;
const float DEFAULT_DESIRED_SPEED_ON_LANE = 1.34;
// const float MAX_NOISE_ON_PEDESTRIAN_SPEED = .5;

__device__ void UpdatePedestrian(Person& p, float global_time,
                                 float step_interval) {
  float s = p.snapshot.s;
  auto& r = p.route.ped->route;
  auto seg = r[p.route_index];
  assert(s >= 0 && s <= seg.lane->length);
  bool next_lane = false;
  bool red_light = false;
  // TODO: 骑车
  float ds = DEFAULT_DESIRED_SPEED_ON_LANE * step_interval;
  // printf("Ped %d Lane %d forward %d s %.3f+%.3f/%.3f\n", p.id, seg.lane->id,
  //        p.runtime.is_forward, s, ds, seg.lane->length);
  float length;
  // dir_forward:1 dir_backward:2 -> is_forward:1 is_backward:0
  // 这个赋值是在初始化new的时候做的
  // p.runtime.is_forward = (seg.dir ==
  // MovingDirection::MOVING_DIRECTION_FORWARD);
  if (p.snapshot.is_forward) {
    s = s + ds;
    length = seg.lane->length;
    if (s > length) {
      if (p.route_index + 1 < r.size &&
          r[p.route_index + 1].lane->IsNoEntry()) {
        red_light = true;
        s = length;
      } else {
        next_lane = true;
        while (true) {
          s -= length;
          ++p.route_index;
          if (p.route_index >= r.size) {
            p.is_end = true;
            break;
          }
          length = r[p.route_index].lane->length;
          if (s <= length) {
            break;
          }
        }
      }
    }
  } else {
    s = s - ds;
    if (s <= 0) {
      if (p.route_index + 1 < r.size &&
          r[p.route_index + 1].lane->IsNoEntry()) {
        red_light = true;
        s = 0;
      } else {
        next_lane = true;
        length = 0;
        while (s + length <= 0) {
          s += length;
          ++p.route_index;
          if (p.route_index >= r.size) {
            p.is_end = true;
            break;
          }
          length = r[p.route_index].lane->length;
        };
        s = -s;
      }
    }
  }
  if (!p.is_end && p.route_index >= r.size - 1 &&
      (seg.dir == MovingDirection::MOVING_DIRECTION_FORWARD
           ? s >= p.route.ped->end_s
           : s <= p.route.ped->end_s)) {
    p.is_end = true;
  }
  if (p.is_end) {
    return;
  }
  if (red_light) {
    p.runtime.speed = 0;
  } else {
    p.runtime.speed = DEFAULT_DESIRED_SPEED_ON_LANE;
    if (next_lane) {
      seg = r[p.route_index];
      p.runtime.is_forward =
          seg.dir == MovingDirection::MOVING_DIRECTION_FORWARD;
      if (!p.runtime.is_forward) {
        s = seg.lane->length - s;
      }
      p.runtime.lane = seg.lane;
      p.snapshot.lane->ped_remove_buffer.Append(&p.node);
      p.runtime.lane->ped_add_buffer.Append(&p.node);
    }
  }
  p.runtime.s = s;
}
}  // namespace simulet::person
