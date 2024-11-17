#include <cassert>
#include "entity/aoi/aoi.cuh"
#include "entity/person/person.cuh"

namespace moss::person {

__device__ bool UpdatePedestrian(Person& p, float t, float dt) {
  bool is_end;
  float s = p.snapshot.s;
  auto& segments = p.route->ped->route;
  auto seg = segments[p.route_index];
  assert(s >= 0 && s <= seg.lane->length);
  // TODO: 骑车
  float v = p.ped_attr.v;
  if (p.snapshot.lane->IsNoEntry()) {
    v *= 2;  // red light, go faster
  }
  float ds = v * dt;
  // printf("Ped %d Lane %d forward %d s %.3f+%.3f/%.3f\n", p.id, seg.lane->id,
  //        p.runtime.is_forward, s, ds, seg.lane->length);

  // add increment to s
  if (seg.dir == MovingDirection::MOVING_DIRECTION_FORWARD) {
    s += ds;
  } else {
    s -= ds;
  }
  // for loop to update s, person position, route_index until s is in the range
  while (true) {
    bool shouldNext = s < 0 || s > seg.lane->length;
    if (!shouldNext) {
      break;
    }
    // check if the next segment is a no entry lane
    if (p.route_index + 1 < segments.size) {
      if (segments[p.route_index + 1].lane->IsNoEntry()) {
        // do nothing, just return
        p.runtime.v = 0;
        return;
      }
      // go to the next segment
      if (s < 0) {
        s = -s;
      } else if (s > seg.lane->length) {
        s -= seg.lane->length;
      }
      ++p.route_index;
      seg = segments[p.route_index];
      if (seg.dir == MovingDirection::MOVING_DIRECTION_FORWARD) {
        // do nothing
      } else {
        // reverse the s
        s = seg.lane->length - s;
      }
    } else {
      // there is no next segment, the person is at the end of the route
      is_end = true;
      break;
    }
  }
  // check if the person is at the end of the route
  auto is_forward = seg.dir == MovingDirection::MOVING_DIRECTION_FORWARD;
  if (!is_end && p.route_index >= segments.size - 1 &&
      (is_forward ? s >= p.trip->end_s : s <= p.trip->end_s)) {
    is_end = true;
  }
  // clamp s to the range
  s = Clamp<float>(s, 0, seg.lane->length);
  if (is_end) {
    p.runtime = {
        .lane = p.trip->end_lane,
        .s = p.trip->end_s,
        .aoi = p.trip->end_aoi,
        .v = 0,
    };
    if (p.runtime.lane) {
      p.runtime.lane->GetPosition(p.runtime.s, p.runtime.x, p.runtime.y);
    } else {
      assert(p.runtime.aoi);
      p.runtime.x = p.runtime.aoi->x;
      p.runtime.y = p.runtime.aoi->y;
    }
    // delete the person from the current lane
    p.snapshot.lane->ped_remove_buffer.Add(&p.node.remove_node);
    return true;
  }
  // update runtime and others
  p.runtime.s = s;
  p.runtime.v = v;
  p.runtime.is_forward = is_forward;
  p.runtime.lane = seg.lane;
  // update lane's linked list
  if (p.runtime.lane != p.snapshot.lane) {
    p.snapshot.lane->ped_remove_buffer.Add(&p.node.remove_node);
    p.runtime.lane->ped_add_buffer.Add(&p.node.add_node);
  }
  return false;
}
}  // namespace moss::person
