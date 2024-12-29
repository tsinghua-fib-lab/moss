#include "entity/person/person.cuh"
#include "utils/utils.cuh"
#include "entity/person/vehicle_car_follow.cuh"

namespace moss {
namespace person {

// IDM theta parameter
const float IDM_THETA = 4;

// IDM Car Following Model
__device__ float IDMCarFollowAcc(Person& p, float target_speed,
                                 float ahead_speed, float distance,
                                 float min_gap, float headway) {
  if (distance <= 0) {
    // collision
    return -1e999;
  }
  auto v = p.snapshot.v;
  // https://en.wikipedia.org/wiki/Intelligent_driver_model
  auto s =
      min_gap + max(0.0, v * (headway + (v - ahead_speed) / 2.f /
                                          sqrt(-p.veh_attr.usual_braking_a *
                                               p.veh_attr.max_a)));
  auto acc = p.veh_attr.max_a *
             (1 - pow(v / target_speed, IDM_THETA) - pow(s / distance, 2));
  return Clamp<float>(acc, p.veh_attr.max_braking_a, p.veh_attr.max_a);
}

}  // namespace person
}  // namespace moss
