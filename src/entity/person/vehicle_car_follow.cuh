#include "entity/person/person.cuh"

namespace moss {
namespace person {

// IDM Car Following Model
__device__ float IDMCarFollowAcc(Person& p, float target_speed,
                                 float ahead_speed, float distance,
                                 float min_gap, float headway);

}  // namespace person
}  // namespace moss
