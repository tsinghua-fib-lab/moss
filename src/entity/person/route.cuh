#ifndef SRC_ENTITY_PERSON_ROUTE_H_
#define SRC_ENTITY_PERSON_ROUTE_H_

#include <string>
#include "city/routing/v2/routing.pb.h"
#include "city/routing/v2/routing_service.grpc.pb.h"
#include "city/routing/v2/routing_service.pb.h"
#include "containers/array.cuh"
#include "entity/lane/lane.cuh"
#include "protos.h"

namespace moss {
struct Road;
struct Aoi;
struct Moss;

using namespace city::routing::v2;

struct PedestrianRouteSegment {
  Lane* lane;
  MovingDirection dir;
};
struct PedestrianRoute {
  // INIT in Data::Init()
  MArr<PedestrianRouteSegment> route;
};
struct VehicleRoute {
  // the road segments of the route
  // INIT in Data::Init()
  MArr<Road*> route;
};
struct Route {
  // vehicle route or pedestrian route
  // INIT in Data::Init()
  bool is_veh;
  // segments of the route
  // INIT in Data::Init()
  union {
    PedestrianRoute* ped;
    VehicleRoute* veh;
  };
};

}  // namespace moss

#endif
