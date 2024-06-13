#ifndef SRC_PROTO_H_
#define SRC_PROTO_H_

#include "city/agent/v2/agent.pb.h"
#include "city/geo/v2/geo.pb.h"
#include "city/map/v2/map.pb.h"
#include "city/routing/v2/routing.pb.h"
#include "city/routing/v2/routing_service.grpc.pb.h"
#include "city/routing/v2/routing_service.pb.h"
#include "city/trip/v2/trip.pb.h"

namespace moss {
// 地图
using PbAoi = city::map::v2::Aoi;
using PbMap = city::map::v2::Map;
using PbTl = city::map::v2::TrafficLight;
using PbLane = city::map::v2::Lane;
using PbLaneType = city::map::v2::LaneType;
using PbRoad = city::map::v2::Road;
using PbJunction = city::map::v2::Junction;
using PbAgents = city::agent::v2::Agents;
using PbAgent = city::agent::v2::Agent;
using PbSchedule = city::trip::v2::Schedule;
using AgentType = city::agent::v2::AgentType;
using TripMode = city::trip::v2::TripMode;
using LightState = city::map::v2::LightState;

// 导航
using PbGetRouteRequest = city::routing::v2::GetRouteRequest;
using PbGetRouteResponse = city::routing::v2::GetRouteResponse;
using LaneType = city::map::v2::LaneType;
using LaneTurn = city::map::v2::LaneTurn;
using RouteType = city::routing::v2::RouteType;
using MovingDirection = city::routing::v2::MovingDirection;
using JourneyType = ::city::routing::v2::JourneyType;

using PbPosition = city::geo::v2::Position;
}  // namespace moss
#endif
