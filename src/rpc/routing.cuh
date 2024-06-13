#ifndef SRC_RPC_ROUTING_H_
#define SRC_RPC_ROUTING_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <string>
#include <thread>
#include "city/routing/v2/routing.pb.h"
#include "city/routing/v2/routing_service.grpc.pb.h"
#include "city/routing/v2/routing_service.pb.h"
#include "containers/array.cuh"
#include "containers/vector.cuh"
#include "entity/entity.h"
#include "protos.h"

namespace moss {
struct Lane;
struct Road;
struct Aoi;
struct Moss;

namespace routing {
using namespace city::routing::v2;

struct Response;
struct Request {
  RouteType type;
  Position start, end;
  Response* resp;
};
struct PedestrianRouteSegment {
  Lane* lane;
  MovingDirection dir;
};
struct PedestrianRoute {
  MArrZ<PedestrianRouteSegment> route;
  float end_s, end_x, end_y;
  Aoi* end_aoi;
  // TODO: 公交线路
};
struct VehicleRoute {
  Lane *start_lane, *end_lane;
  // 车辆依次经过的road_id，大小为n
  MArrZ<uint> route;
  // 在每一段路由开头所看到的到终点的距离，大小为2n-1
  // 由于车辆实际选择的车道并不确定，结果不一定准确
  MArrZ<float> distance_to_end;
  Aoi* end_aoi;
  float end_s, end_x, end_y;
};
struct Response {
  bool waiting;  // 已发送请求并正在等待回复
  bool ready;    // 回复就绪
  bool ok;       // 路由成功
  bool is_veh;   // 是车辆路由
  union {
    const PedestrianRoute* ped;
    const VehicleRoute* veh;
  };
};
struct Data {
  DVector<Request>* d_post;
  std::queue<Request> requests;
  std::mutex requests_mutex;
  std::condition_variable requests_cv;
  std::vector<std::pair<Response*, Response>> responses;
  std::mutex responses_mutex;
  std::condition_variable responses_cv;
  std::unique_ptr<RoutingService::Stub> stub;
  std::vector<std::thread> workers;
  std::atomic<bool> stop;
  uint not_replied, not_replied_total;
  std::unordered_map<uint64_t, Response> cache;
  std::shared_mutex cache_mutex;
  Moss* S;

  void Init(Moss* S, const std::string& url, uint num_workers);
  void Prepare();
  void ProcessRequests();
  void StopWorkers();
  void worker();
};
}  // namespace routing
}  // namespace moss

#endif
