#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <atomic>
#include <unordered_map>
#include <utility>
#include <vector>
#include "entity/aoi/aoi.cuh"
#include "entity/lane/lane.cuh"
#include "entity/road/road.cuh"
#include "moss.cuh"
#include "rpc/routing.cuh"
#include "utils/color_print.h"
#include "utils/timer.h"
#include "utils/utils.cuh"

namespace moss::routing {

void Data::worker() {
  uint64_t _t = 0, _last_cnt = 0;
  Request req;
  while (!stop) {
    {
      std::unique_lock<std::mutex> lock(requests_mutex);
      while (requests.empty()) {
        requests_cv.wait(lock);
        if (stop) {
          return;
        }
      }
      req = std::move(requests.front());
      requests.pop();
    }
    assert(req.start.is_aoi);
    assert(req.end.is_aoi);
    assert(req.type == RouteType::ROUTE_TYPE_DRIVING ||
           req.type == RouteType::ROUTE_TYPE_WALKING);
    uint64_t key = ((uint64_t)(req.start.id - 5'0000'0000) << 32) | req.end.id;
    if (req.type == RouteType::ROUTE_TYPE_DRIVING) {
      key = ~key;
    }
    Response res;
    bool in_cache;
    {
      std::shared_lock<std::shared_mutex> _(cache_mutex);
      in_cache = cache.find(key) != cache.end();
    }
    if (in_cache) {
      std::shared_lock<std::shared_mutex> _(cache_mutex);
      res = cache.at(key);
    } else {
      GetRouteRequest pb_req;
      pb_req.set_type(req.type);
      pb_req.clear_start();
      if (req.start.is_aoi) {
        pb_req.mutable_start()->mutable_aoi_position()->set_aoi_id(
            req.start.id);
      } else {
        pb_req.mutable_start()->mutable_lane_position()->set_lane_id(
            req.start.id);
        pb_req.mutable_start()->mutable_lane_position()->set_s(req.start.s);
      }
      pb_req.clear_end();
      if (req.end.is_aoi) {
        pb_req.mutable_end()->mutable_aoi_position()->set_aoi_id(req.end.id);
      } else {
        pb_req.mutable_end()->mutable_lane_position()->set_lane_id(req.end.id);
        pb_req.mutable_end()->mutable_lane_position()->set_s(req.end.s);
      }
      GetRouteResponse pb_res;
      grpc::ClientContext ctx;
      auto status = stub->GetRoute(&ctx, pb_req, &pb_res);
      if (!status.ok()) {
        Fatal("gRPC failed: ", status.error_details());
      }
      res = {};
      res.ready = true;
      if (pb_res.journeys_size() == 0) {
        res.ok = false;
      } else if (pb_res.journeys_size() == 1) {
        res.ok = true;
        if (req.type == ROUTE_TYPE_DRIVING) {
          res.is_veh = true;
          auto* v = S->mem->MValueZero<VehicleRoute>();
          res.veh = v;
          auto& r = v->route;
          auto& rids = pb_res.journeys(0).driving().road_ids();
          v->start_lane = S->road.At(rids[0])->right_driving_lane;
          uint n = rids.size();
          r.New(S->mem, n);
          for (int i = 0; i < n; ++i) {
            r[i] = rids[i];
          }
          v->end_lane = S->road.At(rids[n - 1])->right_driving_lane;
          auto& d = v->distance_to_end;
          d.New(S->mem, 2 * n - 1);
          for (int i = n - 2; i >= 0; --i) {
            uint nr = rids[i + 1];
            bool check = false;
            for (auto* l : S->road.At(rids[i])->lanes) {
              if (l->type == LaneType::LANE_TYPE_DRIVING) {
                for (auto& ll : l->successors) {
                  if (ll.lane->next_road_id == nr) {
                    d[2 * i + 1] = ll.lane->length;
                    d[2 * i] = l->length;
                    check = true;
                    break;
                  }
                }
                if (check) {
                  break;
                }
              }
            }
            if (!check) {
              Fatal("Cannot reach road ", nr, " from road ", rids[i]);
            }
          }
          double s;
          if (req.end.is_aoi) {
            Aoi* a = v->end_aoi = S->aoi.At(req.end.id);
            bool check = false;
            for (auto& g : a->driving_gates) {
              if (g.lane == v->end_lane) {
                v->end_s = s = g.s;
                v->end_x = g.x;
                v->end_y = g.y;
                check = true;
                break;
              }
            }
            if (!check) {
              Fatal("End Aoi ", a->id, " has no driving gate to lane ",
                    v->end_lane->id);
            }
          } else {
            v->end_s = s = req.end.s;
            v->end_lane->GetPosition(s, v->end_x, v->end_y);
          }
          d.back() = (float)s;
          for (int i = d.size - 2; i >= 0; --i) {
            d[i] = s += d[i];
          }
        } else if (req.type == ROUTE_TYPE_WALKING) {
          res.is_veh = false;
          auto* p = S->mem->MValueZero<PedestrianRoute>();
          res.ped = p;
          auto& r = p->route;
          auto& pb = pb_res.journeys(0).walking().route();
          r.New(S->mem, pb.size());
          uint index = 0;
          for (auto& pb : pb) {
            auto& s = r[index++];
            s.lane = S->lane.At(pb.lane_id());
            s.dir = pb.moving_direction();
          }
          bool check = false;
          auto* l = r.back().lane;
          auto* a = p->end_aoi =
              S->aoi.At(pb_req.end().aoi_position().aoi_id());
          for (auto& g : a->walking_gates) {
            if (g.lane == l) {
              p->end_s = g.s;
              p->end_x = g.x;
              p->end_y = g.y;
              check = true;
              break;
            }
          }
          if (!check) {
            Fatal("End Aoi ", a->id, " has no walking gate to lane ", l->id);
          }
        } else {
          Fatal("Unknown route type: ", req.type);
        }
      } else {
        Fatal("Cannot handle journey_size=", pb_res.journeys_size());
      }
      {
        std::unique_lock<std::shared_mutex> _(cache_mutex);
        cache[key] = res;
      }
    }
    {
      std::lock_guard lock(responses_mutex);
      responses.push_back({req.resp, res});
      not_replied -= 1;
      if (not_replied % 10000 == 0) {
        if (_t == 0) {
          _last_cnt = not_replied;
          _t = Time();
        } else {
          auto __t = Time();
          auto _s = double(_last_cnt - not_replied) / (__t - _t) * 1'000'000;
          auto eta = (int)round(not_replied / _s);
          printf("Routing: %d/%d %.2fHz ETA: %02d:%02d        \r",
                 not_replied_total - not_replied, not_replied_total, _s,
                 eta / 60, eta % 60);
          if (not_replied == 0) {
            _t = 0;
          } else {
            _last_cnt = not_replied;
            _t = __t;
          }
        }
      }
    }
    if (not_replied == 0) {
      responses_cv.notify_all();
    }
  }
}

void Data::Init(Moss* S, const std::string& url, uint num_workers) {
  this->S = S;
  d_post = S->mem->MValueZero<DVector<routing::Request>>();
  d_post->mem = S->mem;
  if (S->config.pre_routing) {
    auto s = S->config.agent_limit;
    d_post->Reserve(s + 1000);
    d_post->size = s;
  } else {
    d_post->Reserve(2000000);
  }
  stub = RoutingService::NewStub(
      grpc::CreateChannel(url, grpc::InsecureChannelCredentials()));
  stop = false;
  for (uint i = 0; i < num_workers; ++i) {
    workers.emplace_back(&Data::worker, this);
  }
}

void Data::Prepare() {
  if (!not_replied_total) {
    return;
  }
  std::unique_lock<std::mutex> lock(responses_mutex);
  while (not_replied) {
    responses_cv.wait(lock);
  }
  printf("\n");
  not_replied_total = 0;
  for (auto&& [i, j] : responses) {
    *i = j;
  }
  responses.clear();
}

void Data::ProcessRequests() {
  if (d_post->size) {
    {
      std::lock_guard lock(requests_mutex);
      for (auto& i : *d_post) {
        requests.push(std::move(i));
      }
      {
        std::lock_guard lock(responses_mutex);
        not_replied += d_post->size;
        not_replied_total += d_post->size;
      }
      d_post->Clear();
    }
    requests_cv.notify_all();
    S->mem->PrintUsage();
  }
}

void Data::StopWorkers() {
  stop = true;
  requests_cv.notify_all();
  for (auto& w : workers) {
    w.join();
  }
}
}  // namespace moss::routing
